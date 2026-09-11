/*
 * @Description: 传统 BLDC 反电动势过零无感六步控制器
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <string.h>
#include "motor_bldc_sensorless.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 正转 step 0..5 的悬空相依次为 C/B/A/C/B/A。 */
static const uint8_t s_floating_phase[6] = {2U, 1U, 0U, 2U, 1U, 0U};
/* Private function prototypes -----------------------------------------------*/
static bool Motor_BldcSensorless_IsFinite(float value);
static bool Motor_BldcSensorless_ConfigValid(const motor_bldc_sensorless_config_t *config);
static void Motor_BldcSensorless_ResetSector(motor_bldc_sensorless_t *control);
static void Motor_BldcSensorless_Trip(motor_bldc_sensorless_t *control,
                                      motor_bldc_sensorless_fault_t fault);
static bool Motor_BldcSensorless_Commutate(motor_bldc_sensorless_t *control, uint8_t step);
static uint8_t Motor_BldcSensorless_NextStep(const motor_bldc_sensorless_t *control);
static float Motor_BldcSensorless_StartupPeriod(const motor_bldc_sensorless_t *control);
static bool Motor_BldcSensorless_ExpectedRising(const motor_bldc_sensorless_t *control);
static bool Motor_BldcSensorless_CheckCrossing(motor_bldc_sensorless_t *control,
                                                const float phase_voltage[3]);
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 判断浮点值是否为有限数。
 * @param {float} value 待检查值
 * @return {bool} 有限值返回 true
 * @author: lkc
 */
static bool Motor_BldcSensorless_IsFinite(float value)
{
    /* 不依赖 libm：NaN 是唯一不等于自身的浮点值。 */
    return value == value && value < 3.4028234e38f && value > -3.4028234e38f;
}

/**
 * @description: 校验无感启动与过零配置。
 * @param {const motor_bldc_sensorless_config_t *} config 待校验配置
 * @return {bool} 配置有效返回 true
 * @author: lkc
 */
static bool Motor_BldcSensorless_ConfigValid(const motor_bldc_sensorless_config_t *config)
{
    return config != NULL && config->align_time_s >= 0.0f &&
           config->startup_initial_period_s > 0.0f && config->startup_final_period_s > 0.0f &&
           config->startup_initial_period_s >= config->startup_final_period_s &&
           config->startup_ramp_time_s > 0.0f && config->startup_timeout_s > config->align_time_s &&
           config->commutation_blanking_s >= 0.0f && config->zero_cross_threshold_v > 0.0f &&
           config->bemf_filter_alpha > 0.0f && config->bemf_filter_alpha <= 1.0f &&
           config->takeover_crossings > 0U && config->closed_loop_timeout_s > 0.0f;
}

/**
 * @description: 复位当前 60° 扇区的检测与调度状态。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSensorless_ResetSector(motor_bldc_sensorless_t *control)
{
    control->sector_elapsed_s = 0.0f;
    control->scheduled_delay_s = 0.0f;
    control->filtered_bemf_v = 0.0f;
    control->crossing_armed = false;
    control->crossing_seen = false;
    control->commutation_scheduled = false;
}

/**
 * @description: 锁存无感故障并停止关联六步输出。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {motor_bldc_sensorless_fault_t} fault 要锁存的故障码
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSensorless_Trip(motor_bldc_sensorless_t *control,
                                      motor_bldc_sensorless_fault_t fault)
{
    control->fault = fault;
    control->mode = MOTOR_BLDC_SENSORLESS_FAULT;
    Motor_BldcSixStep_Stop(control->commutator);
}

/**
 * @description: 发布一个换相步并复位扇区检测状态。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {uint8_t} step 换相步，范围 0..5
 * @return {bool} 发布成功返回 true
 * @author: lkc
 */
static bool Motor_BldcSensorless_Commutate(motor_bldc_sensorless_t *control, uint8_t step)
{
    control->step = (uint8_t)(step % 6U);
    Motor_BldcSensorless_ResetSector(control);
    if (!Motor_BldcSixStep_CommutateStep(control->commutator, control->step)) {
        Motor_BldcSensorless_Trip(control, MOTOR_BLDC_SENSORLESS_FAULT_COMMUTATOR);
        return false;
    }
    return true;
}

/**
 * @description: 根据转向计算下一个电气换相步。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {uint8_t} 下一个换相步，范围 0..5
 * @author: lkc
 */
static uint8_t Motor_BldcSensorless_NextStep(const motor_bldc_sensorless_t *control)
{
    if (control->direction == MOTOR_BLDC_DIRECTION_CCW) {
        return control->step == 0U ? 5U : (uint8_t)(control->step - 1U);
    }
    return (uint8_t)((control->step + 1U) % 6U);
}

/**
 * @description: 计算当前开环斜坡的 60° 换相周期。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {float} 当前换相周期 (s)
 * @author: lkc
 */
static float Motor_BldcSensorless_StartupPeriod(const motor_bldc_sensorless_t *control)
{
    float ratio = control->startup_elapsed_s / control->config.startup_ramp_time_s;
    if (ratio > 1.0f) {
        ratio = 1.0f;
    }
    return control->config.startup_initial_period_s -
           ratio * (control->config.startup_initial_period_s - control->config.startup_final_period_s);
}

/**
 * @description: 计算当前扇区期望的 BEMF 过零方向。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {bool} 上升过零返回 true，下降过零返回 false
 * @author: lkc
 */
static bool Motor_BldcSensorless_ExpectedRising(const motor_bldc_sensorless_t *control)
{
    bool rising = (control->step & 1U) != 0U;
    return control->direction == MOTOR_BLDC_DIRECTION_CCW ? !rising : rising;
}

/**
 * @description: 检查当前悬空相是否完成带迟滞的 BEMF 过零。
 * @detail: 软件中性点为三相端电压平均值；每个扇区最多接受一次过零。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {const float [3]} phase_voltage 逻辑 A/B/C 相端电压 (V)
 * @return {bool} 本周期检测到期望方向过零返回 true
 * @author: lkc
 */
static bool Motor_BldcSensorless_CheckCrossing(motor_bldc_sensorless_t *control,
                                                const float phase_voltage[3])
{
    uint8_t floating = s_floating_phase[control->step];
    float neutral = (phase_voltage[0] + phase_voltage[1] + phase_voltage[2]) / 3.0f;
    float bemf = phase_voltage[floating] - neutral;
    float threshold = control->config.zero_cross_threshold_v;
    bool rising = Motor_BldcSensorless_ExpectedRising(control);

    control->filtered_bemf_v += control->config.bemf_filter_alpha * (bemf - control->filtered_bemf_v);
    if (!control->crossing_armed) {
        if ((rising && control->filtered_bemf_v <= -threshold) ||
            (!rising && control->filtered_bemf_v >= threshold)) {
            control->crossing_armed = true;
        }
        return false;
    }
    if ((rising && control->filtered_bemf_v >= threshold) ||
        (!rising && control->filtered_bemf_v <= -threshold)) {
        control->crossing_seen = true;
        control->crossing_armed = false;
        return true;
    }
    return false;
}

/**
 * @description: 填充无感六步默认配置。
 * @param {motor_bldc_sensorless_config_t *} config 输出配置；NULL 时忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSensorless_DefaultConfig(motor_bldc_sensorless_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->align_time_s = 0.20f;
    config->startup_initial_period_s = 0.050f;
    config->startup_final_period_s = 0.003f;
    config->startup_ramp_time_s = 1.50f;
    config->startup_timeout_s = 3.0f;
    config->commutation_blanking_s = 0.00020f;
    config->zero_cross_threshold_v = 0.20f;
    config->bemf_filter_alpha = 0.20f;
    config->takeover_crossings = 6U;
    config->closed_loop_timeout_s = 0.10f;
}

/**
 * @description: 初始化无感六步控制实例。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {motor_bldc_six_step_t *} commutator 六步输出实例，不可为空
 * @param {const motor_bldc_sensorless_config_t *} config 无感配置，不可为空
 * @return {bool} 初始化成功返回 true
 * @author: lkc
 */
bool Motor_BldcSensorless_Init(motor_bldc_sensorless_t *control,
                               motor_bldc_six_step_t *commutator,
                               const motor_bldc_sensorless_config_t *config)
{
    if (control == NULL) {
        return false;
    }
    memset(control, 0, sizeof(*control));
    control->commutator = commutator;
    if (commutator == NULL || !Motor_BldcSensorless_ConfigValid(config)) {
        control->fault = MOTOR_BLDC_SENSORLESS_FAULT_INVALID_CONFIG;
        control->mode = MOTOR_BLDC_SENSORLESS_FAULT;
        return false;
    }
    control->config = *config;
    control->initialized = true;
    return true;
}

/**
 * @description: 启动无感六步定向与开环启动流程。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {motor_bldc_direction_t} direction 转向
 * @param {float} duty PWM 占空比
 * @return {bool} 成功进入定向状态返回 true
 * @author: lkc
 */
bool Motor_BldcSensorless_Start(motor_bldc_sensorless_t *control,
                                motor_bldc_direction_t direction, float duty)
{
    if (control == NULL || !control->initialized || control->fault != MOTOR_BLDC_SENSORLESS_FAULT_NONE ||
        !Motor_BldcSixStep_Start(control->commutator, direction, duty)) {
        return false;
    }
    control->direction = direction;
    control->mode = MOTOR_BLDC_SENSORLESS_ALIGN;
    control->state_elapsed_s = 0.0f;
    control->startup_elapsed_s = 0.0f;
    control->valid_crossings = 0U;
    return Motor_BldcSensorless_Commutate(control, 0U);
}

/**
 * @description: 停止无感六步状态机及关联输出。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例；NULL 时忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSensorless_Stop(motor_bldc_sensorless_t *control)
{
    if (control == NULL) {
        return;
    }
    if (control->commutator != NULL) {
        Motor_BldcSixStep_Stop(control->commutator);
    }
    control->mode = MOTOR_BLDC_SENSORLESS_IDLE;
    control->state_elapsed_s = 0.0f;
    Motor_BldcSensorless_ResetSector(control);
}

/**
 * @description: 推进一个无感六步控制周期。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {const float [3]} phase_voltage 逻辑 A/B/C 相端电压 (V)
 * @param {float} bus_voltage 直流母线电压 (V)
 * @param {float} dt_s 本控制周期时间 (s)
 * @return {bool} 本周期保持有效运行返回 true
 * @author: lkc
 */
bool Motor_BldcSensorless_Step(motor_bldc_sensorless_t *control,
                               const float phase_voltage[3], float bus_voltage, float dt_s)
{
    float period;
    uint8_t i;

    if (control == NULL || !control->initialized || phase_voltage == NULL ||
        (control->mode != MOTOR_BLDC_SENSORLESS_ALIGN &&
         control->mode != MOTOR_BLDC_SENSORLESS_OPEN_LOOP &&
         control->mode != MOTOR_BLDC_SENSORLESS_CLOSED_LOOP)) {
        return false;
    }
    if (!Motor_BldcSensorless_IsFinite(dt_s) || dt_s <= 0.0f ||
        !Motor_BldcSensorless_IsFinite(bus_voltage) || bus_voltage <= 0.0f) {
        Motor_BldcSensorless_Trip(control, MOTOR_BLDC_SENSORLESS_FAULT_INVALID_SAMPLE);
        return false;
    }
    for (i = 0U; i < 3U; i++) {
        if (!Motor_BldcSensorless_IsFinite(phase_voltage[i])) {
            Motor_BldcSensorless_Trip(control, MOTOR_BLDC_SENSORLESS_FAULT_INVALID_SAMPLE);
            return false;
        }
    }

    control->state_elapsed_s += dt_s;
    control->sector_elapsed_s += dt_s;
    if (control->mode == MOTOR_BLDC_SENSORLESS_ALIGN) {
        if (control->state_elapsed_s >= control->config.align_time_s) {
            control->mode = MOTOR_BLDC_SENSORLESS_OPEN_LOOP;
            control->state_elapsed_s = 0.0f;
            control->startup_elapsed_s = 0.0f;
            return Motor_BldcSensorless_Commutate(control, Motor_BldcSensorless_NextStep(control));
        }
        return true;
    }

    control->startup_elapsed_s += dt_s;
    if (control->mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP &&
        control->startup_elapsed_s >= control->config.startup_timeout_s) {
        Motor_BldcSensorless_Trip(control, MOTOR_BLDC_SENSORLESS_FAULT_STARTUP_TIMEOUT);
        return false;
    }

    if (control->sector_elapsed_s >= control->config.commutation_blanking_s &&
        !control->crossing_seen && Motor_BldcSensorless_CheckCrossing(control, phase_voltage)) {
        if (control->valid_crossings < 255U) {
            control->valid_crossings++;
        }
        /* 过零位于扇区中点；再等待相同时间即为延迟 30 电角度。 */
        control->scheduled_delay_s = control->sector_elapsed_s;
        control->commutation_scheduled = true;
        if (control->mode == MOTOR_BLDC_SENSORLESS_OPEN_LOOP &&
            control->valid_crossings >= control->config.takeover_crossings) {
            control->mode = MOTOR_BLDC_SENSORLESS_CLOSED_LOOP;
            control->state_elapsed_s = 0.0f;
        }
    }

    if (control->mode == MOTOR_BLDC_SENSORLESS_CLOSED_LOOP) {
        if (control->commutation_scheduled &&
            control->sector_elapsed_s >= 2.0f * control->scheduled_delay_s) {
            return Motor_BldcSensorless_Commutate(control, Motor_BldcSensorless_NextStep(control));
        }
        if (control->sector_elapsed_s >= control->config.closed_loop_timeout_s) {
            Motor_BldcSensorless_Trip(control, MOTOR_BLDC_SENSORLESS_FAULT_ZERO_CROSS_LOST);
            return false;
        }
        return true;
    }

    period = Motor_BldcSensorless_StartupPeriod(control);
    if (control->sector_elapsed_s >= period) {
        /* 本扇区未看到可信过零则清连续计数，避免偶发噪声触发接管。 */
        if (!control->crossing_seen) {
            control->valid_crossings = 0U;
        }
        return Motor_BldcSensorless_Commutate(control, Motor_BldcSensorless_NextStep(control));
    }
    return true;
}

/**
 * @description: 获取无感六步运行状态。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例
 * @return {motor_bldc_sensorless_mode_t} 当前运行状态
 * @author: lkc
 */
motor_bldc_sensorless_mode_t Motor_BldcSensorless_GetMode(const motor_bldc_sensorless_t *control)
{
    return control == NULL ? MOTOR_BLDC_SENSORLESS_FAULT : control->mode;
}

/**
 * @description: 获取无感六步故障码。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例
 * @return {motor_bldc_sensorless_fault_t} 当前故障码
 * @author: lkc
 */
motor_bldc_sensorless_fault_t Motor_BldcSensorless_GetFault(const motor_bldc_sensorless_t *control)
{
    return control == NULL ? MOTOR_BLDC_SENSORLESS_FAULT_INVALID_CONFIG : control->fault;
}
#ifdef __cplusplus
}
#endif
