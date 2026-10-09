/*
 * @Description: 默认 BLDC 六步实例、运行控制与状态查询
 * @Author: lkc
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_bldc.h"

#include <stddef.h>

#include "stm32f4xx.h"
#include "hal_motor.h"
#include "motor_bldc_defaults.h"
#include "motor_bldc_isr.h"
#include "motor_bldc_sensorless.h"
#include "motor_bldc_six_step.h"
#include "motor_protect.h"
#include "motor_detect.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_BLDC_AUTO_DUTY_MIN 0.08f
#define MOTOR_BLDC_AUTO_DUTY_MAX 0.10f
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_bldc_six_step_t s_commutator;
static motor_bldc_sensorless_t s_sensorless;
static volatile bool s_selected;
static bool s_initialized;
static bool s_sample_upward = true;
/* Private function prototypes -----------------------------------------------*/
static uint32_t Motor_Bldc_StatusLock(void);
static void Motor_Bldc_StatusUnlock(uint32_t primask);
static bool Motor_Bldc_StartInternal(motor_bldc_direction_t direction, float duty, bool fixed_open_loop,
                                     float sector_period_s);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 进入短临界区，供状态读写与 ISR 互斥。
 * @return {uint32_t} 进入前的 PRIMASK，0 表示原先允许中断
 */
static uint32_t Motor_Bldc_StatusLock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @description: 按进入临界区前的 PRIMASK 恢复中断。
 * @param {uint32_t} primask Motor_Bldc_StatusLock 的返回值
 * @return {void} 无
 */
static void Motor_Bldc_StatusUnlock(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @description: 校验方向和占空比后启动默认 BLDC 实例。
 * @detail: fixed_open_loop 为 false 时还要求占空比在 0.08～0.10，且采样侧为向上计数。
 * @param {motor_bldc_direction_t} direction 换相方向
 * @param {float} duty 目标占空比，无量纲
 * @param {bool} fixed_open_loop true=固定周期开环
 * @param {float} sector_period_s 固定换相周期 (s)，闭环启动时不使用
 * @return {bool} 启动成功返回 true
 */
static bool Motor_Bldc_StartInternal(motor_bldc_direction_t direction, float duty,
                                     bool fixed_open_loop, float sector_period_s)
{
    bool started;

    if (motor.ol_enable || Motor_Detect_IsActive() || Motor_Protect_IsFaultLatched() || duty <= 0.0f || duty > 0.25f ||
        (!fixed_open_loop && !s_sample_upward) ||
        (!fixed_open_loop && (duty < MOTOR_BLDC_AUTO_DUTY_MIN || duty > MOTOR_BLDC_AUTO_DUTY_MAX)) ||
        (direction != MOTOR_BLDC_DIRECTION_CW && direction != MOTOR_BLDC_DIRECTION_CCW)) {
        return false;
    }

    if (!s_initialized) {
        s_initialized = Motor_BldcDefaults_Init(&s_commutator, &s_sensorless);
        if (!s_initialized) {
            return false;
        }
    }
    if (s_sensorless.fault != MOTOR_BLDC_SENSORLESS_FAULT_NONE ||
        Motor_BldcSixStep_GetFault(&s_commutator) != MOTOR_BLDC_FAULT_NONE) {
        if (!Motor_BldcDefaults_Init(&s_commutator, &s_sensorless)) {
            return false;
        }
    }

    s_selected = true;
    Hal_Motor_PwmStart();
    Hal_Motor_BldcPrepare();
    if (fixed_open_loop) {
        started = Motor_BldcSensorless_StartOpenLoop(&s_sensorless, direction, duty, sector_period_s);
    } else {
        started = Motor_BldcSensorless_Start(&s_sensorless, direction, duty);
    }
    if (!started) {
        Hal_Motor_PwmStop();
        s_selected = false;
    }
    return started;
}

/**
 * @description: 以反电势闭环方式启动默认 BLDC 实例。
 * @param {motor_bldc_direction_t} direction 换相方向
 * @param {float} duty 目标占空比，无量纲，自动模式允许 0.08～0.10
 * @return {bool} 启动成功返回 true
 */
bool Motor_Bldc_Start(motor_bldc_direction_t direction, float duty)
{
    return Motor_Bldc_StartInternal(direction, duty, false, 0.0f);
}

/**
 * @description: 以固定扇区周期的开环六步启动默认 BLDC 实例。
 * @param {motor_bldc_direction_t} direction 换相方向
 * @param {float} duty 目标占空比，无量纲，允许范围 (0, 0.25]
 * @param {float} sector_period_s 固定换相周期 (s)
 * @return {bool} 启动成功返回 true
 */
bool Motor_Bldc_StartOpenLoop(motor_bldc_direction_t direction, float duty, float sector_period_s)
{
    return Motor_Bldc_StartInternal(direction, duty, true, sector_period_s);
}

/**
 * @description: 选择 BEMF 采样落在 PWM 向上或向下计数侧。
 * @param {bool} upward true=PWM 向上计数侧
 * @return {bool} 设置成功返回 true
 */
bool Motor_Bldc_SetSampleSide(bool upward)
{
    uint32_t primask;

    if (s_initialized && Motor_BldcSixStep_IsRunning(&s_commutator)) {
        return false;
    }
    primask = Motor_Bldc_StatusLock();
    s_sample_upward = upward;
    Motor_Bldc_StatusUnlock(primask);
    return true;
}

/**
 * @description: 在停机时写入 BEMF 积分门限和最短换相周期。
 * @param {float} integral_threshold_vs BEMF 积分换相门限 (V·s)
 * @param {float} minimum_period_s 闭环最短换相周期 (s)
 * @return {bool} 参数合法且当前未运行时返回 true
 */
bool Motor_Bldc_SetTuning(float integral_threshold_vs, float minimum_period_s)
{
    uint32_t primask;

    if (!(integral_threshold_vs >= 0.00005f && integral_threshold_vs <= 0.010f) ||
        !(minimum_period_s >= 0.0005f && minimum_period_s <= 0.100f)) {
        return false;
    }
    if (!s_initialized) {
        s_initialized = Motor_BldcDefaults_Init(&s_commutator, &s_sensorless);
        if (!s_initialized) {
            return false;
        }
    }
    if (Motor_BldcSixStep_IsRunning(&s_commutator)) {
        return false;
    }
    primask = Motor_Bldc_StatusLock();
    s_sensorless.config.bemf_integral_threshold_vs = integral_threshold_vs;
    s_sensorless.config.closed_loop_min_period_s = minimum_period_s;
    s_sensorless.config.integral_auto_detect_enabled = false;
    s_sensorless.config.duty_period_enabled = false;
    Motor_Bldc_StatusUnlock(primask);
    return true;
}

/**
 * @description: 停止六步输出，并继续占用主 PWM 出口。
 * @return {void} 无
 */
void Motor_Bldc_Stop(void)
{
    s_selected = true;
    if (s_initialized) {
        Motor_BldcSensorless_Stop(&s_sensorless);
    }
    Hal_Motor_PwmStop();
}

/**
 * @description: 停止六步输出并退出 BLDC，交还 FOC 主路径。
 * @return {void} 无
 */
void Motor_Bldc_Exit(void)
{
    if (s_initialized) {
        Motor_BldcSensorless_Stop(&s_sensorless);
    }
    Hal_Motor_PwmStop();
    s_selected = false;
}

/**
 * @description: 查询 BLDC 是否占用主 PWM 出口。
 * @return {bool} 已占用返回 true
 */
bool Motor_Bldc_IsSelected(void)
{
    return s_selected;
}

/**
 * @description: 复制默认 BLDC 实例的只读状态。
 * @param {motor_bldc_status_t *} status 输出快照，不可为空
 * @return {bool} 输出参数有效返回 true
 */
bool Motor_Bldc_GetStatus(motor_bldc_status_t *status)
{
    uint32_t primask;
    bool result;

    if (status == NULL) {
        return false;
    }
    primask = Motor_Bldc_StatusLock();
    if (s_initialized) {
        result = Motor_BldcStatus_Fill(status, s_selected, s_sample_upward, &s_commutator,
                                       &s_sensorless, Motor_Protect_GetFault(),
                                       Hal_Motor_BldcGetSampleCount());
    } else {
        result = Motor_BldcStatus_Fill(status, s_selected, s_sample_upward, NULL, NULL,
                                       FAULT_CODE_NONE, 0U);
    }
    Motor_Bldc_StatusUnlock(primask);
    return result;
}

/**
 * @description: 推进默认 BLDC 实例一个 ADC 中断周期。
 * @param {MOTOR_PARAM_S *} motor_param 当前电机采样与保护数据，不可为空
 * @return {void} 无
 */
void Motor_Bldc_RunIsr(MOTOR_PARAM_S *motor_param)
{
    if (s_selected) {
        Motor_BldcIsr_Step(&s_sensorless, motor_param, s_sample_upward);
    }
}
