/*
 * @Description: 传统 BLDC 霍尔六步换相独立控制器
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "common_assert.h"
#include <string.h>
#include "motor_bldc_six_step.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const motor_bldc_phase_mode_t s_forward_steps[6][3] = {
    {MOTOR_BLDC_PHASE_PWM, MOTOR_BLDC_PHASE_LOW, MOTOR_BLDC_PHASE_FLOAT},
    {MOTOR_BLDC_PHASE_PWM, MOTOR_BLDC_PHASE_FLOAT, MOTOR_BLDC_PHASE_LOW},
    {MOTOR_BLDC_PHASE_FLOAT, MOTOR_BLDC_PHASE_PWM, MOTOR_BLDC_PHASE_LOW},
    {MOTOR_BLDC_PHASE_LOW, MOTOR_BLDC_PHASE_PWM, MOTOR_BLDC_PHASE_FLOAT},
    {MOTOR_BLDC_PHASE_LOW, MOTOR_BLDC_PHASE_FLOAT, MOTOR_BLDC_PHASE_PWM},
    {MOTOR_BLDC_PHASE_FLOAT, MOTOR_BLDC_PHASE_LOW, MOTOR_BLDC_PHASE_PWM},
};
/* Private function prototypes -----------------------------------------------*/
static float Motor_BldcSixStep_ClampDuty(float duty);
static void Motor_BldcSixStep_Apply(motor_bldc_six_step_t *control);
static void Motor_BldcSixStep_SetFloating(motor_bldc_six_step_t *control);
static void Motor_BldcSixStep_Trip(motor_bldc_six_step_t *control, motor_bldc_fault_t fault);
static bool Motor_BldcSixStep_ConfigValid(const motor_bldc_six_step_config_t *config);
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 将占空比限制到有效范围。
 * @param {float} duty 输入占空比
 * @return {float} [0,1] 范围占空比；NaN 返回 0
 * @author: lkc
 */
static float Motor_BldcSixStep_ClampDuty(float duty)
{
    /* 该写法同时把 NaN 变成 0，避免 NaN 进入硬件比较值换算。 */
    if (!(duty >= 0.0f)) return 0.0f;
    if (duty > 1.0f) return 1.0f;
    return duty;
}

/**
 * @description: 通过配置回调发布当前桥臂输出。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSixStep_Apply(motor_bldc_six_step_t *control)
{
    if (control->config.apply_output != NULL) {
        control->config.apply_output(&control->output, control->config.output_user);
    }
}

/**
 * @description: 将实例输出设置为三相悬空并发布。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSixStep_SetFloating(motor_bldc_six_step_t *control)
{
    control->output.phase[0] = MOTOR_BLDC_PHASE_FLOAT;
    control->output.phase[1] = MOTOR_BLDC_PHASE_FLOAT;
    control->output.phase[2] = MOTOR_BLDC_PHASE_FLOAT;
    control->output.duty = 0.0f;
    control->output.hall = control->last_hall;
    control->output.step = MOTOR_BLDC_SIX_STEP_INVALID;
    Motor_BldcSixStep_Apply(control);
}

/**
 * @description: 锁存六步故障并停止输出。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空
 * @param {motor_bldc_fault_t} fault 要锁存的故障码
 * @return {void} 无
 * @author: lkc
 */
static void Motor_BldcSixStep_Trip(motor_bldc_six_step_t *control, motor_bldc_fault_t fault)
{
    control->fault = fault;
    control->running = false;
    Motor_BldcSixStep_SetFloating(control);
}

/**
 * @description: 校验霍尔换相配置。
 * @detail: 霍尔 0/7 必须无效，1..6 必须一一映射到六个不同换相步。
 * @param {const motor_bldc_six_step_config_t *} config 待校验配置
 * @return {bool} 配置有效返回 true
 * @author: lkc
 */
static bool Motor_BldcSixStep_ConfigValid(const motor_bldc_six_step_config_t *config)
{
    uint8_t used = 0U;
    uint8_t hall;

    if (config == NULL || config->hall_to_step[0] != MOTOR_BLDC_SIX_STEP_INVALID ||
        config->hall_to_step[7] != MOTOR_BLDC_SIX_STEP_INVALID) {
        return false;
    }
    for (hall = 1U; hall <= 6U; hall++) {
        uint8_t step = config->hall_to_step[hall];
        uint8_t bit;

        if (step >= 6U) return false;
        bit = (uint8_t)(1U << step);
        if ((used & bit) != 0U) return false;
        used = (uint8_t)(used | bit);
    }
    return used == 0x3FU;
}

/**
 * @description: 填充默认霍尔六步换相配置。
 * @param {motor_bldc_six_step_config_t *} config 输出配置；NULL 时忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSixStep_DefaultConfig(motor_bldc_six_step_config_t *config)
{
    static const uint8_t hall_to_step[8] = {
        MOTOR_BLDC_SIX_STEP_INVALID, 0U, 4U, 5U, 2U, 1U, 3U, MOTOR_BLDC_SIX_STEP_INVALID,
    };

    COMMON_ASSERT(config != NULL);
    memset(config, 0, sizeof(*config));
    memcpy(config->hall_to_step, hall_to_step, sizeof(hall_to_step));
    config->commutation_timeout_s = 0.5f;
}

/**
 * @description: 初始化独立六步换相实例。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空
 * @param {const motor_bldc_six_step_config_t *} config 配置，不可为空
 * @return {bool} 初始化成功返回 true
 * @author: lkc
 */
bool Motor_BldcSixStep_Init(motor_bldc_six_step_t *control,
                            const motor_bldc_six_step_config_t *config)
{
    if (control == NULL) return false;
    memset(control, 0, sizeof(*control));
    control->last_hall = MOTOR_BLDC_SIX_STEP_INVALID;
    control->output.step = MOTOR_BLDC_SIX_STEP_INVALID;
    if (!Motor_BldcSixStep_ConfigValid(config)) {
        control->fault = MOTOR_BLDC_FAULT_INVALID_CONFIG;
        return false;
    }
    control->config = *config;
    control->initialized = true;
    Motor_BldcSixStep_SetFloating(control);
    return true;
}

/**
 * @description: 启动六步换相实例。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空
 * @param {motor_bldc_direction_t} direction 转向
 * @param {float} duty PWM 占空比
 * @return {bool} 成功进入运行态返回 true
 * @author: lkc
 */
bool Motor_BldcSixStep_Start(motor_bldc_six_step_t *control,
                             motor_bldc_direction_t direction, float duty)
{
    if (control == NULL || !control->initialized || control->fault != MOTOR_BLDC_FAULT_NONE ||
        (direction != MOTOR_BLDC_DIRECTION_CW && direction != MOTOR_BLDC_DIRECTION_CCW)) {
        return false;
    }
    control->direction = direction;
    control->duty = Motor_BldcSixStep_ClampDuty(duty);
    control->edge_elapsed_s = 0.0f;
    control->last_hall = MOTOR_BLDC_SIX_STEP_INVALID;
    control->running = true;
    Motor_BldcSixStep_SetFloating(control);
    return true;
}

/**
 * @description: 停止六步换相并请求三相悬空。
 * @param {motor_bldc_six_step_t *} control 控制实例；NULL 时忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSixStep_Stop(motor_bldc_six_step_t *control)
{
    COMMON_ASSERT(control != NULL);
    control->running = false;
    control->edge_elapsed_s = 0.0f;
    Motor_BldcSixStep_SetFloating(control);
}

/**
 * @description: 在停机状态清除六步换相故障。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空
 * @return {bool} 成功清除返回 true
 * @author: lkc
 */
bool Motor_BldcSixStep_ClearFault(motor_bldc_six_step_t *control)
{
    if (control == NULL || control->running) return false;
    control->fault = MOTOR_BLDC_FAULT_NONE;
    control->edge_elapsed_s = 0.0f;
    control->last_hall = MOTOR_BLDC_SIX_STEP_INVALID;
    return control->initialized;
}

/**
 * @description: 设置六步换相 PWM 占空比。
 * @param {motor_bldc_six_step_t *} control 控制实例；NULL 时忽略
 * @param {float} duty PWM 占空比
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSixStep_SetDuty(motor_bldc_six_step_t *control, float duty)
{
    COMMON_ASSERT(control != NULL);
    control->duty = Motor_BldcSixStep_ClampDuty(duty);
    if (control->running && control->output.step < 6U) {
        control->output.duty = control->duty;
        Motor_BldcSixStep_Apply(control);
    }
}

/**
 * @description: 按霍尔状态推进一个六步控制周期。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空
 * @param {uint8_t} hall 三位霍尔状态，范围 0..7
 * @param {float} dt_s 本控制周期时间 (s)
 * @return {bool} 本周期输出有效返回 true
 * @author: lkc
 */
bool Motor_BldcSixStep_Step(motor_bldc_six_step_t *control, uint8_t hall, float dt_s)
{
    uint8_t step;

    if (control == NULL || !control->initialized || !control->running) return false;
    if (hall > 7U || (step = control->config.hall_to_step[hall]) >= 6U) {
        control->last_hall = hall;
        Motor_BldcSixStep_Trip(control, MOTOR_BLDC_FAULT_INVALID_HALL);
        return false;
    }

    if (hall != control->last_hall) {
        control->last_hall = hall;
        control->edge_elapsed_s = 0.0f;
    } else if (dt_s > 0.0f) {
        control->edge_elapsed_s += dt_s;
        if (control->config.commutation_timeout_s > 0.0f &&
            control->edge_elapsed_s >= control->config.commutation_timeout_s) {
            Motor_BldcSixStep_Trip(control, MOTOR_BLDC_FAULT_COMMUTATION_TIMEOUT);
            return false;
        }
    }

    control->output.hall = hall;
    return Motor_BldcSixStep_CommutateStep(control, step);
}

/**
 * @description: 直接发布指定电气换相步。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空
 * @param {uint8_t} step 换相步，范围 0..5
 * @return {bool} 发布成功返回 true
 * @author: lkc
 */
bool Motor_BldcSixStep_CommutateStep(motor_bldc_six_step_t *control, uint8_t step)
{
    uint8_t phase;

    if (control == NULL || !control->initialized || !control->running || step >= 6U) return false;

    control->output.step = step;
    control->output.duty = control->duty;
    for (phase = 0U; phase < 3U; phase++) {
        motor_bldc_phase_mode_t mode = s_forward_steps[step][phase];
        if (control->direction == MOTOR_BLDC_DIRECTION_CCW) {
            if (mode == MOTOR_BLDC_PHASE_PWM) {
                mode = MOTOR_BLDC_PHASE_LOW;
            } else if (mode == MOTOR_BLDC_PHASE_LOW) {
                mode = MOTOR_BLDC_PHASE_PWM;
            }
        }
        control->output.phase[phase] = mode;
    }
    Motor_BldcSixStep_Apply(control);
    return true;
}

/**
 * @description: 获取最近一次桥臂输出。
 * @param {const motor_bldc_six_step_t *} control 控制实例
 * @return {const motor_bldc_output_t *} 输出只读指针；空实例返回 NULL
 * @author: lkc
 */
const motor_bldc_output_t *Motor_BldcSixStep_GetOutput(const motor_bldc_six_step_t *control)
{
    return control == NULL ? NULL : &control->output;
}

/**
 * @description: 获取六步换相故障码。
 * @param {const motor_bldc_six_step_t *} control 控制实例
 * @return {motor_bldc_fault_t} 当前故障码
 * @author: lkc
 */
motor_bldc_fault_t Motor_BldcSixStep_GetFault(const motor_bldc_six_step_t *control)
{
    return control == NULL ? MOTOR_BLDC_FAULT_INVALID_CONFIG : control->fault;
}

/**
 * @description: 查询六步换相是否正在运行。
 * @param {const motor_bldc_six_step_t *} control 控制实例
 * @return {bool} 正在运行返回 true
 * @author: lkc
 */
bool Motor_BldcSixStep_IsRunning(const motor_bldc_six_step_t *control)
{
    return control != NULL && control->running;
}
#ifdef __cplusplus
}
#endif
