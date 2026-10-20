/*
 * @Description: 编码器多圈位置环（位置 P 串速度 PI，输出电压）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "motor_position.h"
#include "motor_position_control.h"
#include "motor_encoder.h"
#include "motor_mt6701.h"
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_POSITION_TOLERANCE_DEG           (0.30f)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_position_runtime_t s_runtime;
/* Private function prototypes -----------------------------------------------*/
static void Motor_Position_ClearHold(void);
static void Motor_Position_FillControlConfig(motor_position_control_config_t *config);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 从应用位置状态生成纯控制模块配置。
 * @param {motor_position_control_config_t *} config 输出配置，不可为空
 * @return {void} 无
 */
static void Motor_Position_FillControlConfig(motor_position_control_config_t *config)
{
    config->position_kp_rpm_per_deg = s_runtime.state.kp_rpm_per_deg;
    config->maximum_rpm = s_runtime.state.max_rpm;
    config->velocity_kp_v_per_rpm = MOTOR_POSITION_VEL_KP_DEFAULT;
    config->velocity_ki_v_per_rpm_s = MOTOR_POSITION_VEL_KI_DEFAULT;
    config->voltage_limit_v = MOTOR_POSITION_VOLTAGE_LIMIT_DEFAULT;
    config->acceleration_limit_rpm_s = MOTOR_POSITION_ACCEL_RPM_S_DEFAULT;
    config->feedforward_v_per_rpm = MOTOR_POSITION_FF_V_PER_RPM_DEFAULT;
    config->maximum_dt_s = 0.05f;
}



static void Motor_Position_FillConfig(motor_position_config_t *config)
{
    config->counts_per_rev = MOTOR_MT6701_COUNTS_PER_REV;
    config->hybrid_speed_enabled = MOTOR_POSITION_HYBRID_SPEED_ENABLE != 0U;
    config->hold_enter_deg = MOTOR_POSITION_HOLD_ENTER_DEG;
    config->hold_exit_deg = MOTOR_POSITION_HOLD_EXIT_DEG;
    Motor_Position_FillControlConfig(&config->control);
}

/**
 * @description: 退出位置保持，清掉速度积分和电压输出。
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Position_ClearHold(void)
{
    Motor_PositionCore_ClearHold(&s_runtime);
    motor.position_hold = false;
    motor.position_voltage_hold = false;
    motor.position_hold_v = 0.0f;
}

/**
 * @description: 用当前累计计数作为零点，并装入默认比例与转速上限。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_Init(void)
{
    motor_position_config_t config;
    s_runtime.state.kp_rpm_per_deg = MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG;
    s_runtime.state.max_rpm = MOTOR_POSITION_MAX_RPM_DEFAULT;
    Motor_Position_FillConfig(&config);
    (void)Motor_PositionCore_Init(&s_runtime, &config, Motor_Encoder_GetAccumulatedCount(), MOTOR_POSITION_TOLERANCE_DEG);
}

/**
 * @description: 把当前位置记为零，并清掉目标与速度指令。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_Zero(void)
{
    Motor_PositionCore_Zero(&s_runtime, Motor_Encoder_GetAccumulatedCount());
}

/**
 * @description: 开关位置环。
 * @detail: 打开前要求编码器零点有效且无信号故障。关闭时速度目标清零。
 * @param {bool} enable  true=使能
 * @return {bool} true=已设置；false=编码器未就绪
 * @author: lkc
 */
bool Motor_Position_Enable(bool enable)
{
    bool ready = !enable || (Motor_Encoder_IsZeroValid() && !Motor_Encoder_IsSignalFault());
    if (!Motor_PositionCore_Enable(&s_runtime, enable, ready)) return false;
    if (!enable) Motor_Position_ClearHold();
    return true;
}

/**
 * @description: 设置绝对目标位置。
 * @param {float} target_deg  相对零点的目标 (deg)，须为有限值
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Position_SetTargetDeg(float target_deg)
{
    return Motor_PositionCore_SetTarget(&s_runtime, target_deg);
}

/**
 * @description: 在当前目标上叠加一段位移。
 * @param {float} delta_deg  增量 (deg)，须为有限值
 * @return {bool} true=相加后目标仍为有限值
 * @author: lkc
 */
bool Motor_Position_MoveDeg(float delta_deg)
{
    return Motor_PositionCore_Move(&s_runtime, delta_deg);
}

/**
 * @description: 设置位置比例。
 * @param {float} kp_rpm_per_deg  比例 (RPM/deg)，范围 (0, 100]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Position_SetKp(float kp_rpm_per_deg)
{
    return Motor_PositionCore_SetKp(&s_runtime, kp_rpm_per_deg);
}

/**
 * @description: 设置位置环给出的速度上限。
 * @param {float} max_rpm  限幅 (RPM)，范围 [1, 当前电机允许上限]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Position_SetMaxRpm(float max_rpm)
{
    return Motor_PositionCore_SetMaxRpm(&s_runtime, max_rpm, MOTOR_POSITION_MAX_RPM_ALLOWED);
}

/**
 * @description: 推进一拍位置环；大误差可用速度电流环移动，目标附近直接电压保持。
 * @detail: 未使能、电机未启动或编码器故障时退出位置保持并清积分。
 *          位置误差始终换成转速命令，再由速度 PI 出电压。转速写进 ol_target_rpm 的值保持 0，
 *          避免电压速度模式的起动前馈在终点被踢出来。到达目标后积分保留，用来抵消齿槽。
 * @param {float} dt  调用周期 (s)。非正或大于 0.05 s 时本拍不积分
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_Run(float dt)
{
    motor_position_config_t config;
    motor_position_input_t input;
    motor_position_output_t output;
    Motor_Position_FillConfig(&config);
    input.accumulated_count = Motor_Encoder_GetAccumulatedCount();
    input.motor_running = motor.ol_enable;
    input.feedback_valid = !Motor_Encoder_IsSignalFault();
    input.speed_valid = Motor_Encoder_IsSpeedValid();
    input.measured_rpm = input.speed_valid ? Motor_Encoder_GetMechRpm() : 0.0f;
    input.dt = dt;
    if (!Motor_PositionCore_Step(&s_runtime, &config, &input, &output)) {
        Motor_Position_ClearHold();
        return;
    }
    motor.position_hold = output.hold_active;
    motor.position_voltage_hold = output.voltage_hold;
    motor.position_hold_v = output.voltage_v;
    if (output.owns_speed_command) motor.ol_target_rpm = output.speed_rpm;
}

/**
 * @description: 复制位置环状态。
 * @param {motor_position_state_t} *state  输出，为空则忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_Position_GetState(motor_position_state_t *state)
{
    motor_position_config_t config;
    Motor_Position_FillConfig(&config);
    Motor_PositionCore_Snapshot(&s_runtime, &config, Motor_Encoder_GetAccumulatedCount(),
                                !Motor_Encoder_IsSignalFault(), state);
}

/**
 * @description: 查询位置环是否使能。
 * @return {bool} true=已使能
 * @author: lkc
 */
bool Motor_Position_IsEnabled(void)
{
    return s_runtime.state.enabled;
}
#ifdef __cplusplus
}
#endif
