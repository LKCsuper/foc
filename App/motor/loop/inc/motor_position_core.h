/* 位置命令、多圈零点与混合控制状态；不访问硬件或全局电机。 */
#ifndef MOTOR_POSITION_CORE_H
#define MOTOR_POSITION_CORE_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "motor_position_control.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Private typedef -----------------------------------------------------------*/
/** 位置命令与诊断状态；角度deg，速度机械RPM。 */
typedef struct {
    bool enabled;
    int32_t origin_count;
    float target_deg, position_deg, error_deg, target_rpm;
    float kp_rpm_per_deg, max_rpm, tolerance_deg;
    bool voltage_hold_mode;
} motor_position_state_t;

typedef struct {
    motor_position_state_t state;
    motor_position_control_t control;
} motor_position_runtime_t;

/** 计数每圈须非零；退出阈值须不小于进入阈值。配置由调用者持有。 */
typedef struct {
    uint32_t counts_per_rev;
    bool hybrid_speed_enabled;
    float hold_enter_deg, hold_exit_deg;
    motor_position_control_config_t control;
} motor_position_config_t;

typedef struct {
    uint32_t accumulated_count; /* uint32回绕计数，距零点须在int32范围内 */
    bool motor_running, feedback_valid, speed_valid;
    float measured_rpm, dt;
} motor_position_input_t;

typedef struct {
    bool hold_active, voltage_hold;
    bool owns_speed_command; /* 未使能时false，上层须保留其他速度命令 */
    float voltage_v, speed_rpm;
} motor_position_output_t;

/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 初始化，非法配置不改状态。tolerance保留作兼容诊断值。
 * @param {motor_position_runtime_t *} runtime  运行状态，不可为空
 * @param {const motor_position_config_t *} config  配置，不可为空
 * @param {uint32_t} origin_count  多圈零点累计计数
 * @param {float} tolerance_deg    兼容保留的诊断容差 (deg)
 * @return {bool} 初始化成功返回 true
 */
bool Motor_PositionCore_Init(motor_position_runtime_t *runtime, const motor_position_config_t *config,
                             uint32_t origin_count, float tolerance_deg);
/**
 * @description: 置多圈零点并复位控制积分，保留使能和增益。
 * @param {motor_position_runtime_t *} runtime  运行状态，不可为空
 * @param {uint32_t} count  新的零点累计计数
 * @return {void} 无
 */
void Motor_PositionCore_Zero(motor_position_runtime_t *runtime, uint32_t count);
/**
 * @description: 使能需ready；禁用清控制积分和保持模式。
 * @param {motor_position_runtime_t *} runtime  运行状态，不可为空
 * @param {bool} enable  true=使能
 * @param {bool} ready   true=可以进入使能
 * @return {bool} 设置成功返回 true
 */
bool Motor_PositionCore_Enable(motor_position_runtime_t *runtime, bool enable, bool ready);
/**
 * @description: 设置绝对目标位置。
 * @param {motor_position_runtime_t *} runtime  运行状态
 * @param {float} target_deg  目标 (deg)，须为有限值
 * @return {bool} 写入成功返回 true
 */
bool Motor_PositionCore_SetTarget(motor_position_runtime_t *runtime, float target_deg);
/**
 * @description: 在当前目标上叠加一段位移。
 * @param {motor_position_runtime_t *} runtime  运行状态
 * @param {float} delta_deg  增量 (deg)，须为有限值
 * @return {bool} 相加后目标仍为有限值返回 true
 */
bool Motor_PositionCore_Move(motor_position_runtime_t *runtime, float delta_deg);
/**
 * @description: 设置位置比例。
 * @param {motor_position_runtime_t *} runtime  运行状态
 * @param {float} kp  比例 (RPM/deg)，范围 (0, 100]
 * @return {bool} 写入成功返回 true
 */
bool Motor_PositionCore_SetKp(motor_position_runtime_t *runtime, float kp);
/**
 * @description: 设置位置环给出的速度上限。
 * @param {motor_position_runtime_t *} runtime  运行状态
 * @param {float} rpm         速度上限 (RPM)
 * @param {float} allowed_rpm 允许的电机上限 (RPM)
 * @return {bool} 写入成功返回 true
 */
bool Motor_PositionCore_SetMaxRpm(motor_position_runtime_t *runtime, float rpm, float allowed_rpm);
/**
 * @description: 清积分与模式，不改目标或使能。
 * @param {motor_position_runtime_t *} runtime  运行状态，不可为空
 * @return {void} 无
 */
void Motor_PositionCore_ClearHold(motor_position_runtime_t *runtime);
/**
 * @description: 一拍控制与模式切换；输出关断/写PWM由调用者完成。初始化后调用，输入须有限。
 * 控制配置非法时返回false并输出释放保持，命令所有权由enabled决定。
 * @param {motor_position_runtime_t *} runtime  运行状态，不可为空
 * @param {const motor_position_config_t *} config  配置，不可为空
 * @param {const motor_position_input_t *} input  本拍输入，不可为空
 * @param {motor_position_output_t *} output  本拍输出，不可为空
 * @return {bool} 本拍计算完成返回 true
 */
bool Motor_PositionCore_Step(motor_position_runtime_t *runtime, const motor_position_config_t *config,
                             const motor_position_input_t *input, motor_position_output_t *output);
/**
 * @description: 只刷新返回快照，不推进积分；config及runtime须已初始化。
 * @param {const motor_position_runtime_t *} runtime  运行状态，不可为空
 * @param {const motor_position_config_t *} config  配置，不可为空
 * @param {uint32_t} count  当前累计计数
 * @param {bool} feedback_valid  true=位置反馈可用
 * @param {motor_position_state_t *} snapshot  输出快照，不可为空
 * @return {void} 无
 */
void Motor_PositionCore_Snapshot(const motor_position_runtime_t *runtime, const motor_position_config_t *config,
                                 uint32_t count, bool feedback_valid, motor_position_state_t *snapshot);
#ifdef __cplusplus
}
#endif
#endif
