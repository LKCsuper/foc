#ifndef MOTOR_POSITION_CORE_H
#define MOTOR_POSITION_CORE_H
#include <stdint.h>
#include "motor_position_control.h"
#ifdef __cplusplus
extern "C" {
#endif

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

/** 初始化，非法配置不改状态。tolerance保留作兼容诊断值。 */
bool Motor_PositionCore_Init(motor_position_runtime_t *runtime, const motor_position_config_t *config,
                             uint32_t origin_count, float tolerance_deg);
/** 置多圈零点并复位控制积分，保留使能和增益。 */
void Motor_PositionCore_Zero(motor_position_runtime_t *runtime, uint32_t count);
/** 使能需ready；禁用清控制积分和保持模式。 */
bool Motor_PositionCore_Enable(motor_position_runtime_t *runtime, bool enable, bool ready);
bool Motor_PositionCore_SetTarget(motor_position_runtime_t *runtime, float target_deg);
bool Motor_PositionCore_Move(motor_position_runtime_t *runtime, float delta_deg);
bool Motor_PositionCore_SetKp(motor_position_runtime_t *runtime, float kp);
bool Motor_PositionCore_SetMaxRpm(motor_position_runtime_t *runtime, float rpm, float allowed_rpm);
/** 清积分与模式，不改目标或使能。 */
void Motor_PositionCore_ClearHold(motor_position_runtime_t *runtime);
/** 一拍控制与模式切换；输出关断/写PWM由调用者完成。初始化后调用，输入须有限。
 * 控制配置非法时返回false并输出释放保持，命令所有权由enabled决定。
 */
bool Motor_PositionCore_Step(motor_position_runtime_t *runtime, const motor_position_config_t *config,
                             const motor_position_input_t *input, motor_position_output_t *output);
/** 只刷新返回快照，不推进积分；config及runtime须已初始化。 */
void Motor_PositionCore_Snapshot(const motor_position_runtime_t *runtime, const motor_position_config_t *config,
                                 uint32_t count, bool feedback_valid, motor_position_state_t *snapshot);
#ifdef __cplusplus
}
#endif
#endif
