#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_sensorless.h"
#include "motor_param.h"

int main(void)
{
    motor_sensorless_runtime_t a;
    motor_sensorless_runtime_t b;
    motor_sensorless_diagnostics_t diag_a;
    motor_sensorless_diagnostics_t diag_b;
    motor_sensorless_effects_t effects;
    MOTOR_PARAM_S motor;
    float angle_a;
    float angle_b;

    memset(&motor, 0, sizeof(motor));
    motor.ol_mode = MOTOR_OL_MODE_IF;
    motor.ol_enable = true;
    motor.sensorless_enable = false;

    Motor_Sensorless_InitState(&a);
    Motor_Sensorless_InitState(&b);
    assert(Motor_Sensorless_GetStateState(&a) == MOTOR_SENSORLESS_OPEN_LOOP);
    assert(Motor_Sensorless_GetStateState(&b) == MOTOR_SENSORLESS_OPEN_LOOP);

    /* 双实例电压记录互不影响。 */
    Motor_Sensorless_SetAppliedVoltageState(&a, 3.0f, 4.0f); /* |V|=5 */
    Motor_Sensorless_SetAppliedVoltageState(&b, 0.0f, 8.0f); /* |V|=8 */
    Motor_Sensorless_GetDiagnosticsState(&a, &diag_a);
    Motor_Sensorless_GetDiagnosticsState(&b, &diag_b);
    assert(fabsf(diag_a.applied_voltage - 5.0f) < 1e-4f);
    assert(fabsf(diag_b.applied_voltage - 8.0f) < 1e-4f);

    /* 关闭无感时 UpdateState 复位并声明反馈无效。 */
    Motor_Sensorless_UpdateState(&a, &motor, 0.001f, &effects);
    assert(!effects.feedback_valid);
    assert(Motor_Sensorless_GetStateState(&a) == MOTOR_SENSORLESS_OPEN_LOOP);

    /* 手动置 CLOSED：控制角与接管完成标志按实例隔离。 */
    a.state = MOTOR_SENSORLESS_CLOSED;
    a.control_angle = 1.25f;
    a.blend = 1.0f;
    a.pll_angle = 1.25f;
    b.state = MOTOR_SENSORLESS_OPEN_LOOP;
    b.control_angle = 0.1f;
    b.blend = 0.0f;
    angle_a = Motor_Sensorless_GetControlAngleState(&a, 0.0f);
    angle_b = Motor_Sensorless_GetControlAngleState(&b, 9.0f);
    assert(fabsf(angle_a - 1.25f) < 1e-6f);
    assert(fabsf(angle_b - 9.0f) < 1e-6f);
    assert(Motor_Sensorless_IsClosedLoopState(&a));
    assert(!Motor_Sensorless_IsClosedLoopState(&b));
    assert(Motor_Sensorless_IsTakeoverCompleteState(&a));
    assert(!Motor_Sensorless_IsTakeoverCompleteState(&b));

    /* 高速失锁路径：CLOSED + 磁链失败 + 目标超 I/F 安全上限 → 请求失速故障。 */
    memset(&effects, 0, sizeof(effects));
    Motor_Sensorless_InitState(&a);
    a.state = MOTOR_SENSORLESS_CLOSED;
    a.initialized = true;
    a.x_alpha = 0.01f;
    a.x_beta = 0.0f;
    a.filtered_rpm = 50.0f;
    a.pll_angle = 0.0f;
    a.observer_angle = 0.0f;
    motor.sensorless_enable = true;
    motor.ol_enable = true;
    motor.ol_mode = MOTOR_OL_MODE_IF;
    motor.ol_current_rpm = 2000.0f;
    motor.ol_target_rpm = 2000.0f;
    motor.ol_max_rpm = 5000.0f;
    motor.ol_pole_pairs = 7.0f;
    motor.sensorless_motor_r = 0.4f;
    motor.sensorless_motor_l = 0.0008f;
    motor.sensorless_flux_linkage = 0.02f;
    motor.sensorless_observer_gain = 0.0f;
    motor.sensorless_transition_rpm = 500.0f;
    motor.sensorless_fallback_rpm = 200.0f;
    motor.sensorless_loss_time = 0.001f;
    motor.sensorless_openloop_max_rpm = 800.0f;
    motor.sensorless_speed_filter = 1.0f;
    motor.sensorless_pll_kp = 200.0f;
    motor.sensorless_pll_ki = 20000.0f;
    motor.ia = 0.0f;
    motor.ib = 0.0f;
    motor.ic = 0.0f;
    motor.ol_theta = 0.0f;
    /* 多拍让 loss_timer 超过 loss_time；小磁链使 flux_ok 失败。首拍即应请求失速。 */
    Motor_Sensorless_UpdateState(&a, &motor, 0.001f, &effects);
    assert(effects.request_speed_loss_fault);
    assert(!effects.feedback_valid);
    assert(Motor_Sensorless_GetStateState(&a) == MOTOR_SENSORLESS_OPEN_LOOP);

    printf("test_motor_sensorless: pass\n");
    return 0;
}
