#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_protect.h"
#include "motor_param.h"
#include "motor_types.h"

int main(void)
{
    motor_protect_state_t a;
    motor_protect_state_t b;
    MOTOR_PARAM_S motor_a;
    MOTOR_PARAM_S motor_b;
    motor_protect_aux_t aux;
    motor_protect_snapshot_t snap;
    motor_protect_config_t cfg;
    float dt = 0.001f;
    unsigned i;

    memset(&motor_a, 0, sizeof(motor_a));
    memset(&motor_b, 0, sizeof(motor_b));
    motor_a.ol_enable = true;
    motor_a.ol_target_rpm = 500.0f;
    motor_a.v_bus = 24.0f;
    motor_a.ia = 0.1f;
    motor_a.ib = 0.1f;
    motor_a.ic = 0.1f;
    motor_a.ol_iq_set = 0.3f;
    motor_b = motor_a;

    Motor_Protect_InitState(&a, NULL);
    Motor_Protect_InitState(&b, NULL);

    /* A：瞬时过流去抖后锁存；B 保持正常。 */
    motor_a.ia = 20.0f;
    memset(&aux, 0, sizeof(aux));
    aux.iq_command = 1.25f;
    aux.observer_rpm = 321.0f;
    aux.flux_ok = true;
    for (i = 0; i < 5U; ++i) {
        (void)Motor_Protect_RunState(&a, &motor_a, dt, &aux);
        (void)Motor_Protect_RunState(&b, &motor_b, dt, NULL);
    }
    assert(Motor_Protect_IsFaultLatchedState(&a));
    assert(Motor_Protect_GetFaultState(&a) == FAULT_CODE_ABS_OVER_CURRENT);
    assert(!Motor_Protect_IsFaultLatchedState(&b));
    assert(Motor_Protect_GetFaultState(&b) == FAULT_CODE_NONE);

    Motor_Protect_GetSnapshotState(&a, &snap);
    assert(snap.iq_command == 1.25f);
    assert(snap.observer_rpm == 321.0f);
    assert(snap.flux_ok);
    assert(snap.current_peak >= 20.0f);

    /* 清 A 不影响 B；随后 B 欠压锁存不影响 A。 */
    Motor_Protect_ClearFaultState(&a);
    assert(!Motor_Protect_IsFaultLatchedState(&a));
    motor_b.v_bus = 1.0f;
    for (i = 0; i < 150U; ++i) {
        (void)Motor_Protect_RunState(&b, &motor_b, dt, NULL);
    }
    assert(Motor_Protect_IsFaultLatchedState(&b));
    assert(Motor_Protect_GetFaultState(&b) == FAULT_CODE_UNDER_VOLTAGE);
    assert(!Motor_Protect_IsFaultLatchedState(&a));

    /* 自定义阈值：提高过流门限后同电流不触发。 */
    cfg = a.config;
    cfg.over_current = 50.0f;
    cfg.over_current_time = 0.0001f;
    Motor_Protect_InitState(&a, &cfg);
    motor_a.ia = 20.0f;
    for (i = 0; i < 5U; ++i) {
        (void)Motor_Protect_RunState(&a, &motor_a, dt, NULL);
    }
    assert(!Motor_Protect_IsFaultLatchedState(&a));

    assert(strcmp(Motor_Protect_GetFaultName(FAULT_CODE_MOTOR_STALL), "MOTOR_STALL") == 0);

    /* 150 运转中直接 olspeed 3000：斜坡未到位，不得误报失速。 */
    Motor_Protect_InitState(&a, NULL);
    memset(&motor_a, 0, sizeof(motor_a));
    motor_a.ol_enable = true;
    motor_a.v_bus = 24.0f;
    motor_a.ia = 0.4f;
    motor_a.ib = 0.4f;
    motor_a.ic = 0.4f;
    motor_a.ol_target_rpm = 150.0f;
    motor_a.ol_current_rpm = 150.0f;
    Motor_Protect_SetSpeedFeedbackState(&a, 150.0f, true);
    for (i = 0; i < 50U; ++i) {
        assert(!Motor_Protect_CheckSpeedLossState(&a, &motor_a, dt));
    }
    motor_a.ol_target_rpm = 3000.0f;
    motor_a.ol_current_rpm = 165.0f;
    Motor_Protect_SetSpeedFeedbackState(&a, 162.5f, true);
    for (i = 0; i < 500U; ++i) {
        assert(!Motor_Protect_CheckSpeedLossState(&a, &motor_a, dt));
    }
    /* 斜坡已到 3000、实测仍在加速：也不得当成丢速。 */
    motor_a.ol_current_rpm = 3000.0f;
    Motor_Protect_SetSpeedFeedbackState(&a, 400.0f, true);
    for (i = 0; i < 500U; ++i) {
        assert(!Motor_Protect_CheckSpeedLossState(&a, &motor_a, dt));
    }

    /* 斜坡已跟上 800、曾达转速后再掉到 100：应在 0.3 s 去抖后判失速。 */
    motor_a.ol_target_rpm = 800.0f;
    motor_a.ol_current_rpm = 800.0f;
    Motor_Protect_SetSpeedFeedbackState(&a, 800.0f, true);
    assert(!Motor_Protect_CheckSpeedLossState(&a, &motor_a, dt));
    Motor_Protect_SetSpeedFeedbackState(&a, 100.0f, true);
    for (i = 0; i < 250U; ++i) {
        assert(!Motor_Protect_CheckSpeedLossState(&a, &motor_a, dt));
    }
    {
        bool lost = false;
        for (i = 0; i < 150U; ++i) {
            if (Motor_Protect_CheckSpeedLossState(&a, &motor_a, dt)) {
                lost = true;
                break;
            }
        }
        assert(lost);
    }

    printf("test_motor_protect: pass\n");
    return 0;
}
