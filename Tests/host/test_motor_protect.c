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

    printf("test_motor_protect: pass\n");
    return 0;
}
