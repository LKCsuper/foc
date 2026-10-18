#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_protect.h"
#include "motor_param.h"
#include "motor_types.h"
extern unsigned protect_diagnostic_reads;

static void test_overcurrent_sampling_window(void)
{
    motor_protect_state_t state;
    motor_protect_input_t input;
    motor_protect_snapshot_t snapshot;
    const float interval = 1.0f / 15000.0f;
    memset(&input, 0, sizeof(input));
    input.v_bus = 24.0f;
    Motor_Protect_InitState(&state, NULL);
    for (unsigned step = 0; step < 1000; ++step) {
        input.ia = step % 2 ? 1.0f : 11.187f;
        assert(!Motor_Protect_RunInputState(&state, &input, interval, NULL));
    }
    input.ia = 1.0f;
    assert(!Motor_Protect_RunInputState(&state, &input, interval, NULL));
    assert(state.over_current_timer == 0.0f);
    input.ia = 20.0f;
    for (unsigned step = 0; step < 14; ++step)
        assert(!Motor_Protect_RunInputState(&state, &input, interval, NULL));
    input.ia = 0.0f;
    input.ib = -11.187f;
    input.Id = -9.827f;
    input.Iq = 6.908f;
    for (unsigned step = 0; step < 2; ++step)
        (void)Motor_Protect_RunInputState(&state, &input, interval, NULL);
    assert(Motor_Protect_GetFaultState(&state) == FAULT_CODE_ABS_OVER_CURRENT);
    Motor_Protect_GetSnapshotState(&state, &snapshot);
    assert(fabsf(snapshot.current_peak - 11.187f) < 1.0e-5f);
    assert(snapshot.id == input.Id && snapshot.iq == input.Iq);
    input.ic = 50.0f;
    (void)Motor_Protect_RunInputState(&state, &input, interval, NULL);
    Motor_Protect_GetSnapshotState(&state, &snapshot);
    assert(fabsf(snapshot.current_peak - 11.187f) < 1.0e-5f);
}

int main(void)
{
    motor_protect_state_t a;
    motor_protect_state_t b;
    MOTOR_PARAM_S motor_a;
    MOTOR_PARAM_S motor_b;
    motor_protect_aux_t aux;
    motor_protect_snapshot_t snap;
    motor_protect_sample_t sample;
    motor_protect_config_t cfg;
    float dt = 0.001f;
    unsigned i;
    test_overcurrent_sampling_window();
    {
        motor_protect_state_t combined, separate;
        motor_protect_input_t input;
        unsigned step;
        memset(&input, 0, sizeof(input));
        Motor_Protect_InitState(&combined, NULL);
        Motor_Protect_InitState(&separate, NULL);
        for (step = 0U; step < 10000U; step++) {
            mc_fault_code expected = FAULT_CODE_NONE;
            float interval = step % 3U == 0U ? 0.0005f : 0.001f;
            input.ia = (float)((int)(step % 41U) - 20);
            input.ib = (float)((int)(step % 17U) - 8);
            input.ic = -input.ia - input.ib;
            input.v_bus = step % 89U < 40U ? 7.0f : 24.0f;
            input.ol_enable = step % 71U != 0U;
            input.ol_target_rpm = step % 2U ? 600.0f : -600.0f;
            input.ol_current_rpm = step % 31U ? input.ol_target_rpm : 0.0f;
            Motor_Protect_SetSpeedFeedbackState(&combined, (float)(step % 700U), step % 23U != 0U);
            Motor_Protect_SetSpeedFeedbackState(&separate, (float)(step % 700U), step % 23U != 0U);
            if (Motor_Protect_CheckOverCurrentInputState(&separate, &input, interval))
                expected = FAULT_CODE_ABS_OVER_CURRENT;
            else if (Motor_Protect_CheckUnderVoltageInputState(&separate, &input, interval))
                expected = FAULT_CODE_UNDER_VOLTAGE;
            else if (Motor_Protect_CheckStallInputState(&separate, &input, interval))
                expected = FAULT_CODE_MOTOR_STALL;
            else if (Motor_Protect_CheckSpeedLossInputState(&separate, &input, interval))
                expected = FAULT_CODE_MOTOR_SPEED_LOSS;
            assert(Motor_Protect_EvaluateInputState(&combined, &input, interval) == expected);
            assert(combined.over_current_timer == separate.over_current_timer);
            assert(combined.under_voltage_timer == separate.under_voltage_timer);
            assert(combined.stall_timer == separate.stall_timer);
            assert(combined.speed_loss_timer == separate.speed_loss_timer);
            assert(combined.speed_was_reached == separate.speed_was_reached);
        }
    }

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

    memset(&motor_a, 0, sizeof(motor_a));
    motor_a.ol_enable = true;
    motor_a.v_bus = 24.0f;
    motor_a.ol_iq_set = 0.5f;
    Motor_Protect_Init(NULL);
    protect_diagnostic_reads = 0U;
    for (i = 0U; i < 100U; i++) assert(!Motor_Protect_Run(&motor_a, 0.001f));
    assert(protect_diagnostic_reads == 0U);
    motor_a.ia = 20.0f;
    motor_a.currentAdc[0] = 2184.0f;
    motor_a.currentAdc[1] = 2044.0f;
    motor_a.currentAdc[2] = 2043.0f;
    motor_a.currentOffset[0] = 2045.6f;
    motor_a.currentOffset[1] = 2044.8f;
    motor_a.currentOffset[2] = 2045.2f;
    Motor_Protect_CaptureFaultSample(&motor_a, .08f, 10, 20, 5600, 16);
    Motor_Protect_GetFaultSample(&sample);
    assert(!sample.valid && !Motor_Protect_FaultSampleIsPending());
    assert(!Motor_Protect_Run(&motor_a, 0.0005f));
    assert(protect_diagnostic_reads == 0U);
    assert(Motor_Protect_Run(&motor_a, 0.0005f));
    assert(protect_diagnostic_reads == 1U);
    assert(Motor_Protect_FaultSampleIsPending());
    Motor_Protect_CaptureFaultSample(&motor_a, .08058608f, 1234, 3500, 5600, 161);
    Motor_Protect_GetFaultSample(&sample);
    assert(sample.valid && !Motor_Protect_FaultSampleIsPending());
    assert(sample.adc[0] == 2184.0f && sample.adc[1] == 2044.0f && sample.adc[2] == 2043.0f);
    assert(sample.configured_offset[0] == motor_a.currentOffset[0]);
    assert(sample.current_abc[0] == 20.0f && sample.current_abc[1] == motor_a.ib);
    assert(sample.scale_a_per_count == .08058608f && sample.entry_us == 1234);
    assert(sample.callback_count == 3500 && sample.callback_period == 5600 && sample.callback_cr1 == 161);
    assert(sample.trigger_dt_s == .0005f && sample.over_current_elapsed_s == .001f);
    assert(sample.over_current_threshold_a == 10.0f && sample.over_current_delay_s == .001f);
    motor_a.currentAdc[0] = 3000;
    Motor_Protect_CaptureFaultSample(&motor_a, 1, 9999, 0, 1, 0);
    Motor_Protect_GetFaultSample(&sample);
    assert(sample.adc[0] == 2184.0f && sample.entry_us == 1234);
    Motor_Protect_GetSnapshot(&snap);
    assert(Motor_Protect_GetFault() == FAULT_CODE_ABS_OVER_CURRENT);
    assert(snap.applied_voltage == 7.0f && snap.blend == 0.75f && snap.flux_ok);
    assert(snap.iq_command == 0.5f);
    motor_a.v_bus = 0.0f;
    motor_a.ol_iq_set = 9.0f;
    assert(Motor_Protect_Run(&motor_a, 1.0f));
    Motor_Protect_GetSnapshot(&snap);
    assert(protect_diagnostic_reads == 1U && snap.iq_command == 0.5f);
    assert(Motor_Protect_GetFault() == FAULT_CODE_ABS_OVER_CURRENT);
    Motor_Protect_ClearFault();
    Motor_Protect_GetFaultSample(&sample);
    assert(!sample.valid && !Motor_Protect_FaultSampleIsPending());
    motor_a.ia = 0.0f;
    motor_a.v_bus = 24.0f;
    assert(!Motor_Protect_Run(&motor_a, 0.001f));
    assert(protect_diagnostic_reads == 1U);
    assert(!Motor_Protect_Run(NULL, 0.001f));
    Motor_Protect_Init(NULL);
    Motor_Protect_TriggerFault(FAULT_CODE_CONTROL_TIMING, &motor_a);
    assert(!Motor_Protect_FaultSampleIsPending());
    Motor_Protect_CaptureFaultSample(&motor_a, .08f, 100, 200, 5600, 16);
    Motor_Protect_GetFaultSample(&sample);
    assert(!sample.valid);
    Motor_Protect_GetFaultSample(NULL);
    printf("test_motor_protect: pass\n");
    return 0;
}
