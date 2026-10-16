#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_sensorless.h"
#include "motor_param.h"
#include "motor_timing.h"

extern bool host_observer_step_active;
extern motor_observer_step_sample_t host_observer_step;
extern unsigned host_observer_step_count;

static void check_observer_step_capture(void)
{
    MOTOR_PARAM_S captured_motor = {0}, reference_motor;
    motor_sensorless_runtime_t reference;
    motor_sensorless_effects_t effects;
    unsigned index;
    float previous_alpha = 0.0f, previous_beta = 0.0f;
    captured_motor.ol_mode = MOTOR_OL_MODE_IF; captured_motor.ol_enable = true;
    captured_motor.sensorless_enable = true; captured_motor.sensorless_motor_r = 0.6f;
    captured_motor.sensorless_motor_l = 0.00013f; captured_motor.sensorless_flux_linkage = 0.0023f;
    captured_motor.sensorless_observer_gain = 12000000.0f;
    captured_motor.sensorless_pll_kp = 2000.0f; captured_motor.sensorless_pll_ki = 30000.0f;
    captured_motor.ol_max_rpm = 6000.0f; captured_motor.ol_pole_pairs = 7.0f;
    Motor_Sensorless_Init(); Motor_Sensorless_InitState(&reference);
    host_observer_step_active = true; host_observer_step_count = 0U;
    for (index = 0U; index < 300U; index++) {
        float magnitude_error, rotor_alpha, rotor_beta;
        const float dt = 0.000067f;
        motor_observer_step_sample_t *step = &host_observer_step;
        captured_motor.ia = 0.3f * sinf(index * 0.03f);
        captured_motor.ib = 0.2f * cosf(index * 0.03f);
        captured_motor.ic = -captured_motor.ia - captured_motor.ib;
        reference_motor = captured_motor;
        Motor_Sensorless_SetAppliedVoltage(0.8f, 0.5f);
        Motor_Sensorless_SetAppliedVoltageState(&reference, 0.8f, 0.5f);
        Motor_Sensorless_Update(&captured_motor, dt);
        Motor_Sensorless_UpdateState(&reference, &reference_motor, dt, &effects);
        assert(host_observer_step_count == index + 1U);
        assert(fabsf(step->x_after_alpha - reference.x_alpha) < 1e-9f);
        assert(fabsf(step->x_after_beta - reference.x_beta) < 1e-9f);
        assert(step->observer == reference.observer_angle);
        rotor_alpha = step->x_before_alpha - step->inductance * step->i_alpha;
        rotor_beta = step->x_before_beta - step->inductance * step->i_beta;
        magnitude_error = fminf(0.0f, step->flux_nominal * step->flux_nominal - rotor_alpha * rotor_alpha - rotor_beta * rotor_beta);
        assert(fabsf(step->correction_alpha - 0.5f * step->gain * rotor_alpha * magnitude_error) < 1e-6f);
        assert(fabsf(step->x_after_alpha - step->x_before_alpha -
                     (step->v_alpha - step->resistance * step->i_alpha + step->correction_alpha) * dt) < 1e-9f);
        if (index > 0U) {
            assert(step->previous_dt == dt);
            assert(step->i_before_alpha == previous_alpha && step->i_before_beta == previous_beta);
        } else assert(step->previous_dt == 0.0f);
        previous_alpha = step->i_alpha; previous_beta = step->i_beta;
    }
    host_observer_step_active = false;
    Motor_Sensorless_Update(&captured_motor, 0.000067f);
    assert(host_observer_step_count == 300U);
    Motor_Sensorless_Init();
}

/* 静止角差隔离角度过渡：半程应消除一半角差，不能几毫秒就贴到PLL。 */
static void check_blend_duration(float start, float target, float omega)
{
    MOTOR_PARAM_S m;
    motor_sensorless_runtime_t rt;
    motor_sensorless_effects_t effects;
    const float dt = 1.0f / 15000.0f;
    float initial_error = remainderf(target - start, 6.283185307f);
    float previous_flux_a, previous_flux_b;
    unsigned n;
    memset(&m, 0, sizeof(m));
    Motor_Sensorless_InitState(&rt);
    m.ol_enable = true;
    m.ol_mode = MOTOR_OL_MODE_IF;
    m.sensorless_enable = m.sensorless_takeover = true;
    m.sensorless_flux_linkage = 0.00239f;
    m.sensorless_observer_gain = 12000000.0f;
    m.sensorless_blend_time = 0.2f;
    m.sensorless_loss_time = 100.0f;
    m.ol_theta = start;
    m.ol_pole_pairs = 7.0f;
    m.ol_max_rpm = 1000.0f;
    rt.initialized = true;
    rt.state = MOTOR_SENSORLESS_CLOSED;
    rt.x_alpha = m.sensorless_flux_linkage * cosf(target);
    rt.x_beta = m.sensorless_flux_linkage * sinf(target);
    previous_flux_a = rt.x_alpha;
    previous_flux_b = rt.x_beta;
    rt.pll_angle = target;
    rt.pll_speed_electrical = omega;
    for (n = 1; n <= 4500; ++n) {
        float moving_target = remainderf(target + n * dt * omega, 6.283185307f);
        float fa = m.sensorless_flux_linkage * cosf(moving_target);
        float fb = m.sensorless_flux_linkage * sinf(moving_target);
        /* 独立等速转子轨迹；调用者每拍推进强制角，再由控制器回写。 */
        m.ol_theta = remainderf(m.ol_theta + omega * dt, 6.283185307f);
        Motor_Sensorless_SetAppliedVoltageState(&rt, (fa - previous_flux_a) / dt, (fb - previous_flux_b) / dt);
        previous_flux_a = fa;
        previous_flux_b = fb;
        Motor_Sensorless_UpdateState(&rt, &m, dt, &effects);
        if (n > 3010) {
            assert(rt.control_angle == rt.pll_angle);
            assert(m.ol_theta == rt.pll_angle);
            assert(effects.feedback_valid);
        }
        if (n == 1500 || n == 2700) {
            float expected = initial_error * (1.0f - n / 3000.0f);
            float remaining = remainderf(moving_target - rt.control_angle, 6.283185307f);
            printf("blend n=%u remaining=%.6f expected=%.6f\n", n, remaining, expected);
            assert(fabsf(remaining - expected) < 0.003f);
        }
    }
    assert(Motor_Sensorless_IsTakeoverCompleteState(&rt));
    assert(fabsf(remainderf(target + 4500 * dt * omega - rt.control_angle, 6.283185307f)) < 0.001f);
}

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

    check_observer_step_capture();

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
    motor.sensorless_takeover = true;
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

    /* 撤销授权后低速退出观测角，保留 PLL/磁链，不把反馈继续报为有效。 */
    Motor_Sensorless_InitState(&a);
    a.state = MOTOR_SENSORLESS_CLOSED;
    a.initialized = true;
    a.x_alpha = 0.02f;
    a.pll_angle = 0.0f;
    a.pll_speed_electrical = 300.0f;
    a.blend = 1.0f;
    motor.ol_current_rpm = 600.0f;
    motor.ol_theta = 0.02f;
    motor.sensorless_takeover = false;
    motor.sensorless_phase_tolerance = 1.4f;
    Motor_Sensorless_UpdateState(&a, &motor, 0.00001f, &effects);
    assert(!Motor_Sensorless_IsClosedLoopState(&a));
    assert(!Motor_Sensorless_IsTakeoverCompleteState(&a));
    assert(!effects.feedback_valid && !effects.prime_speed_iq);
    assert(!effects.request_speed_loss_fault);
    assert(a.initialized && a.x_alpha > 0.0f && a.pll_speed_electrical > 200.0f);
    assert(fabsf(Motor_Sensorless_GetControlAngleState(&a, motor.ol_theta) - motor.ol_theta) < 1e-6f);
    Motor_Sensorless_GetDiagnosticsState(&a, &diag_a);
    assert(fabsf(diag_a.phase_error - (diag_a.pll_angle - diag_a.reference_angle)) < 1e-5f);
    assert(diag_a.phase_ok == (fabsf(diag_a.lock_phase_error) <= motor.sensorless_phase_tolerance));
    /* 快照值不会随后续状态写入变化。 */
    angle_a = diag_a.pll_angle;
    a.pll_angle = 2.0f;
    assert(diag_a.pll_angle == angle_a);

    /* 高速撤销授权必须请求故障，不能退回超上限 I/F。 */
    a.state = MOTOR_SENSORLESS_CLOSED;
    a.blend = 0.5f;
    motor.ol_current_rpm = 1200.0f;
    Motor_Sensorless_UpdateState(&a, &motor, 0.00001f, &effects);
    assert(effects.request_speed_loss_fault && !effects.feedback_valid);
    assert(!Motor_Sensorless_IsClosedLoopState(&a));
    assert(a.blend == 0.0f);

    /* 排除升速段、记录每拍失败；停机保留，下次起步重置。 */
    Motor_Sensorless_InitState(&a);
    motor.ol_current_rpm = 600.0f;
    motor.ol_target_rpm = 700.0f;
    motor.sensorless_transition_rpm = 500.0f;
    Motor_Sensorless_UpdateState(&a, &motor, 0.00001f, &effects);
    assert(a.stats.samples == 0U);
    motor.ol_target_rpm = 600.0f;
    Motor_Sensorless_UpdateState(&a, &motor, 0.00001f, &effects);
    assert(a.stats.samples == 1U);
    assert(a.stats.flux_fail == (unsigned)!a.flux_ok);
    assert(a.stats.phase_fail == (unsigned)!a.phase_ok);
    {
        unsigned bin, count = 0, failures = 0;
        for (bin = 0; bin < MOTOR_SENSORLESS_STATS_BINS; ++bin) {
            count += a.stats.bins[bin].samples;
            failures += a.stats.bins[bin].phase_fail;
        }
        assert(count == a.stats.samples / 8U && failures <= a.stats.phase_fail);
    }
    assert(a.stats.flux_min == a.flux_magnitude && a.stats.flux_max == a.flux_magnitude);
    a.stats.longest_candidate = 0.02f;
    Motor_Sensorless_ResetState(&a);
    assert(!a.initialized && a.stats.samples == 1U && a.stats.longest_candidate == 0.02f);
    motor.ol_enable = false;
    Motor_Sensorless_UpdateState(&a, &motor, 0.00001f, &effects);
    assert(a.stats.samples == 1U);
    motor.ol_enable = true;
    motor.ol_current_rpm = 0.0f;
    Motor_Sensorless_UpdateState(&a, &motor, 0.00001f, &effects);
    assert(a.stats.samples == 0U && a.stats.longest_candidate == 0.0f);
    a.stats.elapsed = 10.0f;
    motor.ol_current_rpm = 600.0f;
    Motor_Sensorless_UpdateState(&a, &motor, 0.00001f, &effects);
    assert(a.stats.samples == 0U);

    check_blend_duration(0.0f, 1.2f, 0.0f);
    check_blend_duration(2.8f, -2.3f, 0.0f);
    check_blend_duration(-2.8f, 2.3f, 0.0f);
    check_blend_duration(2.8f, -2.3f, 439.82297f);
    check_blend_duration(-2.8f, 2.3f, -439.82297f);
    {
        motor_sensorless_light_snapshot_t snapshot;
        a.pll_angle = 0.3f; a.flux_magnitude = 0.002f; a.filtered_rpm = 600.0f;
        a.blend = 0.25f; a.loss_timer = 0.001f;
        a.applied_v_alpha = 0.8f; a.applied_v_beta = -0.9f;
        a.state = MOTOR_SENSORLESS_CLOSED; a.flux_ok = true; a.direction_ok = false;
        b = a;
        Motor_Sensorless_GetLightSnapshotState(&a, &snapshot);
        assert(snapshot.state == a.state && snapshot.flux_ok && !snapshot.direction_ok);
        assert(snapshot.pll == a.pll_angle && snapshot.flux == a.flux_magnitude && snapshot.rpm == a.filtered_rpm);
        assert(snapshot.blend == a.blend && snapshot.loss == a.loss_timer);
        assert(snapshot.v_alpha == a.applied_v_alpha && snapshot.v_beta == a.applied_v_beta);
        assert(memcmp(&a, &b, sizeof(a)) == 0);
        Motor_Sensorless_GetLightSnapshotState(&a, NULL);
        assert(memcmp(&a, &b, sizeof(a)) == 0);
    }
    printf("test_motor_sensorless: pass\n");
    return 0;
}
