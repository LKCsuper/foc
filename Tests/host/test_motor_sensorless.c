/* 无感观测器主机测试：参数下界、PLL 限幅、接管混合与诊断统计 */
/* Includes ------------------------------------------------------------------*/
#include "test_common_assert.h"
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
extern float host_speed_feedback_rpm;
extern bool host_speed_feedback_valid;
extern unsigned host_speed_feedback_calls;
extern mc_fault_code host_protect_fault;
extern unsigned host_protect_fault_calls;

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 验证参数下限与非法输入拒绝。
 * @return {void} 无
 */
static void check_parameter_lower_bounds(void)
{
    const float values[] = {-INFINITY, -3.0f, -0.0f, 0.0f, 0.0000001f, 2.0f, INFINITY, NAN};
    unsigned field, index;
    for (field = 0; field < 4U; ++field) {
        for (index = 0; index < sizeof(values) / sizeof(values[0]); ++index) {
            MOTOR_PARAM_S actual = {0}, reference;
            motor_sensorless_runtime_t actual_state, reference_state;
            motor_sensorless_effects_t actual_effects, reference_effects;
            float *actual_value, *reference_value;
            float minimum = field < 2U ? 0.0f : (field == 2U ? 0.000001f : 1.0f);
            actual.ol_enable = true;
            actual.ol_mode = MOTOR_OL_MODE_IF;
            actual.sensorless_enable = true;
            actual.sensorless_takeover = true;
            actual.sensorless_flux_linkage = 0.0023f;
            actual.sensorless_motor_l = 0.0001f;
            actual.sensorless_motor_r = 0.47f;
            actual.sensorless_observer_gain = 12000000.0f;
            actual.ol_pole_pairs = 7.0f;
            actual.ol_max_rpm = 6000.0f;
            actual.ia = 0.1f;
            actual.ib = -0.1f;
            actual.ol_current_rpm = 300.0f;
            reference = actual;
            actual_value = field == 0U ? &actual.sensorless_motor_r :
                           field == 1U ? &actual.sensorless_motor_l :
                           field == 2U ? &actual.sensorless_flux_linkage : &actual.ol_pole_pairs;
            reference_value = field == 0U ? &reference.sensorless_motor_r :
                              field == 1U ? &reference.sensorless_motor_l :
                              field == 2U ? &reference.sensorless_flux_linkage : &reference.ol_pole_pairs;
            *actual_value = values[index];
            *reference_value = fmaxf(values[index], minimum);
            Motor_Sensorless_InitState(&actual_state);
            actual_state.initialized = true;
            actual_state.x_alpha = 0.0023f;
            reference_state = actual_state;
            Motor_Sensorless_UpdateState(&actual_state, &actual, 0.000067f, &actual_effects);
            Motor_Sensorless_UpdateState(&reference_state, &reference, 0.000067f, &reference_effects);
            assert(memcmp(&actual_state, &reference_state, sizeof(actual_state)) == 0);
            assert(actual_effects.request_speed_loss_fault == reference_effects.request_speed_loss_fault);
            assert(actual_effects.feedback_valid == reference_effects.feedback_valid);
        }
    }
}

/**
 * @description: 验证 PLL 限制参数更新。
 * @return {void} 无
 */
static void check_pll_limit_updates(void)
{
    MOTOR_PARAM_S motor = {0};
    motor_sensorless_runtime_t runtime;
    motor_sensorless_effects_t effects;
    const float pairs[] = {7.0f, 2.0f, 0.0f, -3.0f};
    unsigned index;
    motor.ol_enable = true;
    motor.ol_mode = MOTOR_OL_MODE_IF;
    motor.sensorless_enable = true;
    motor.sensorless_takeover = true;
    motor.sensorless_flux_linkage = 0.0023f;
    motor.sensorless_observer_gain = 1.0f;
    Motor_Sensorless_InitState(&runtime);
    for (index = 0; index < sizeof(pairs) / sizeof(pairs[0]); ++index) {
        float expected;
        motor.ol_pole_pairs = pairs[index];
        motor.ol_max_rpm = index & 1U ? -300.0f : 600.0f;
        expected = fabsf(motor.ol_max_rpm) * fmaxf(pairs[index], 1.0f) * 1.5f *
                   (6.283185307f / 60.0f);
        runtime.initialized = true;
        runtime.x_alpha = motor.sensorless_flux_linkage;
        runtime.x_beta = 0.0f;
        runtime.pll_speed_electrical = index & 1U ? -100000.0f : 100000.0f;
        Motor_Sensorless_UpdateState(&runtime, &motor, 0.000001f, &effects);
        assert(fabsf(fabsf(runtime.pll_speed_electrical) - expected) < 0.0001f);
        assert((runtime.pll_speed_electrical < 0.0f) == ((index & 1U) != 0U));
    }
}

/**
 * @description: 验证观测器单步采集记录。
 * @return {void} 无
 */
static void check_observer_step_capture(void)
{
    MOTOR_PARAM_S captured_motor = {0}, reference_motor;
    motor_sensorless_runtime_t reference;
    motor_sensorless_effects_t effects;
    unsigned index;
    float previous_alpha = 0.0f, previous_beta = 0.0f;
    captured_motor.ol_mode = MOTOR_OL_MODE_IF; captured_motor.ol_enable = true;
    captured_motor.sensorless_enable = true; captured_motor.sensorless_takeover = true;
    captured_motor.sensorless_motor_r = 0.6f;
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

/**
 * @description: 静止角差隔离角度过渡：半程应消除一半角差，不能几毫秒就贴到PLL。
 * @param {float} start start 参数
 * @param {float} target 目标机械转速 (RPM)
 * @param {float} omega omega 参数
 * @return {void} 无
 */
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

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
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

    check_parameter_lower_bounds();
    check_pll_limit_updates();
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

    /* 撤销授权后低速退出观测角并清成冷态，不请求故障，也不再积分。 */
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
    assert(a.initialized && a.x_alpha != 0.0f);
    assert(fabsf(Motor_Sensorless_GetControlAngleState(&a, motor.ol_theta) - motor.ol_theta) < 1e-6f);
    a.stats.samples = 3U;
    Motor_Sensorless_UpdateState(&a, &motor, 0.00001f, &effects);
    assert(a.initialized && a.x_alpha != 0.0f && a.stats.samples == 3U);
    assert(!effects.request_speed_loss_fault);

    /* 高速撤销授权必须请求故障，不能退回超上限 I/F。 */
    a.state = MOTOR_SENSORLESS_CLOSED;
    a.initialized = true;
    a.blend = 0.5f;
    a.x_alpha = 0.02f;
    motor.ol_current_rpm = 1200.0f;
    Motor_Sensorless_UpdateState(&a, &motor, 0.00001f, &effects);
    assert(effects.request_speed_loss_fault && !effects.feedback_valid);
    assert(!Motor_Sensorless_IsClosedLoopState(&a));
    assert(a.blend == 0.0f && a.initialized);

    /* 重新授权后检查相位差诊断。 */
    motor.sensorless_takeover = true;
    motor.ol_current_rpm = 600.0f;
    Motor_Sensorless_InitState(&a);
    a.initialized = true;
    a.x_alpha = 0.02f;
    a.pll_angle = 0.01f;
    a.pll_speed_electrical = 300.0f;
    motor.ol_theta = 0.02f;
    Motor_Sensorless_UpdateState(&a, &motor, 0.00001f, &effects);
    assert(a.initialized && a.x_alpha != 0.0f);
    Motor_Sensorless_GetDiagnosticsState(&a, &diag_a);
    assert(fabsf(diag_a.phase_error - (diag_a.pll_angle - diag_a.reference_angle)) < 1e-4f);
    assert(diag_a.phase_ok == (fabsf(diag_a.lock_phase_error) <= motor.sensorless_phase_tolerance));
    angle_a = diag_a.pll_angle;
    a.pll_angle = 2.0f;
    assert(diag_a.pll_angle == angle_a);

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
        EXPECT_COMMON_ASSERT(Motor_Sensorless_GetLightSnapshotState(&a, NULL));
        assert(memcmp(&a, &b, sizeof(a)) == 0);
    }
    /* 未授权时继续观测，但不发布有效速度或切换控制角。 */
    {
        MOTOR_PARAM_S cold = {0};
        unsigned feedback_calls;
        unsigned fault_calls;
        cold.ol_enable = true;
        cold.ol_mode = MOTOR_OL_MODE_IF;
        cold.sensorless_enable = true;
        cold.sensorless_takeover = true;
        cold.sensorless_motor_r = 0.4f;
        cold.sensorless_motor_l = 0.0008f;
        cold.sensorless_flux_linkage = 0.0023f;
        cold.sensorless_observer_gain = 0.0f;
        cold.ol_pole_pairs = 7.0f;
        cold.ol_max_rpm = 6000.0f;
        cold.ol_current_rpm = 300.0f;
        cold.ia = 0.2f;
        cold.ib = -0.1f;
        cold.ic = -0.1f;
        Motor_Sensorless_Init();
        Motor_Sensorless_SetAppliedVoltage(1.0f, 0.2f);
        Motor_Sensorless_Update(&cold, 0.000067f);
        assert(Motor_Sensorless_GetFluxMagnitude() > 0.0f);
        cold.sensorless_takeover = false;
        fault_calls = host_protect_fault_calls;
        Motor_Sensorless_Update(&cold, 0.000067f);
        assert(Motor_Sensorless_GetFluxMagnitude() > 0.0f);
        assert(Motor_Sensorless_GetState() == MOTOR_SENSORLESS_OPEN_LOOP);
        assert(host_protect_fault_calls == fault_calls);
        feedback_calls = host_speed_feedback_calls;
        Motor_Sensorless_Update(&cold, 0.000067f);
        assert(Motor_Sensorless_GetFluxMagnitude() > 0.0f);
        assert(host_speed_feedback_calls == feedback_calls + 1U);
        assert(!host_speed_feedback_valid && host_speed_feedback_rpm == 0.0f);
        assert(fabsf(Motor_Sensorless_GetControlAngle(0.4f) - 0.4f) < 1e-6f);
        host_observer_step_active = true;
        host_observer_step_count = 0U;
        cold.sensorless_takeover = true;
        Motor_Sensorless_Update(&cold, 0.000067f);
        Motor_Sensorless_Update(&cold, 0.000067f);
        assert(host_observer_step_count == 2U);
        assert(host_observer_step.previous_dt == 0.000067f);
        Motor_Sensorless_Init();
        host_observer_step_active = false;
        cold.sensorless_takeover = false;
        Motor_Sensorless_Update(&cold, 0.000067f);
        host_observer_step_active = true;
        cold.sensorless_takeover = true;
        Motor_Sensorless_Update(&cold, 0.000067f);
        assert(host_observer_step_count == 3U);
        assert(host_observer_step.previous_dt == 0.0f);
        host_observer_step_active = false;
        Motor_Sensorless_Update(&cold, 0.000067f);
        Motor_Sensorless_Init();
    }
    printf("test_motor_sensorless: pass\n");
    return 0;
}
