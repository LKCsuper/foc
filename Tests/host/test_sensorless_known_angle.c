/* 无感已知转子角主机测试：真实磁链方程下的角度误差 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_sensorless.h"

/* 已知转子磁链与独立电压方程，不以开环参考角冒充真实转子角。 */
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 运行当前测试场景。
 * @param {float} load_angle load_angle 参数
 * @param {float} voltage_bias voltage_bias 参数
 * @param {int} cold_start cold_start 参数
 * @param {float} gain gain 参数
 * @param {float} true_flux true_flux 参数
 * @param {float} model_flux model_flux 参数
 * @return {double} 本次计算或查询的结果
 */
static double run(float load_angle, float voltage_bias, int cold_start, float gain,
                  float true_flux, float model_flux)
{
    MOTOR_PARAM_S m;
    motor_sensorless_runtime_t rt;
    motor_sensorless_effects_t effects;
    const float dt = 1.0f / 15000.0f;
    const float omega = 600.0f * 7.0f * 6.283185307f / 60.0f;
    double error_sum = 0.0;
    float maximum = 0.0f;
    float previous_a = 0.0f, previous_b = 1.0f;
    float previous_flux_a = true_flux * cosf(load_angle);
    float previous_flux_b = true_flux * sinf(load_angle);
    unsigned n;
    memset(&m, 0, sizeof(m));
    Motor_Sensorless_InitState(&rt);
    m.ol_enable = true;
    m.ol_mode = MOTOR_OL_MODE_IF;
    m.ol_pole_pairs = 7.0f;
    m.ol_current_rpm = m.ol_target_rpm = 600.0f;
    m.ol_iq_set = 1.0f;
    m.ol_max_rpm = 5920.0f;
    m.sensorless_enable = true;
    m.sensorless_takeover = true;
    m.sensorless_motor_r = 0.474f;
    m.sensorless_motor_l = 0.000112f;
    m.sensorless_flux_linkage = model_flux;
    m.sensorless_observer_gain = gain;
    m.sensorless_pll_kp = 2000.0f;
    m.sensorless_pll_ki = 30000.0f;
    m.sensorless_speed_filter = 0.02f;
    m.sensorless_transition_rpm = 500.0f;
    m.sensorless_phase_tolerance = 1.4f;
    m.sensorless_lock_time = 0.1f;
    rt.initialized = !cold_start;
    rt.x_alpha = previous_flux_a;
    rt.x_beta = previous_flux_b + m.sensorless_motor_l;
    rt.pll_angle = load_angle;
    rt.pll_speed_electrical = omega;
    for (n = 1; n <= 45000; ++n) {
        float theta = remainderf(n * dt * omega, 6.283185307f);
        float truth = remainderf(theta + load_angle, 6.283185307f);
        float ia = -sinf(theta), ibeta = cosf(theta);
        float fa = true_flux * cosf(truth);
        float fb = true_flux * sinf(truth);
        float va = m.sensorless_motor_r * ia + m.sensorless_motor_l * (ia - previous_a) / dt +
                   (fa - previous_flux_a) / dt + voltage_bias;
        float vb = m.sensorless_motor_r * ibeta + m.sensorless_motor_l * (ibeta - previous_b) / dt +
                   (fb - previous_flux_b) / dt;
        m.ia = ia;
        m.ib = -0.5f * ia + 0.866025404f * ibeta;
        m.ic = -0.5f * ia - 0.866025404f * ibeta;
        m.ol_theta = theta;
        Motor_Sensorless_SetAppliedVoltageState(&rt, va, vb);
        Motor_Sensorless_UpdateState(&rt, &m, dt, &effects);
        if (n > 15000) {
            float error = remainderf(rt.pll_angle - truth, 6.283185307f);
            error_sum += error * error;
            if (fabsf(error) > maximum) maximum = fabsf(error);
        }
        previous_a = ia;
        previous_b = ibeta;
        previous_flux_a = fa;
        previous_flux_b = fb;
    }
    printf("load=%.3f bias=%.3f cold=%d gain=%.0f true_error_rms=%.4f max=%.4f phase_fail=%u/%u longest=%.4f\n",
           load_angle, voltage_bias, cold_start, gain, sqrt(error_sum / 30000.0), maximum, rt.stats.phase_fail,
           rt.stats.samples, rt.stats.longest_candidate);
    printf("flux true=%.6f model=%.6f center=%.6fWb\n", true_flux, model_flux,
           hypotf(rt.stats.flux_alpha_sum, rt.stats.flux_beta_sum) / rt.stats.samples);
    if (voltage_bias == 0.0f) assert(maximum < 0.08f);
    if (load_angle > 1.4f && voltage_bias == 0.0f) {
        assert(rt.stats.phase_fail == 0U);
        assert(rt.stats.longest_candidate > 0.1f);
    }
    if (load_angle < -1.4f && voltage_bias == 0.0f) {
        assert(rt.stats.phase_fail > rt.stats.samples / 2U);
    }
    return sqrt(error_sum / 30000.0);
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    double biased_error, corrected_error;
    run(1.2f, 0.0f, 0, 12000000.0f, 0.00239f, 0.00239f);
    run(1.5f, 0.0f, 0, 12000000.0f, 0.00239f, 0.00239f);
    run(-1.5f, 0.0f, 0, 12000000.0f, 0.00239f, 0.00239f);
    biased_error = run(1.2f, 0.03f, 0, 12000000.0f, 0.00239f, 0.00239f);
    run(1.2f, 0.0f, 1, 12000000.0f, 0.00239f, 0.00239f);
    corrected_error = run(1.2f, 0.03f, 0, 60000000.0f, 0.00239f, 0.00239f);
    assert(corrected_error < biased_error);
    biased_error = run(1.2f, 0.013f, 0, 120000000.0f, 0.00205f, 0.00239f);
    corrected_error = run(1.2f, 0.013f, 0, 120000000.0f, 0.00205f, 0.00205f);
    assert(corrected_error < biased_error);
    return 0;
}
