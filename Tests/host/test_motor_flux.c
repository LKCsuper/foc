#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_flux.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float wrap_pi(float angle)
{
    const float pi = (float)M_PI;
    const float two_pi = 2.0f * pi;

    while (angle >= pi) {
        angle -= two_pi;
    }
    while (angle < -pi) {
        angle += two_pi;
    }
    return angle;
}

static void pmsm_voltage(float theta, float omega_e, float iq, const motor_flux_config_t *cfg, float *v_alpha,
                         float *v_beta, float *i_alpha, float *i_beta)
{
    float s = sinf(theta);
    float c = cosf(theta);
    float flux = cfg->flux_linkage;

    *i_alpha = -iq * s;
    *i_beta = iq * c;
    *v_alpha = cfg->resistance * (*i_alpha) - cfg->inductance * iq * omega_e * c - omega_e * flux * s;
    *v_beta = cfg->resistance * (*i_beta) - cfg->inductance * iq * omega_e * s + omega_e * flux * c;
}

static void default_config(motor_flux_config_t *cfg)
{
    cfg->enabled = true;
    cfg->resistance = 0.4f;
    cfg->inductance = 0.0008f;
    cfg->flux_linkage = 0.02573f;
    cfg->observer_gain = 12000000.0f;
    cfg->phase_offset = 0.0f;
}

int main(void)
{
    motor_flux_t obs;
    motor_flux_config_t cfg;
    motor_flux_output_t out;
    float v_alpha;
    float v_beta;
    float i_alpha;
    float i_beta;
    float dt;
    float theta;
    float omega_e;
    float err;
    unsigned i;

    Motor_Flux_Reset(&obs);
    assert(!obs.initialized);
    assert(fabsf(obs.x_alpha) < 1e-9f);
    assert(fabsf(Motor_Flux_GetAngle(&obs)) < 1e-9f);
    assert(fabsf(Motor_Flux_GetMagnitude(&obs)) < 1e-9f);

    default_config(&cfg);
    cfg.enabled = false;
    dt = 1.0f / 15000.0f;
    assert(!Motor_Flux_Step(&obs, &cfg, 1.0f, 0.0f, 0.0f, 0.0f, dt, &out));
    assert(!out.valid);
    assert(!obs.initialized);

    cfg.resistance = -1.0f;
    cfg.inductance = -0.2f;
    cfg.flux_linkage = 0.0f;
    cfg.observer_gain = -5.0f;
    cfg.phase_offset = NAN;
    Motor_Flux_NormalizeConfig(&cfg);
    assert(fabsf(cfg.resistance) < 1e-9f);
    assert(fabsf(cfg.inductance) < 1e-9f);
    assert(fabsf(cfg.flux_linkage - MOTOR_FLUX_MIN_LINKAGE) < 1e-12f);
    assert(fabsf(cfg.observer_gain) < 1e-9f);
    assert(fabsf(cfg.phase_offset) < 1e-9f);

    default_config(&cfg);
    Motor_Flux_NormalizeConfig(&cfg);
    Motor_Flux_Reset(&obs);
    Motor_Flux_Seed(&obs, &cfg, 0.40f, 0.0f, 0.0f);
    assert(obs.initialized);
    assert(fabsf(Motor_Flux_GetAngle(&obs) - 0.40f) < 1e-5f);
    assert(fabsf(Motor_Flux_GetMagnitude(&obs) - cfg.flux_linkage) < 1e-5f);
    assert(fabsf(obs.psi_alpha - cfg.flux_linkage * cosf(0.40f)) < 1e-5f);
    assert(fabsf(obs.psi_beta - cfg.flux_linkage * sinf(0.40f)) < 1e-5f);

    /* 零电流、额定电频率：理想反电势应让观测角跟着真角走。 */
    omega_e = 2.0f * (float)M_PI * 100.0f;
    theta = 0.40f;
    for (i = 0U; i < 300U; ++i) {
        pmsm_voltage(theta, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        assert(Motor_Flux_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out));
        assert(out.valid);
        assert(out.flux_ok);
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Flux_GetAngle(&obs) - theta));
    assert(err < 0.08f);
    assert(fabsf(Motor_Flux_GetMagnitude(&obs) - cfg.flux_linkage) / cfg.flux_linkage < 0.10f);

    /* 带 Iq 时必须扣掉 L×I，观测角仍应对准转子。 */
    Motor_Flux_Reset(&obs);
    theta = 0.20f;
    Motor_Flux_Seed(&obs, &cfg, theta, -2.0f * sinf(theta), 2.0f * cosf(theta));
    for (i = 0U; i < 400U; ++i) {
        pmsm_voltage(theta, omega_e, 2.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        assert(Motor_Flux_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out));
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Flux_GetAngle(&obs) - theta));
    assert(err < 0.10f);
    assert(out.flux_ok);

    /* 初值偏差后，电压方程积分应把磁链方向拉向真角。 */
    Motor_Flux_Reset(&obs);
    theta = 1.00f;
    Motor_Flux_Seed(&obs, &cfg, 0.50f, 0.0f, 0.0f);
    for (i = 0U; i < 1200U; ++i) {
        pmsm_voltage(theta, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        Motor_Flux_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out);
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Flux_GetAngle(&obs) - theta));
    assert(err < 0.15f);

    /* Ortega 单边修正：磁链偏大时应被拉回额定圆附近。 */
    Motor_Flux_Reset(&obs);
    Motor_Flux_Seed(&obs, &cfg, 0.0f, 0.0f, 0.0f);
    obs.x_alpha *= 2.0f;
    obs.x_beta *= 2.0f;
    theta = 0.0f;
    for (i = 0U; i < 400U; ++i) {
        pmsm_voltage(theta, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        Motor_Flux_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out);
        theta = wrap_pi(theta + omega_e * dt);
    }
    assert(Motor_Flux_GetMagnitude(&obs) < cfg.flux_linkage * 1.25f);

    /* 相位补偿加在 atan2 之后。 */
    Motor_Flux_Reset(&obs);
    cfg.phase_offset = 0.30f;
    Motor_Flux_Seed(&obs, &cfg, 0.10f, 0.0f, 0.0f);
    assert(fabsf(wrap_pi(Motor_Flux_GetAngle(&obs) - 0.40f)) < 1e-5f);
    cfg.phase_offset = 0.0f;

    /* 非法 dt / NaN 电压必须复位，不能把脏角留给 Park。 */
    Motor_Flux_Seed(&obs, &cfg, 0.2f, 0.0f, 0.0f);
    assert(!Motor_Flux_Step(&obs, &cfg, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, &out));
    assert(!obs.initialized);
    Motor_Flux_Seed(&obs, &cfg, 0.2f, 0.0f, 0.0f);
    assert(!Motor_Flux_Step(&obs, &cfg, NAN, 0.0f, 0.0f, 0.0f, dt, &out));
    assert(!obs.initialized);
    assert(!out.valid);

    /* 双实例互不影响 */
    {
        motor_flux_t a;
        motor_flux_t b;

        Motor_Flux_Reset(&a);
        Motor_Flux_Reset(&b);
        Motor_Flux_Seed(&a, &cfg, 0.2f, 0.0f, 0.0f);
        Motor_Flux_Seed(&b, &cfg, 2.0f, 0.0f, 0.0f);
        pmsm_voltage(0.2f, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        Motor_Flux_Step(&a, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out);
        assert(fabsf(Motor_Flux_GetAngle(&b) - 2.0f) < 1e-5f);
        assert(fabsf(b.x_alpha - cfg.flux_linkage * cosf(2.0f)) < 1e-5f);
    }

    /* 空指针安全 */
    Motor_Flux_Reset(NULL);
    Motor_Flux_NormalizeConfig(NULL);
    Motor_Flux_Seed(NULL, &cfg, 1.0f, 0.0f, 0.0f);
    assert(fabsf(Motor_Flux_GetAngle(NULL)) < 1e-9f);
    assert(fabsf(Motor_Flux_GetMagnitude(NULL)) < 1e-9f);
    assert(!Motor_Flux_Step(NULL, &cfg, 0.0f, 0.0f, 0.0f, 0.0f, dt, &out));

    printf("test_motor_flux: PASS\n");
    return 0;
}
