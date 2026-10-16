#include "test_common_assert.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_smo.h"

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

static void pmsm_voltage(float theta, float omega_e, float iq, const motor_smo_config_t *cfg, float *v_alpha,
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

static void default_config(motor_smo_config_t *cfg)
{
    cfg->enabled = true;
    cfg->resistance = 0.4f;
    cfg->inductance = 0.0008f;
    cfg->flux_linkage = 0.02573f;
    cfg->slide_gain = 0.0f;
    cfg->lpf = 0.0f;
    cfg->sigmoid_eps = 0.0f;
    cfg->phase_offset = 0.0f;
}

int main(void)
{
    motor_smo_t obs;
    motor_smo_config_t cfg;
    motor_smo_output_t out;
    float v_alpha;
    float v_beta;
    float i_alpha;
    float i_beta;
    float dt;
    float theta;
    float omega_e;
    float err;
    unsigned i;

    Motor_Smo_Reset(&obs);
    assert(!obs.initialized);
    assert(fabsf(Motor_Smo_GetAngle(&obs)) < 1e-9f);
    assert(fabsf(Motor_Smo_GetMagnitude(&obs)) < 1e-9f);
    assert(fabsf(Motor_Smo_GetSpeedE(&obs)) < 1e-9f);

    default_config(&cfg);
    cfg.enabled = false;
    dt = 1.0f / 15000.0f;
    assert(!Motor_Smo_Step(&obs, &cfg, 1.0f, 0.0f, 0.0f, 0.0f, dt, &out));
    assert(!out.valid);
    assert(!obs.initialized);

    cfg.resistance = -1.0f;
    cfg.inductance = 0.0f;
    cfg.flux_linkage = 0.0f;
    cfg.slide_gain = -5.0f;
    cfg.lpf = 2.0f;
    cfg.sigmoid_eps = -0.1f;
    cfg.phase_offset = NAN;
    Motor_Smo_NormalizeConfig(&cfg);
    assert(fabsf(cfg.resistance) < 1e-9f);
    assert(fabsf(cfg.inductance - MOTOR_SMO_MIN_INDUCTANCE) < 1e-12f);
    assert(fabsf(cfg.flux_linkage - MOTOR_SMO_MIN_LINKAGE) < 1e-12f);
    assert(fabsf(cfg.slide_gain) < 1e-9f);
    assert(fabsf(cfg.lpf) < 1e-9f);
    assert(fabsf(cfg.sigmoid_eps) < 1e-9f);
    assert(fabsf(cfg.phase_offset) < 1e-9f);

    default_config(&cfg);
    Motor_Smo_NormalizeConfig(&cfg);
    omega_e = 2.0f * (float)M_PI * 100.0f;
    Motor_Smo_Reset(&obs);
    Motor_Smo_Seed(&obs, &cfg, 0.40f, omega_e, 0.0f, 0.0f);
    assert(obs.initialized);
    assert(fabsf(Motor_Smo_GetAngle(&obs) - 0.40f) < 1e-5f);
    assert(fabsf(Motor_Smo_GetSpeedE(&obs) - omega_e) < 1e-4f);
    assert(fabsf(obs.e_alpha + omega_e * cfg.flux_linkage * sinf(0.40f)) < 1e-5f);

    theta = 0.40f;
    for (i = 0U; i < 600U; ++i) {
        pmsm_voltage(theta, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        assert(Motor_Smo_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out));
        assert(out.valid);
        assert(out.bemf_ok);
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Smo_GetAngle(&obs) - theta));
    assert(err < 0.15f);
    assert(fabsf(Motor_Smo_GetSpeedE(&obs) - omega_e) / omega_e < 0.20f);

    Motor_Smo_Reset(&obs);
    theta = 0.20f;
    Motor_Smo_Seed(&obs, &cfg, theta, omega_e, -2.0f * sinf(theta), 2.0f * cosf(theta));
    for (i = 0U; i < 600U; ++i) {
        pmsm_voltage(theta, omega_e, 2.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        assert(Motor_Smo_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out));
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Smo_GetAngle(&obs) - theta));
    assert(err < 0.18f);
    assert(out.bemf_ok);

    Motor_Smo_Reset(&obs);
    theta = 1.00f;
    Motor_Smo_Seed(&obs, &cfg, 0.50f, omega_e, 0.0f, 0.0f);
    for (i = 0U; i < 1500U; ++i) {
        pmsm_voltage(theta, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        Motor_Smo_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out);
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Smo_GetAngle(&obs) - theta));
    assert(err < 0.20f);

    Motor_Smo_Reset(&obs);
    omega_e = -2.0f * (float)M_PI * 80.0f;
    theta = 0.30f;
    Motor_Smo_Seed(&obs, &cfg, theta, omega_e, 0.0f, 0.0f);
    for (i = 0U; i < 700U; ++i) {
        pmsm_voltage(theta, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        assert(Motor_Smo_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out));
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Smo_GetAngle(&obs) - theta));
    assert(err < 0.18f);
    assert(Motor_Smo_GetSpeedE(&obs) < 0.0f);

    Motor_Smo_Reset(&obs);
    cfg.phase_offset = 0.30f;
    Motor_Smo_Seed(&obs, &cfg, 0.10f, 100.0f, 0.0f, 0.0f);
    assert(fabsf(wrap_pi(Motor_Smo_GetAngle(&obs) - 0.40f)) < 1e-5f);
    cfg.phase_offset = 0.0f;

    omega_e = 2.0f * (float)M_PI * 100.0f;
    Motor_Smo_Seed(&obs, &cfg, 0.2f, omega_e, 0.0f, 0.0f);
    assert(!Motor_Smo_Step(&obs, &cfg, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, &out));
    assert(!obs.initialized);
    Motor_Smo_Seed(&obs, &cfg, 0.2f, omega_e, 0.0f, 0.0f);
    assert(!Motor_Smo_Step(&obs, &cfg, NAN, 0.0f, 0.0f, 0.0f, dt, &out));
    assert(!obs.initialized);
    assert(!out.valid);

    {
        motor_smo_t a;
        motor_smo_t b;

        Motor_Smo_Reset(&a);
        Motor_Smo_Reset(&b);
        Motor_Smo_Seed(&a, &cfg, 0.2f, omega_e, 0.0f, 0.0f);
        Motor_Smo_Seed(&b, &cfg, 2.0f, omega_e, 0.0f, 0.0f);
        pmsm_voltage(0.2f, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        Motor_Smo_Step(&a, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out);
        assert(fabsf(Motor_Smo_GetAngle(&b) - 2.0f) < 1e-5f);
        assert(fabsf(b.e_beta - omega_e * cfg.flux_linkage * cosf(2.0f)) < 1e-5f);
    }

    Motor_Smo_Reset(NULL);
    EXPECT_COMMON_ASSERT(Motor_Smo_NormalizeConfig(NULL));
    EXPECT_COMMON_ASSERT(Motor_Smo_Seed(NULL, &cfg, 1.0f, 1.0f, 0.0f, 0.0f));
    assert(fabsf(Motor_Smo_GetAngle(NULL)) < 1e-9f);
    assert(fabsf(Motor_Smo_GetMagnitude(NULL)) < 1e-9f);
    assert(fabsf(Motor_Smo_GetSpeedE(NULL)) < 1e-9f);
    assert(!Motor_Smo_Step(NULL, &cfg, 0.0f, 0.0f, 0.0f, 0.0f, dt, &out));

    printf("test_motor_smo: PASS\n");
    return 0;
}
