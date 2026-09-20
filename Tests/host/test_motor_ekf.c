#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_ekf.h"

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

static void pmsm_voltage(float theta, float omega_e, float iq, const motor_ekf_config_t *cfg, float *v_alpha,
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

static void default_config(motor_ekf_config_t *cfg)
{
    cfg->enabled = true;
    cfg->resistance = 0.4f;
    cfg->inductance = 0.0008f;
    cfg->flux_linkage = 0.02573f;
    cfg->q_current = 0.0f;
    cfg->q_speed = 0.0f;
    cfg->q_angle = 0.0f;
    cfg->r_current = 0.0f;
    cfg->phase_offset = 0.0f;
}

int main(void)
{
    motor_ekf_t obs;
    motor_ekf_config_t cfg;
    motor_ekf_output_t out;
    float v_alpha;
    float v_beta;
    float i_alpha;
    float i_beta;
    float dt;
    float theta;
    float omega_e;
    float err;
    unsigned i;

    Motor_Ekf_Reset(&obs);
    assert(!obs.initialized);
    assert(fabsf(Motor_Ekf_GetAngle(&obs)) < 1e-9f);
    assert(fabsf(Motor_Ekf_GetSpeedE(&obs)) < 1e-9f);

    default_config(&cfg);
    cfg.enabled = false;
    dt = 1.0f / 15000.0f;
    assert(!Motor_Ekf_Step(&obs, &cfg, 1.0f, 0.0f, 0.0f, 0.0f, dt, &out));
    assert(!out.valid);
    assert(!obs.initialized);

    cfg.resistance = -1.0f;
    cfg.inductance = 0.0f;
    cfg.flux_linkage = 0.0f;
    cfg.q_current = -1.0f;
    cfg.q_speed = -1.0f;
    cfg.q_angle = -1.0f;
    cfg.r_current = -1.0f;
    cfg.phase_offset = NAN;
    Motor_Ekf_NormalizeConfig(&cfg);
    assert(fabsf(cfg.resistance) < 1e-9f);
    assert(fabsf(cfg.inductance - MOTOR_EKF_MIN_INDUCTANCE) < 1e-12f);
    assert(fabsf(cfg.flux_linkage - MOTOR_EKF_MIN_LINKAGE) < 1e-12f);
    assert(fabsf(cfg.q_current) < 1e-9f);
    assert(fabsf(cfg.phase_offset) < 1e-9f);

    default_config(&cfg);
    Motor_Ekf_NormalizeConfig(&cfg);
    omega_e = 2.0f * (float)M_PI * 100.0f;
    Motor_Ekf_Reset(&obs);
    Motor_Ekf_Seed(&obs, &cfg, 0.40f, omega_e, 0.0f, 0.0f);
    assert(obs.initialized);
    assert(fabsf(Motor_Ekf_GetAngle(&obs) - 0.40f) < 1e-5f);
    assert(fabsf(Motor_Ekf_GetSpeedE(&obs) - omega_e) < 1e-4f);

    theta = 0.40f;
    for (i = 0U; i < 400U; ++i) {
        pmsm_voltage(theta, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        assert(Motor_Ekf_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out));
        assert(out.valid);
        assert(out.speed_ok);
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Ekf_GetAngle(&obs) - theta));
    assert(err < 0.08f);
    assert(fabsf(Motor_Ekf_GetSpeedE(&obs) - omega_e) / omega_e < 0.10f);

    Motor_Ekf_Reset(&obs);
    theta = 0.20f;
    Motor_Ekf_Seed(&obs, &cfg, theta, omega_e, -2.0f * sinf(theta), 2.0f * cosf(theta));
    for (i = 0U; i < 400U; ++i) {
        pmsm_voltage(theta, omega_e, 2.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        assert(Motor_Ekf_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out));
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Ekf_GetAngle(&obs) - theta));
    assert(err < 0.10f);

    Motor_Ekf_Reset(&obs);
    theta = 1.00f;
    Motor_Ekf_Seed(&obs, &cfg, 0.50f, omega_e, 0.0f, 0.0f);
    for (i = 0U; i < 1200U; ++i) {
        pmsm_voltage(theta, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        Motor_Ekf_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out);
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Ekf_GetAngle(&obs) - theta));
    assert(err < 0.15f);

    Motor_Ekf_Reset(&obs);
    omega_e = -2.0f * (float)M_PI * 80.0f;
    theta = 0.30f;
    Motor_Ekf_Seed(&obs, &cfg, theta, omega_e, 0.0f, 0.0f);
    for (i = 0U; i < 500U; ++i) {
        pmsm_voltage(theta, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        assert(Motor_Ekf_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out));
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Ekf_GetAngle(&obs) - theta));
    assert(err < 0.12f);
    assert(Motor_Ekf_GetSpeedE(&obs) < 0.0f);

    Motor_Ekf_Reset(&obs);
    cfg.phase_offset = 0.30f;
    Motor_Ekf_Seed(&obs, &cfg, 0.10f, 100.0f, 0.0f, 0.0f);
    assert(fabsf(wrap_pi(Motor_Ekf_GetAngle(&obs) - 0.40f)) < 1e-5f);
    cfg.phase_offset = 0.0f;

    omega_e = 2.0f * (float)M_PI * 100.0f;
    Motor_Ekf_Seed(&obs, &cfg, 0.2f, omega_e, 0.0f, 0.0f);
    assert(!Motor_Ekf_Step(&obs, &cfg, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, &out));
    assert(!obs.initialized);
    Motor_Ekf_Seed(&obs, &cfg, 0.2f, omega_e, 0.0f, 0.0f);
    assert(!Motor_Ekf_Step(&obs, &cfg, NAN, 0.0f, 0.0f, 0.0f, dt, &out));
    assert(!obs.initialized);
    assert(!out.valid);

    {
        motor_ekf_t a;
        motor_ekf_t b;

        Motor_Ekf_Reset(&a);
        Motor_Ekf_Reset(&b);
        Motor_Ekf_Seed(&a, &cfg, 0.2f, omega_e, 0.0f, 0.0f);
        Motor_Ekf_Seed(&b, &cfg, 2.0f, omega_e, 0.0f, 0.0f);
        pmsm_voltage(0.2f, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        Motor_Ekf_Step(&a, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out);
        assert(fabsf(Motor_Ekf_GetAngle(&b) - 2.0f) < 1e-5f);
        assert(fabsf(b.x[2] - omega_e) < 1e-4f);
    }

    Motor_Ekf_Reset(NULL);
    Motor_Ekf_NormalizeConfig(NULL);
    Motor_Ekf_Seed(NULL, &cfg, 1.0f, 1.0f, 0.0f, 0.0f);
    assert(fabsf(Motor_Ekf_GetAngle(NULL)) < 1e-9f);
    assert(fabsf(Motor_Ekf_GetSpeedE(NULL)) < 1e-9f);
    assert(!Motor_Ekf_Step(NULL, &cfg, 0.0f, 0.0f, 0.0f, 0.0f, dt, &out));

    printf("test_motor_ekf: PASS\n");
    return 0;
}
