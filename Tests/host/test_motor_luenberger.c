#include "test_common_assert.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_luenberger.h"

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

static void pmsm_voltage(float theta, float omega_e, float iq, const motor_luenberger_config_t *cfg, float *v_alpha,
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

static void default_config(motor_luenberger_config_t *cfg)
{
    cfg->enabled = true;
    cfg->resistance = 0.4f;
    cfg->inductance = 0.0008f;
    cfg->flux_linkage = 0.02573f;
    cfg->current_gain = 0.0f;
    cfg->bemf_gain = 0.0f;
    cfg->phase_offset = 0.0f;
}

int main(void)
{
    motor_luenberger_t obs;
    motor_luenberger_config_t cfg;
    motor_luenberger_output_t out;
    float v_alpha;
    float v_beta;
    float i_alpha;
    float i_beta;
    float dt;
    float theta;
    float omega_e;
    float err;
    unsigned i;

    Motor_Luenberger_Reset(&obs);
    assert(!obs.initialized);
    assert(fabsf(Motor_Luenberger_GetAngle(&obs)) < 1e-9f);
    assert(fabsf(Motor_Luenberger_GetMagnitude(&obs)) < 1e-9f);
    assert(fabsf(Motor_Luenberger_GetSpeedE(&obs)) < 1e-9f);

    default_config(&cfg);
    cfg.enabled = false;
    dt = 1.0f / 15000.0f;
    assert(!Motor_Luenberger_Step(&obs, &cfg, 1.0f, 0.0f, 0.0f, 0.0f, dt, &out));
    assert(!out.valid);
    assert(!obs.initialized);

    cfg.resistance = -1.0f;
    cfg.inductance = 0.0f;
    cfg.flux_linkage = 0.0f;
    cfg.current_gain = -2.0f;
    cfg.bemf_gain = -3.0f;
    cfg.phase_offset = NAN;
    Motor_Luenberger_NormalizeConfig(&cfg);
    assert(fabsf(cfg.resistance) < 1e-9f);
    assert(fabsf(cfg.inductance - MOTOR_LUENBERGER_MIN_INDUCTANCE) < 1e-12f);
    assert(fabsf(cfg.flux_linkage - MOTOR_LUENBERGER_MIN_LINKAGE) < 1e-12f);
    assert(fabsf(cfg.current_gain) < 1e-9f);
    assert(fabsf(cfg.bemf_gain) < 1e-9f);
    assert(fabsf(cfg.phase_offset) < 1e-9f);

    default_config(&cfg);
    Motor_Luenberger_NormalizeConfig(&cfg);
    omega_e = 2.0f * (float)M_PI * 100.0f;
    Motor_Luenberger_Reset(&obs);
    Motor_Luenberger_Seed(&obs, &cfg, 0.40f, omega_e, 0.0f, 0.0f);
    assert(obs.initialized);
    assert(fabsf(Motor_Luenberger_GetAngle(&obs) - 0.40f) < 1e-5f);
    assert(fabsf(Motor_Luenberger_GetSpeedE(&obs) - omega_e) < 1e-4f);
    assert(fabsf(obs.e_alpha + omega_e * cfg.flux_linkage * sinf(0.40f)) < fabsf(omega_e * cfg.flux_linkage) * 0.000025f + 1e-6f);
    assert(fabsf(obs.e_beta - omega_e * cfg.flux_linkage * cosf(0.40f)) < fabsf(omega_e * cfg.flux_linkage) * 0.000025f + 1e-6f);

    /* 零电流、额定电频率：理想反电势应让观测角跟着真角走。 */
    theta = 0.40f;
    for (i = 0U; i < 400U; ++i) {
        pmsm_voltage(theta, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        assert(Motor_Luenberger_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out));
        assert(out.valid);
        assert(out.bemf_ok);
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Luenberger_GetAngle(&obs) - theta));
    assert(err < 0.10f);
    assert(fabsf(Motor_Luenberger_GetSpeedE(&obs) - omega_e) / omega_e < 0.15f);

    /* 带 Iq 时电流模型仍应对准转子。 */
    Motor_Luenberger_Reset(&obs);
    theta = 0.20f;
    Motor_Luenberger_Seed(&obs, &cfg, theta, omega_e, -2.0f * sinf(theta), 2.0f * cosf(theta));
    for (i = 0U; i < 400U; ++i) {
        pmsm_voltage(theta, omega_e, 2.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        assert(Motor_Luenberger_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out));
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Luenberger_GetAngle(&obs) - theta));
    assert(err < 0.12f);
    assert(out.bemf_ok);

    /* 初值偏差后，电流误差应把 ê 拉向真反电势。 */
    Motor_Luenberger_Reset(&obs);
    theta = 1.00f;
    Motor_Luenberger_Seed(&obs, &cfg, 0.50f, omega_e, 0.0f, 0.0f);
    for (i = 0U; i < 1200U; ++i) {
        pmsm_voltage(theta, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        Motor_Luenberger_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out);
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Luenberger_GetAngle(&obs) - theta));
    assert(err < 0.15f);

    /* 反转：θ = atan2(eα, −eβ) 才能跟上负的 ωe。 */
    Motor_Luenberger_Reset(&obs);
    omega_e = -2.0f * (float)M_PI * 80.0f;
    theta = 0.30f;
    Motor_Luenberger_Seed(&obs, &cfg, theta, omega_e, 0.0f, 0.0f);
    for (i = 0U; i < 500U; ++i) {
        pmsm_voltage(theta, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        assert(Motor_Luenberger_Step(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out));
        theta = wrap_pi(theta + omega_e * dt);
    }
    err = fabsf(wrap_pi(Motor_Luenberger_GetAngle(&obs) - theta));
    assert(err < 0.12f);
    assert(Motor_Luenberger_GetSpeedE(&obs) < 0.0f);

    /* 相位补偿加在 atan2 之后。 */
    Motor_Luenberger_Reset(&obs);
    cfg.phase_offset = 0.30f;
    Motor_Luenberger_Seed(&obs, &cfg, 0.10f, 100.0f, 0.0f, 0.0f);
    assert(fabsf(wrap_pi(Motor_Luenberger_GetAngle(&obs) - 0.40f)) < 1e-5f);
    cfg.phase_offset = 0.0f;

    /* 非法 dt / NaN 电压必须复位。 */
    omega_e = 2.0f * (float)M_PI * 100.0f;
    Motor_Luenberger_Seed(&obs, &cfg, 0.2f, omega_e, 0.0f, 0.0f);
    assert(!Motor_Luenberger_Step(&obs, &cfg, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, &out));
    assert(!obs.initialized);
    Motor_Luenberger_Seed(&obs, &cfg, 0.2f, omega_e, 0.0f, 0.0f);
    assert(!Motor_Luenberger_Step(&obs, &cfg, NAN, 0.0f, 0.0f, 0.0f, dt, &out));
    assert(!obs.initialized);
    assert(!out.valid);

    /* 双实例互不影响 */
    {
        motor_luenberger_t a;
        motor_luenberger_t b;

        Motor_Luenberger_Reset(&a);
        Motor_Luenberger_Reset(&b);
        Motor_Luenberger_Seed(&a, &cfg, 0.2f, omega_e, 0.0f, 0.0f);
        Motor_Luenberger_Seed(&b, &cfg, 2.0f, omega_e, 0.0f, 0.0f);
        pmsm_voltage(0.2f, omega_e, 0.0f, &cfg, &v_alpha, &v_beta, &i_alpha, &i_beta);
        Motor_Luenberger_Step(&a, &cfg, v_alpha, v_beta, i_alpha, i_beta, dt, &out);
        assert(fabsf(Motor_Luenberger_GetAngle(&b) - 2.0f) < 1e-5f);
        assert(fabsf(b.e_beta - omega_e * cfg.flux_linkage * cosf(2.0f)) < fabsf(omega_e * cfg.flux_linkage) * 0.000025f + 1e-6f);
    }

    Motor_Luenberger_Reset(NULL);
    EXPECT_COMMON_ASSERT(Motor_Luenberger_NormalizeConfig(NULL));
    EXPECT_COMMON_ASSERT(Motor_Luenberger_Seed(NULL, &cfg, 1.0f, 1.0f, 0.0f, 0.0f));
    assert(fabsf(Motor_Luenberger_GetAngle(NULL)) < 1e-9f);
    assert(fabsf(Motor_Luenberger_GetMagnitude(NULL)) < 1e-9f);
    assert(fabsf(Motor_Luenberger_GetSpeedE(NULL)) < 1e-9f);
    assert(!Motor_Luenberger_Step(NULL, &cfg, 0.0f, 0.0f, 0.0f, 0.0f, dt, &out));

    printf("test_motor_luenberger: PASS\n");
    return 0;
}
