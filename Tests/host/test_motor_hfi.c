#include "test_common_assert.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_hfi.h"

static void hfi_dq_to_ab(float id, float iq, float angle, float *i_alpha, float *i_beta)
{
    float s = sinf(angle);
    float c = cosf(angle);

    *i_alpha = id * c - iq * s;
    *i_beta = id * s + iq * c;
}

int main(void)
{
    motor_hfi_t hfi;
    motor_hfi_config_t cfg;
    motor_hfi_output_t out;
    float i_alpha;
    float i_beta;
    float true_angle;
    float dt;
    float err0;
    float err1;
    float iq_hf;
    int8_t applied_p;
    unsigned i;

    Motor_Hfi_Reset(&hfi);
    assert(fabsf(hfi.angle) < 1e-6f);
    assert(hfi.polarity == 0);
    assert(!hfi.running);

    cfg.enabled = false;
    cfg.voltage = 2.0f;
    cfg.gain = 1.0f;
    cfg.max_bus_voltage_ratio = 0.20f;
    assert(!Motor_Hfi_Step(&hfi, &cfg, 0.0f, 0.0f, 24.0f, 1.0f / 15000.0f, &out));
    assert(!out.apply);
    assert(fabsf(out.vd_inj) < 1e-6f);

    cfg.enabled = true;
    cfg.voltage = -3.0f;
    cfg.gain = -2.0f;
    cfg.max_bus_voltage_ratio = 2.0f;
    Motor_Hfi_NormalizeConfig(&cfg);
    assert(fabsf(cfg.voltage) < 1e-6f);
    assert(fabsf(cfg.gain) < 1e-6f);
    assert(fabsf(cfg.max_bus_voltage_ratio - MOTOR_HFI_DEFAULT_BUS_RATIO) < 1e-6f);

    cfg.voltage = 2.0f;
    cfg.gain = 8.0f;
    Motor_Hfi_NormalizeConfig(&cfg);
    Motor_Hfi_Reset(&hfi);
    Motor_Hfi_SeedAngle(&hfi, 0.40f);
    assert(fabsf(Motor_Hfi_GetAngle(&hfi) - 0.40f) < 1e-5f);

    /* 首拍只注入 +Vh，不改角 */
    hfi_dq_to_ab(0.0f, 0.0f, hfi.angle, &i_alpha, &i_beta);
    assert(Motor_Hfi_Step(&hfi, &cfg, i_alpha, i_beta, 24.0f, 1.0f / 15000.0f, &out));
    assert(out.apply);
    assert(fabsf(out.vd_inj - 2.0f) < 1e-5f);
    assert(fabsf(out.vq_inj) < 1e-6f);
    assert(fabsf(hfi.angle - 0.40f) < 1e-5f);
    applied_p = hfi.polarity;

    /* 母线比例限制注入幅值 */
    Motor_Hfi_Reset(&hfi);
    cfg.max_bus_voltage_ratio = 0.10f;
    Motor_Hfi_NormalizeConfig(&cfg);
    hfi_dq_to_ab(0.0f, 0.0f, 0.0f, &i_alpha, &i_beta);
    Motor_Hfi_Step(&hfi, &cfg, i_alpha, i_beta, 10.0f, 1.0f / 15000.0f, &out);
    assert(fabsf(fabsf(out.vd_inj) - 1.0f) < 1e-5f);
    cfg.max_bus_voltage_ratio = 0.20f;
    Motor_Hfi_NormalizeConfig(&cfg);

    /* IPMSM 凸极模型：iq_h = C·p·sin(2θ_err)，C<0。正 gain 应把滞后估计角拉向真角。 */
    true_angle = 1.00f;
    dt = 1.0f / 15000.0f;
    Motor_Hfi_Reset(&hfi);
    Motor_Hfi_SeedAngle(&hfi, 0.55f);
    err0 = fabsf(true_angle - hfi.angle);
    applied_p = 0;
    for (i = 0U; i < 400U; ++i) {
        if (applied_p == 0) {
            iq_hf = 0.0f;
        } else {
            iq_hf = -0.02f * (float)applied_p * sinf(2.0f * (true_angle - hfi.angle));
        }
        hfi_dq_to_ab(0.0f, iq_hf, hfi.angle, &i_alpha, &i_beta);
        Motor_Hfi_Step(&hfi, &cfg, i_alpha, i_beta, 24.0f, dt, &out);
        applied_p = hfi.polarity;
        assert(out.apply);
        assert(fabsf(out.vd_inj - (float)applied_p * cfg.voltage) < 1e-5f);
    }
    err1 = fabsf(true_angle - Motor_Hfi_GetAngle(&hfi));
    assert(err1 < err0 * 0.25f);
    assert(err1 < 0.08f);

    /* 双实例互不影响 */
    {
        motor_hfi_t a;
        motor_hfi_t b;

        Motor_Hfi_Reset(&a);
        Motor_Hfi_Reset(&b);
        Motor_Hfi_SeedAngle(&a, 0.2f);
        Motor_Hfi_SeedAngle(&b, 2.0f);
        hfi_dq_to_ab(0.0f, 0.0f, a.angle, &i_alpha, &i_beta);
        Motor_Hfi_Step(&a, &cfg, i_alpha, i_beta, 24.0f, dt, &out);
        assert(fabsf(Motor_Hfi_GetAngle(&b) - 2.0f) < 1e-5f);
        assert(b.polarity == 0);
    }

    /* 空指针安全 */
    Motor_Hfi_Reset(NULL);
    EXPECT_COMMON_ASSERT(Motor_Hfi_NormalizeConfig(NULL));
    EXPECT_COMMON_ASSERT(Motor_Hfi_SeedAngle(NULL, 1.0f));
    assert(fabsf(Motor_Hfi_GetAngle(NULL)) < 1e-6f);
    assert(fabsf(Motor_Hfi_GetSpeedE(NULL)) < 1e-6f);
    assert(!Motor_Hfi_Step(NULL, &cfg, 0.0f, 0.0f, 24.0f, dt, &out));

    printf("test_motor_hfi: PASS\n");
    return 0;
}
