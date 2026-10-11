/* 只链接编码器核心；没有HAL、MT6701、全局motor或RTOS替身。 */
#include "motor_encoder_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    motor_encoder_runtime_t a, b, before;
    motor_encoder_config_t config = {30.0f, 55000.0f, 2.0f, 0.002f};
    assert(Motor_Encoder_InitState(&a, 16384U, false, &config));
    config.max_rpm = 60.0f;
    config.pll_bandwidth_rad_s = 100.0f;
    assert(Motor_Encoder_InitState(&b, 16384U, true, &config));
    Motor_Encoder_UpdateState(&a, 16380U, .001f);
    Motor_Encoder_UpdateState(&b, 16380U, .001f);
    Motor_Encoder_UpdateState(&a, 100U, .001f);
    Motor_Encoder_UpdateState(&b, 100U, .001f);
    assert(a.last_count == 100U && !a.signal_fault && a.accumulated_count == 104U);
    assert(b.last_count == 16380U && b.signal_fault && b.invalid_count == 1U);
    Motor_Encoder_MarkZeroState(&a, 100U, 7.0f);
    {
        float angle = Motor_Encoder_GetElecAngleState(&a, 100U, 7.0f);
        assert(fminf(angle, 6.283185307f - angle) < 1e-5f);
    }
    Motor_Encoder_PredictState(&a, .003f);
    assert(a.sample_stale && !a.speed_valid && a.zero_valid);
    Motor_Encoder_SeedState(&a, 101U);
    assert(a.zero_valid && a.accumulated_count == 104U && !a.speed_valid);
    memcpy(&before, &a, sizeof(a));
    config.pll_bandwidth_rad_s = NAN;
    assert(!Motor_Encoder_InitState(&a, 16384U, false, &config));
    assert(memcmp(&a, &before, sizeof(a)) == 0);
    config.pll_bandwidth_rad_s = 30.0f;
    assert(!Motor_Encoder_InitState(&a, 65537U, false, &config));
    assert(!Motor_Encoder_InitState(&a, 0U, false, &config));
    Motor_Encoder_ApplyCalibrationState(&a, -1.0f, false, true);
    assert(a.elec_offset_rad > 5.0f && a.elec_offset_rad < 6.0f);
    puts("encoder standalone core: pass");
    return 0;
}
