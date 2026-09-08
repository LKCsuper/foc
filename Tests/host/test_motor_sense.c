/*
 * @Description: 采样帧换算主机测试（假 ADC，不访问硬件）
 * @Author: lkc
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_sense.h"
#include "motor_param.h"

/* motor_sense.c 仍含 Update 入口，链接时需这些符号；本测只走 ApplyFrame。 */
bool Hal_Motor_SampleCapture(hal_motor_sample_t *out)
{
    (void)out;
    return false;
}
float Hal_Motor_CurrentScale(void)
{
    return 0.0f;
}
float Hal_Motor_PhaseVoltageGain(void)
{
    return 0.0f;
}

int main(void)
{
    MOTOR_PARAM_S m;
    hal_motor_sample_t sample;
    const float i_scale = 0.01f; /* A/count */
    const float v_gain = 10.0f;

    memset(&m, 0, sizeof(m));
    m.currentOffset[0] = 2048.0f;
    m.currentOffset[1] = 2048.0f;
    m.currentOffset[2] = 2048.0f;
    m.voltundrivenOffset[0] = 1.0f;
    m.voltundrivenOffset[1] = 1.0f;
    m.voltundrivenOffset[2] = 1.0f;
    m.v_bus = 0.0f;

    memset(&sample, 0, sizeof(sample));
    sample.current_adc[0] = 2148; /* +100 counts → +1.0 A */
    sample.current_adc[1] = 1948; /* -100 counts → -1.0 A */
    sample.current_adc[2] = 2048; /* 0 A */
    sample.sense_volts[0] = 1.2f; /* SENS1 → va */
    sample.sense_volts[1] = 1.4f; /* SENS2 → vc */
    sample.sense_volts[2] = 1.6f; /* SENS3 → vb */
    sample.v_bus = 24.0f;
    sample.time_us = 1000U;
    sample.current_mask = 0x07U;
    sample.three_shunts = true;

    Motor_Sense_ApplyFrame(&m, &sample, i_scale, v_gain);

    assert(fabsf(m.ia - 1.0f) < 1e-5f);
    /* ACB：adc[2]→ib，adc[1]→ic，与 PWM B/C 对调一致 */
    assert(fabsf(m.ib) < 1e-5f);
    assert(fabsf(m.ic - (-1.0f)) < 1e-5f);
    assert(fabsf(m.va - (1.2f - 1.0f) * v_gain) < 1e-5f);
    assert(fabsf(m.vb - (1.6f - 1.0f) * v_gain) < 1e-5f);
    assert(fabsf(m.vc - (1.4f - 1.0f) * v_gain) < 1e-5f);
    /* 首拍低通：v_bus = 0 + 0.1*(24-0) */
    assert(fabsf(m.v_bus - 2.4f) < 1e-5f);
    assert(m.currentAdc[0] == 2148);

    /* 非法偏移回退默认中点 2045 */
    m.currentOffset[0] = 50.0f;
    sample.current_adc[0] = 2145;
    Motor_Sense_ApplyFrame(&m, &sample, i_scale, v_gain);
    assert(fabsf(m.ia - 1.0f) < 1e-5f);

    /* current_mask 缺硬件 C 相（bit2）时不改写算法 ib */
    m.ib = 9.0f;
    sample.current_mask = 0x03U;
    Motor_Sense_ApplyFrame(&m, &sample, i_scale, v_gain);
    assert(fabsf(m.ib - 9.0f) < 1e-5f);

    puts("test_motor_sense: pass");
    return 0;
}
