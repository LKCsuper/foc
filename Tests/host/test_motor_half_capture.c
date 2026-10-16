#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor_timing.h"
#include "motor_sensorless.h"

typedef struct { uint32_t NDTR; } fake_dma_t;
typedef struct { uint32_t CR1, CNT, ARR, CCR1, CCR2, CCR3; } fake_timer_t;
static fake_dma_t dma;
static fake_timer_t timer;
#define DMA2_Stream4 (&dma)
#define TIM1 (&timer)
#define TIM_CR1_DIR 16U
static unsigned raw_calls, pwm_calls, fault_mask, pushes;
static unsigned stops;
static bool due;
static motor_pair_frame_t captured;
uint32_t Hal_Motor_TimeUs(void) { return 1000U; }
uint32_t Hal_Motor_PwmDiagnosticSequence(void)
{
    pwm_calls++;
    return 42U + (((fault_mask & 8U) && pwm_calls > 1U) ? 1U : 0U);
}
uint16_t Hal_Motor_CurrentAdc(uint8_t phase)
{
    raw_calls++;
    if (raw_calls == 6U) {
        if (fault_mask & 2U) timer.CR1 ^= TIM_CR1_DIR;
        if (fault_mask & 4U) dma.NDTR--;
    }
    return (uint16_t)(2040U + phase + (((fault_mask & 1U) && raw_calls > 3U) ? 1U : 0U));
}
bool Motor_Timing_PairFrame(const motor_pair_frame_t *frame) { captured = *frame; return due; }
void Motor_Timing_HandoffStop(void) { stops++; }
void Motor_Timing_PairPush(float alpha, float beta)
{
    assert(alpha == 1.0f && beta == 2.0f);
    pushes++;
}
void Motor_Sensorless_GetLightSnapshot(motor_sensorless_light_snapshot_t *out)
{
    out->v_alpha = 1.0f; out->v_beta = 2.0f;
}
#include "half_capture_under_test.inc"

int main(void)
{
    for (fault_mask = 0U; fault_mask < 16U; fault_mask++) {
        raw_calls = pwm_calls = 0U;
        dma.NDTR = 15U;
        timer.CR1 = 161U; timer.CNT = 1000U; timer.ARR = 5600U;
        timer.CCR1 = 2800U; timer.CCR2 = 2700U; timer.CCR3 = 2900U;
        due = true;
        Motor_DmaInt_CaptureHalf(false);
        assert(captured.reconstructed == fault_mask);
        assert(captured.adc[0] == 2040U && captured.adc[2] == 2042U);
        assert(captured.pwm_sequence == 42U && captured.callback_cr1 == 161U);
        assert(captured.callback_count == 1000U && captured.entry_us == 1000U);
        assert(captured.ia == 0.0f && captured.ib == 0.0f && captured.ic == 0.0f);
        assert(timer.CCR1 == 2800U && timer.CCR2 == 2700U && timer.CCR3 == 2900U);
    }
    assert(pushes == 16U);
    due = false;
    Motor_DmaInt_CaptureHalf(false);
    assert(pushes == 16U);
    raw_calls = pwm_calls = 0U;
    Motor_DmaInt_CaptureHalf(true);
    assert(stops == 1U && raw_calls == 0U && pwm_calls == 0U && pushes == 16U);
    puts("ADC half callback capture: pass");
    return 0;
}
