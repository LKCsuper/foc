#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define MOTOR_CFG_DIAG 1
#define SCB_ICSR_PENDSTSET_Msk (1U << 26)
#define SCB_ICSR_PENDSVSET_Msk (1U << 28)
#define DMA2_Stream4_IRQn 60
static struct { uint32_t HISR; } dma;
static struct { uint32_t CSR; } adc;
static struct { uint32_t NDTR; } stream;
static struct { uint32_t ICSR; } scb;
#define DMA2 (&dma)
#define ADC (&adc)
#define DMA2_Stream4 (&stream)
#define SCB (&scb)
volatile uint32_t g_motor_phase_active;
static unsigned begins, ends, handlers, probes, timings;
static uint32_t clock_us, begin_us, exit_us;
static bool wake, timing, freeze;
static bool Motor_Timing_WakeProbeDmaDue(void) { return wake; }
static bool Motor_Timing_IrqDue(unsigned kind) { assert(kind == 0U); return timing; }
static uint32_t Hal_Motor_TimeUs(void) { return clock_us++; }
static uint32_t NVIC_GetPendingIRQ(int irq) { assert(irq == 60); return 1U; }
static void Motor_Timing_PhaseIrqBegin(uint32_t time) { begins++; begin_us = time; }
static void Motor_Timing_PhaseIrqEnd(uint32_t time, uint32_t dma_status, uint32_t adc_status,
                                    uint32_t remaining, uint32_t pending)
{
    ends++;
    exit_us = time;
    assert(dma_status == 0x20U && adc_status == 0x202020U && remaining == 7U && pending == 1U);
}
static void Bsp_Dma2Stream4_Handler(void)
{
    handlers++;
    dma.HISR = 0x20U;
    adc.CSR = 0x202020U;
    stream.NDTR = 7U;
    if (freeze) g_motor_phase_active = 0U;
}
static void Motor_Timing_WakeProbeDmaNote(uint32_t begin, uint32_t end)
{
    assert(end >= begin);
    probes++;
}
static void Motor_Timing_IrqNote(unsigned kind, uint32_t begin, uint32_t end, unsigned pending)
{
    assert(kind == 0U && end >= begin && pending == 0U);
    timings++;
}
#include "phase_irq_under_test.inc"

int main(void)
{
    DMA2_Stream4_IRQHandler();
    assert(handlers == 1U && begins == 0U && ends == 0U && clock_us == 0U);
    g_motor_phase_active = 1U;
    clock_us = 10U;
    DMA2_Stream4_IRQHandler();
    assert(handlers == 2U && begins == 1U && ends == 1U && begin_us == 10U && exit_us == 11U);
    freeze = true;
    wake = true;
    timing = true;
    DMA2_Stream4_IRQHandler();
    assert(handlers == 3U && begins == 2U && ends == 2U && g_motor_phase_active == 0U);
    assert(probes == 1U && timings == 1U);
    g_motor_phase_active = 3U;
    freeze = false;
    wake = false;
    timing = false;
    DMA2_Stream4_IRQHandler();
    assert(handlers == 4U && begins == 2U && ends == 2U && g_motor_phase_active == 3U);
    puts("Phase IRQ wrapper tests passed");
    return 0;
}
