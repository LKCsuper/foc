/* 相位 IRQ 包装主机测试：DMA 中断入口/退出与冻结 */
/* Includes ------------------------------------------------------------------*/
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
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：查询唤醒探测或任务执行计时窗口是否需要累计 DMA 耗时。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 wake
 */
static bool Motor_Timing_WakeProbeDmaDue(void) { return wake; }
/**
 * @description: 主机测试替身：判断本次 IRQ 是否允许记一次耗时。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {unsigned} kind kind 参数
 * @return {bool} 测试记录值 timing
 */
static bool Motor_Timing_IrqDue(unsigned kind) { assert(kind == 0U); return timing; }
/**
 * @description: 主机测试替身：读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 clock_us++
 */
static uint32_t Hal_Motor_TimeUs(void) { return clock_us++; }
/**
 * @description: 主机测试替身：读取假 NVIC 的中断挂起状态。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {int} irq irq 参数
 * @return {uint32_t} 测试记录值 1U
 */
static uint32_t NVIC_GetPendingIRQ(int irq) { assert(irq == 60); return 1U; }
/**
 * @description: 主机测试替身：记录一次 IRQ 入口时间并打开本拍 IRQ 现场关联窗口。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} time time 参数
 * @return {void} 无
 */
static void Motor_Timing_PhaseIrqBegin(uint32_t time) { begins++; begin_us = time; }
/**
 * @description: 主机测试替身：关闭 IRQ 窗口，把出口时间与 DMA/ADC 状态写入对应事件。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} time time 参数
 * @param {uint32_t} dma_status dma_status 参数
 * @param {uint32_t} adc_status adc_status 参数
 * @param {uint32_t} remaining remaining 参数
 * @param {uint32_t} pending pending 参数
 * @return {void} 无
 */
static void Motor_Timing_PhaseIrqEnd(uint32_t time, uint32_t dma_status, uint32_t adc_status,
                                    uint32_t remaining, uint32_t pending)
{
    ends++;
    exit_us = time;
    assert(dma_status == 0x20U && adc_status == 0x202020U && remaining == 7U && pending == 1U);
}
/**
 * @description: 主机测试替身：DMA2 Stream4 中断处理（ADC 采样完成触发 FOC 计算）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void Bsp_Dma2Stream4_Handler(void)
{
    handlers++;
    dma.HISR = 0x20U;
    adc.CSR = 0x202020U;
    stream.NDTR = 7U;
    if (freeze) g_motor_phase_active = 0U;
}
/**
 * @description: 主机测试替身：累计截止 tick 之后、任务唤醒之前的一次 DMA 耗时。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} begin begin 参数
 * @param {uint32_t} end end 参数
 * @return {void} 无
 */
static void Motor_Timing_WakeProbeDmaNote(uint32_t begin, uint32_t end)
{
    assert(end >= begin);
    probes++;
}
/**
 * @description: 主机测试替身：记录一次已放行的 IRQ 函数体耗时。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {unsigned} kind kind 参数
 * @param {uint32_t} begin begin 参数
 * @param {uint32_t} end end 参数
 * @param {unsigned} pending pending 参数
 * @return {void} 无
 */
static void Motor_Timing_IrqNote(unsigned kind, uint32_t begin, uint32_t end, unsigned pending)
{
    assert(kind == 0U && end >= begin && pending == 0U);
    timings++;
}
#include "phase_irq_under_test.inc"

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
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
