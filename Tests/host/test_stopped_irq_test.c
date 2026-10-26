/* 停机 IRQ 测试 HAL 主机测试：寄存器保存/恢复与前置校验 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef struct { uint32_t ISER[2]; } nvic_t;
typedef struct { uint32_t IDR, MODER, AFR[2]; } gpio_t;
typedef struct { uint32_t CR2, BDTR, CCMR1, CCMR2, CCER; } tim_t;
static nvic_t nvic;
static gpio_t gpio_a, gpio_b;
static tim_t tim;
static unsigned primask;
static bool stuck_high;
#define NVIC (&nvic)
#define GPIOA (&gpio_a)
#define GPIOB (&gpio_b)
#define TIM1 (&tim)
#define TIM_BDTR_MOE 0x8000U
#define TIM_BDTR_OSSI 0x400U
#define TIM_BDTR_AOE 0x4000U
#define DMA2_Stream4_IRQn 60U
#define IRQ_MASK (1U << 28U)
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：读取测试模拟的全局中断屏蔽位。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 primask
 */
static uint32_t __get_PRIMASK(void) { return primask; }
/**
 * @description: 主机测试替身：模拟屏蔽中断并更新测试记录。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void __disable_irq(void) { primask = 1U; }
/**
 * @description: 主机测试替身：模拟恢复中断并更新测试记录。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void __enable_irq(void) { primask = 0U; }
/**
 * @description: 主机测试替身：模拟数据同步屏障并记录调用。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void __DSB(void) { gpio_a.IDR = stuck_high ? 0x100U : 0U; }
/**
 * @description: 主机测试替身：模拟指令同步屏障并记录调用。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void __ISB(void) {}
/**
 * @description: 主机测试替身：模拟禁用指定 IRQ 并更新假 NVIC 寄存器。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {unsigned} irq irq 参数
 * @return {void} 无
 */
static void NVIC_DisableIRQ(unsigned irq) { assert(irq == 60U); nvic.ISER[1] &= ~IRQ_MASK; }
/**
 * @description: 主机测试替身：模拟启用指定 IRQ 并更新假 NVIC 寄存器。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {unsigned} irq irq 参数
 * @return {void} 无
 */
static void NVIC_EnableIRQ(unsigned irq) { assert(irq == 60U); nvic.ISER[1] |= IRQ_MASK; }
#include "stopped_irq_test_under_test.inc"
/**
 * @description: 复位当前测试场景及记录值。
 * @return {void} 无
 */
static void reset(void)
{
    tim = (tim_t){0x7f01U, 0xac3cU, 0x4848U, 0x6848U, 0x1111U};
    nvic.ISER[1] = IRQ_MASK | 8U;
    gpio_a.IDR = gpio_b.IDR = primask = 0U;
    gpio_a.MODER = 0x2a0000U;
    gpio_b.MODER = 0xa8000000U;
    gpio_a.AFR[1] = 0x111U;
    gpio_b.AFR[1] = 0x11100000U;
    stuck_high = s_irq_test_active = false;
}
/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    unsigned masked;
    for (masked = 0U; masked < 2U; masked++) {
        reset();
        assert(Hal_Motor_StoppedIrqTestBegin(masked != 0U));
        assert(Hal_Motor_StoppedIrqTestCheck(masked != 0U));
        assert(!Hal_Motor_StoppedIrqTestBegin(true));
        assert((tim.CR2 & 0x3f00U) == 0U && !(tim.BDTR & TIM_BDTR_MOE));
        assert(nvic.ISER[1] == (masked ? 8U : IRQ_MASK | 8U));
        assert(Hal_Motor_StoppedIrqTestEnd(true));
        assert(tim.CR2 == 0x7f01U && tim.BDTR == 0xac3cU);
        assert(tim.CCMR1 == 0x4848U && tim.CCMR2 == 0x6848U && tim.CCER == 0x1111U);
        assert(nvic.ISER[1] == (IRQ_MASK | 8U) && primask == 0U);
        assert(!Hal_Motor_StoppedIrqTestEnd(true));
    }
    reset(); primask = 1U; assert(!Hal_Motor_StoppedIrqTestBegin(true));
    reset(); tim.BDTR &= ~TIM_BDTR_MOE; assert(!Hal_Motor_StoppedIrqTestBegin(true));
    reset(); tim.BDTR |= TIM_BDTR_AOE; assert(!Hal_Motor_StoppedIrqTestBegin(true));
    reset(); tim.BDTR &= ~TIM_BDTR_OSSI; assert(!Hal_Motor_StoppedIrqTestBegin(true));
    reset(); tim.CCER |= 4U; assert(!Hal_Motor_StoppedIrqTestBegin(true));
    reset(); tim.CCMR1 = 0x6868U; assert(!Hal_Motor_StoppedIrqTestBegin(true));
    reset(); gpio_a.MODER = 0U; assert(!Hal_Motor_StoppedIrqTestBegin(true));
    reset(); gpio_b.AFR[1] = 0U; assert(!Hal_Motor_StoppedIrqTestBegin(true));
    reset(); nvic.ISER[1] = 8U; assert(!Hal_Motor_StoppedIrqTestBegin(true));
    reset(); stuck_high = true; assert(!Hal_Motor_StoppedIrqTestBegin(true));
    assert(!(tim.BDTR & TIM_BDTR_MOE) && (nvic.ISER[1] & IRQ_MASK));
    reset(); assert(Hal_Motor_StoppedIrqTestBegin(true));
    assert(!Hal_Motor_StoppedIrqTestEnd(false));
    assert(!(tim.BDTR & TIM_BDTR_MOE) && (nvic.ISER[1] & IRQ_MASK));
    reset(); assert(Hal_Motor_StoppedIrqTestBegin(true));
    gpio_b.IDR = 0x2000U;
    assert(!Hal_Motor_StoppedIrqTestCheck(true));
    assert(!Hal_Motor_StoppedIrqTestEnd(true));
    assert(!(tim.BDTR & TIM_BDTR_MOE) && (nvic.ISER[1] & IRQ_MASK));
    puts("test_stopped_irq_test: pass");
    return 0;
}
