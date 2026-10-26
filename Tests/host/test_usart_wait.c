/* USART 让出到 ISR 主机测试：WFI/DSB 与中断态跳过 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define STATIC static
#define VOID void
enum { osKernelRunning = 2 };
static uint32_t s_ipsr, s_primask, s_basepri;
static int s_kernel = osKernelRunning;
static unsigned s_barriers, s_waits;
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：读取测试模拟的异常编号；0 表示线程上下文。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 s_ipsr
 */
static uint32_t __get_IPSR(void) { return s_ipsr; }
/**
 * @description: 主机测试替身：读取测试模拟的全局中断屏蔽位。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 s_primask
 */
static uint32_t __get_PRIMASK(void) { return s_primask; }
/**
 * @description: 主机测试替身：读取测试模拟的中断优先级屏蔽值。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 s_basepri
 */
static uint32_t __get_BASEPRI(void) { return s_basepri; }
/**
 * @description: 主机测试替身：返回测试指定的调度器状态。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {int} 测试记录值 s_kernel
 */
static int osKernelGetState(void) { return s_kernel; }
/**
 * @description: 主机测试替身：模拟数据同步屏障并记录调用。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void __DSB(void) { s_barriers++; }
/**
 * @description: 主机测试替身：模拟等待中断并记录等待次数，不挂起主机。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void __WFI(void) { s_waits++; }
#include "usart_wait_under_test.inc"

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    Bsp_Usart_YieldToIsr();
    assert(s_waits == 1U && s_barriers == 1U);
    s_ipsr = 1U; Bsp_Usart_YieldToIsr(); s_ipsr = 0U;
    s_primask = 1U; Bsp_Usart_YieldToIsr(); s_primask = 0U;
    s_basepri = 0x50U; Bsp_Usart_YieldToIsr(); s_basepri = 0U;
    s_kernel = 0; Bsp_Usart_YieldToIsr();
    assert(s_waits == 1U && s_barriers == 1U);
    s_kernel = osKernelRunning;
    Bsp_Usart_YieldToIsr();
    assert(s_waits == 2U);
    puts("test_usart_wait: pass");
    return 0;
}
