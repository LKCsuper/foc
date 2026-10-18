#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define STATIC static
#define VOID void
enum { osKernelRunning = 2 };
static uint32_t s_ipsr, s_primask, s_basepri;
static int s_kernel = osKernelRunning;
static unsigned s_barriers, s_waits;
static uint32_t __get_IPSR(void) { return s_ipsr; }
static uint32_t __get_PRIMASK(void) { return s_primask; }
static uint32_t __get_BASEPRI(void) { return s_basepri; }
static int osKernelGetState(void) { return s_kernel; }
static void __DSB(void) { s_barriers++; }
static void __WFI(void) { s_waits++; }
#include "usart_wait_under_test.inc"

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
