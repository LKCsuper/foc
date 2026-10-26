/* 验证当前移植的相位契约，不写定时器；覆盖上下行及窗口单位。 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <stdio.h>
#include "hal_motor.h"
#define COMMON_ASSERT(x) assert(x)
#define TIM_CR1_DIR 16U
/* 假定时器：仅提供被测函数读取的 CR1/CNT/ARR，验证读取不改变寄存器。 */
static struct { uint32_t CR1, CNT, ARR; } timer;
#define TIM1 (&timer)
#include "pwm_phase_under_test.inc"
/**
 * @description: 覆盖向下/向上计数、counts_per_us 换算及寄存器只读契约。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    hal_motor_pwm_phase_t phase;
    timer.CR1 = TIM_CR1_DIR | 1U;
    timer.CNT = 2000U;
    timer.ARR = 5600U;
    Hal_Motor_PwmPhaseCapture(&phase);
    assert(phase.counting_down && phase.count == 2000U && phase.period == 5600U);
    assert(phase.counts_per_us * 8U == 1344U);
    assert(phase.counts_per_us * 22U == 3696U);
    assert(timer.CR1 == (TIM_CR1_DIR | 1U) && timer.CNT == 2000U && timer.ARR == 5600U);
    timer.CR1 = 1U;
    timer.CNT = 0U;
    Hal_Motor_PwmPhaseCapture(&phase);
    assert(!phase.counting_down && phase.count == 0U);
    puts("PWM phase HAL tests passed");
    return 0;
}
