/* PWM 窗口主机测试：占空比写捕获与窗口容量/步长 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "hal_motor.h"

typedef struct {
    uint32_t CR1;
    uint32_t CNT;
    uint32_t ARR;
} test_timer_t;

static test_timer_t timer;
static uint32_t now_us;
static unsigned duty_writes;
static unsigned time_reads;
#define TIM1 (&timer)
#define __DMB() ((void)0)
#define TIMER_UPDATE_DUTY_M1(duty_a, duty_b, duty_c) \
    do { (void)(duty_a); (void)(duty_b); (void)(duty_c); duty_writes++; timer.CNT++; } while (0)

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 now_us
 */
uint32_t Hal_Motor_TimeUs(void) { time_reads++; return now_us; }
#include "pwm_window_under_test.inc"

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    unsigned index;
    timer.ARR = 5600U;
    timer.CNT = 2500U;
    Hal_Motor_PwmWindowReserve();
    assert(g_hal_motor_pwm_window.enabled == 2U);
    Hal_Motor_PwmWindowPoll(false, false);
    Hal_Motor_PwmSetDuty(1U, 2U, 3U);
    assert(g_hal_motor_pwm_window.count == 0U && duty_writes == 1U);
    Hal_Motor_PwmWindowPoll(true, false);
    Hal_Motor_PwmWindowPoll(true, true);
    now_us = 999999U;
    Hal_Motor_PwmWindowPoll(true, true);
    assert(g_hal_motor_pwm_window.enabled == 2U);
    now_us = 1000000U;
    Hal_Motor_PwmWindowPoll(true, true);
    assert(g_hal_motor_pwm_window.enabled == 1U);
    for (index = 0U; index < HAL_MOTOR_PWM_WINDOW_CAPACITY; index++)
        Hal_Motor_PwmSetDuty(1U, 2U, 3U);
    assert(g_hal_motor_pwm_window.enabled == 0U && g_hal_motor_pwm_window.count == 32U);
    assert(g_hal_motor_pwm_window.frames[0].before_count == 2501U);
    assert(g_hal_motor_pwm_window.frames[0].after_count == 2502U);
    assert(g_hal_motor_pwm_window.frames[0].period == 5600U);
    Hal_Motor_PwmSetDuty(1U, 2U, 3U);
    assert(g_hal_motor_pwm_window.count == 32U && duty_writes == 34U);

    Hal_Motor_PwmWindowReserve();
    now_us = 0xffff0000U;
    Hal_Motor_PwmWindowPoll(true, true);
    now_us += 500000U;
    Hal_Motor_PwmWindowPoll(true, false);
    Hal_Motor_PwmWindowPoll(true, true);
    now_us += 999999U;
    Hal_Motor_PwmWindowPoll(true, true);
    assert(g_hal_motor_pwm_window.enabled == 2U);
    now_us++;
    Hal_Motor_PwmWindowPoll(true, true);
    assert(g_hal_motor_pwm_window.enabled == 1U);
    Hal_Motor_PwmSetDuty(1U, 2U, 3U);
    Hal_Motor_PwmWindowPoll(false, false);
    assert(g_hal_motor_pwm_window.enabled == 0U && g_hal_motor_pwm_window.count == 1U);

    Hal_Motor_PwmWindowReserve();
    Hal_Motor_PwmWindowPoll(true, false);
    Hal_Motor_PwmWindowPoll(false, false);
    assert(g_hal_motor_pwm_window.enabled == 0U && g_hal_motor_pwm_window.count == 0U);
    Hal_Motor_PwmWindowReserve();
    Hal_Motor_PwmWindowEnable(false);
    assert(g_hal_motor_pwm_window.enabled == 0U);
    Hal_Motor_PwmWindowReserveLong();
    assert(Hal_Motor_PwmWindowStride() == 997U);
    Hal_Motor_PwmWindowPoll(true, true);
    now_us += 1000000U;
    Hal_Motor_PwmWindowPoll(true, true);
    time_reads = 0U;
    for (index = 0U; index <= 31U * 997U; index++) {
        now_us++;
        Hal_Motor_PwmSetDuty(1U, 2U, 3U);
        assert(g_hal_motor_pwm_window.count == index / 997U + 1U);
    }
    assert(time_reads == 64U);
    assert(g_hal_motor_pwm_window.enabled == 0U && g_hal_motor_pwm_window.count == 32U);
    assert(g_hal_motor_pwm_window.frames[31].before_us - g_hal_motor_pwm_window.frames[0].before_us == 31U * 997U);
    Hal_Motor_PwmWindowReserveLong();
    Hal_Motor_PwmWindowPoll(true, true);
    now_us += 1000000U;
    Hal_Motor_PwmWindowPoll(true, true);
    Hal_Motor_PwmSetDuty(1U, 2U, 3U);
    Hal_Motor_PwmWindowPoll(false, false);
    assert(g_hal_motor_pwm_window.enabled == 0U && g_hal_motor_pwm_window.count == 1U);
    Hal_Motor_PwmWindowReserve();
    assert(Hal_Motor_PwmWindowStride() == 1U);
    Hal_Motor_PwmWindowPoll(true, true);
    now_us += 1000000U;
    Hal_Motor_PwmWindowPoll(true, true);
    Hal_Motor_PwmSetDuty(1U, 2U, 3U);
    Hal_Motor_PwmSetDuty(1U, 2U, 3U);
    assert(g_hal_motor_pwm_window.count == 2U);
    puts("PWM window tests passed");
    return 0;
}
