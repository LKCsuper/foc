/* 检测 ISR：先采样后喂给检测逻辑，空指针安全与 PWM 降栅标志。 */
#include "test_common_assert.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "motor_detect_isr.h"

static unsigned s_sample_calls;
static unsigned s_feed_calls;
static bool s_pwm_down;

/**
 * @description: 主机测试替身：捕获采样帧并换算写入运行时（ISR 默认入口）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 电机实例
 * @return {void} 无
 */
void Motor_Sense_Update(MOTOR_PARAM_S *motor)
{
    assert(motor != NULL);
    s_sample_calls++;
}

/**
 * @description: 主机测试替身：DMA 完成中断喂入一拍电流/电压，仅零偏 ISR 采集窗口内有效。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {bool} pwm_down pwm_down 参数
 * @return {void} 无
 */
void Motor_Detect_IsrFeed(bool pwm_down)
{
    assert(s_sample_calls == s_feed_calls + 1U);
    s_feed_calls++;
    s_pwm_down = pwm_down;
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    MOTOR_PARAM_S motor;

    memset(&motor, 0, sizeof(motor));
    EXPECT_COMMON_ASSERT(Motor_DetectIsr_Step(NULL, true));
    assert(s_sample_calls == 0U && s_feed_calls == 0U);

    Motor_DetectIsr_Step(&motor, false);
    assert(s_sample_calls == 1U && s_feed_calls == 1U && !s_pwm_down);
    Motor_DetectIsr_Step(&motor, true);
    assert(s_sample_calls == 2U && s_feed_calls == 2U && s_pwm_down);
    puts("test_motor_detect_isr: pass");
    return 0;
}
