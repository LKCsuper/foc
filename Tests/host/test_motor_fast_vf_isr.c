/* 快速 V/F ISR：门控、采样节奏与控制步调用及运行时记录。 */
#include "test_common_assert.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "motor_fast_vf.h"

static bool s_should_run;
static uint32_t s_time_us;
static float s_control_hz;
static unsigned s_sample_calls;
static unsigned s_step_calls;
static unsigned s_runtime_calls;
static uint32_t s_runtime_start;

/**
 * @description: 主机测试替身：读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 s_time_us
 */
uint32_t Hal_Motor_TimeUs(void) { return s_time_us; }
/**
 * @description: 主机测试替身：查询本拍是否到达 FOC 控制分频点。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_should_run
 */
bool Motor_FocIsr_ShouldRun(void) { return s_should_run; }
/**
 * @description: 主机测试替身：捕获采样帧并换算写入运行时（ISR 默认入口）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 电机实例
 * @return {void} 无
 */
void Motor_Sense_Update(MOTOR_PARAM_S *motor)
{
    s_sample_calls++;
    motor->v_bus = s_sample_calls == 1U ? 0.5f : 24.0f;
}
/**
 * @description: 主机测试替身：返回 FOC 控制环实际运行频率。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_control_hz
 */
float Motor_DmaInt_GetControlFrequencyHz(void) { return s_control_hz; }
/**
 * @description: 主机测试替身：推进一次简化 V/F 诊断控制拍并提交电压。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @param {float} control_hz control_hz 参数
 * @return {void} 无
 */
void Motor_FastVf_Step(MOTOR_PARAM_S *motor, float control_hz)
{
    (void)motor;
    assert(control_hz == s_control_hz);
    s_step_calls++;
}
/**
 * @description: 主机测试替身：更新本拍 ADC-DMA 中断耗时统计（平均与最大值）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} start_time_us start_time_us，单位 us
 * @return {void} 无
 */
void Motor_Timing_RecordRuntime(uint32_t start_time_us)
{
    s_runtime_calls++;
    s_runtime_start = start_time_us;
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    MOTOR_PARAM_S motor;

    memset(&motor, 0, sizeof(motor));
    EXPECT_COMMON_ASSERT(Motor_FastVf_RunIsr(NULL));
    assert(s_runtime_calls == 0U);

    s_time_us = 77U;
    s_should_run = false;
    Motor_FastVf_RunIsr(&motor);
    assert(s_sample_calls == 0U && s_step_calls == 0U);
    assert(s_runtime_calls == 1U && s_runtime_start == 77U);

    s_should_run = true;
    s_control_hz = 15000.0f;
    Motor_FastVf_RunIsr(&motor);
    assert(s_sample_calls == 1U && s_step_calls == 0U);
    assert(s_runtime_calls == 2U);

    Motor_FastVf_RunIsr(&motor);
    assert(s_sample_calls == 2U && s_step_calls == 1U);
    assert(s_runtime_calls == 3U);
    puts("test_motor_fast_vf_isr: pass");
    return 0;
}
