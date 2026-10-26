/* BLDC ISR：采样→门控→步进的调用顺序、故障停机与相位换序。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bldc_isr.h"

static bool s_fault;
static bool s_sample_ready;
static unsigned s_sense_calls;
static unsigned s_stop_calls;
static unsigned s_step_calls;
static unsigned s_record_calls;
static bool s_last_sample_upward;
static float s_phase[3];
static float s_bus;
static float s_dt;

/**
 * @description: 主机测试替身：读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 100U
 */
uint32_t Hal_Motor_TimeUs(void) { return 100U; }
/**
 * @description: 主机测试替身：捕获采样帧并换算写入运行时（ISR 默认入口）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Sense_Update(MOTOR_PARAM_S *motor)
{
    (void)motor;
    s_sense_calls++;
}
/**
 * @description: 主机测试替身：计算 ADC/保护用采样周期。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @return {float} 测试记录值 0.001f
 */
float Motor_Timing_GetAdcDt(MOTOR_PARAM_S *motor)
{
    (void)motor;
    return 0.001f;
}
/**
 * @description: 主机测试替身：更新本拍 ADC-DMA 中断耗时统计（平均与最大值）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} start_time start_time 参数
 * @return {void} 无
 */
void Motor_Timing_RecordRuntime(uint32_t start_time)
{
    assert(start_time == 100U);
    s_record_calls++;
}
/**
 * @description: 主机测试替身：停止无感六步状态机。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_bldc_sensorless_t *} control 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_BldcSensorless_Stop(motor_bldc_sensorless_t *control)
{
    (void)control;
    s_stop_calls++;
}
/**
 * @description: 主机测试替身：默认实例完整保护步进，仅首次故障同拍采集无感/速度辅助快照。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @param {float} dt 本次推进时间间隔 (s)
 * @return {bool} 测试记录值 s_fault
 */
bool Motor_Protect_Run(const MOTOR_PARAM_S *motor, float dt)
{
    (void)motor;
    assert(dt == 0.001f);
    return s_fault;
}
/**
 * @description: 主机测试替身：判断当前 ADC 帧是否位于指定的 TIM1 计数半周期。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {bool} upward upward 参数
 * @return {bool} 测试记录值 s_sample_ready
 */
bool Hal_Motor_BldcBemfSampleReady(bool upward)
{
    s_last_sample_upward = upward;
    return s_sample_ready;
}
/**
 * @description: 主机测试替身：推进一个无感六步控制周期。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_bldc_sensorless_t *} control 兼容参数，本实现不使用
 * @param {const float [3]} phase 相位信息或相编号
 * @param {float} bus bus 参数
 * @param {float} dt 本次推进时间间隔 (s)
 * @return {bool} 本实现固定返回 true
 */
bool Motor_BldcSensorless_Step(motor_bldc_sensorless_t *control,
                               const float phase[3], float bus, float dt)
{
    (void)control;
    s_step_calls++;
    memcpy(s_phase, phase, sizeof(s_phase));
    s_bus = bus;
    s_dt = dt;
    return true;
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    MOTOR_PARAM_S motor;
    motor_bldc_sensorless_t sensorless;

    memset(&motor, 0, sizeof(motor));
    memset(&sensorless, 0, sizeof(sensorless));
    motor.va = 1.0f;
    motor.vb = 2.0f;
    motor.vc = 3.0f;
    motor.v_bus = 24.0f;

    s_fault = true;
    Motor_BldcIsr_Step(&sensorless, &motor, true);
    assert(s_sense_calls == 1U && s_stop_calls == 1U && s_step_calls == 0U);
    assert(s_record_calls == 1U);

    s_fault = false;
    s_sample_ready = false;
    Motor_BldcIsr_Step(&sensorless, &motor, false);
    assert(!s_last_sample_upward && s_step_calls == 0U && s_record_calls == 2U);

    s_sample_ready = true;
    Motor_BldcIsr_Step(&sensorless, &motor, true);
    assert(s_last_sample_upward && s_step_calls == 1U && s_record_calls == 3U);
    assert(s_phase[0] == 1.0f && s_phase[1] == 3.0f && s_phase[2] == 2.0f);
    assert(s_bus == 24.0f && s_dt == 0.002f);

    puts("test_motor_bldc_isr: pass");
    return 0;
}
