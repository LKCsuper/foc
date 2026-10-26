/* ADC 半周期捕获：重建标志、PWM 序列与故障掩码组合，以及停桥路径。 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor_timing.h"
#include "motor_sensorless.h"

typedef struct { uint32_t NDTR; } fake_dma_t;
typedef struct { uint32_t CR1, CNT, ARR, CCR1, CCR2, CCR3; } fake_timer_t;
static fake_dma_t dma;
static fake_timer_t timer;
#define DMA2_Stream4 (&dma)
#define TIM1 (&timer)
#define TIM_CR1_DIR 16U
static unsigned raw_calls, pwm_calls, fault_mask, pushes;
static unsigned stops;
static bool due;
static motor_pair_frame_t captured;
/**
 * @description: 主机测试替身：读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 1000U
 */
uint32_t Hal_Motor_TimeUs(void) { return 1000U; }
/**
 * @description: 主机测试替身： 读取停机诊断软件提交计数（enable 时清零，仅停机诊断使用）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 本次计算或查询的结果
 */
uint32_t Hal_Motor_PwmDiagnosticSequence(void)
{
    pwm_calls++;
    return 42U + (((fault_mask & 8U) && pwm_calls > 1U) ? 1U : 0U);
}
/**
 * @description: 主机测试替身：读取相电流 ADC 原始值（含分流极性处理）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint8_t} phase 相位信息或相编号
 * @return {uint16_t} ADC 计数
 */
uint16_t Hal_Motor_CurrentAdc(uint8_t phase)
{
    raw_calls++;
    if (raw_calls == 6U) {
        if (fault_mask & 2U) timer.CR1 ^= TIM_CR1_DIR;
        if (fault_mask & 4U) dma.NDTR--;
    }
    return (uint16_t)(2040U + phase + (((fault_mask & 1U) && raw_calls > 3U) ? 1U : 0U));
}
/**
 * @description: 主机测试替身：写入重构后的电流帧，并判断本拍是否凑成一对。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const motor_pair_frame_t *} frame frame 参数
 * @return {bool} 测试记录值 due
 */
bool Motor_Timing_PairFrame(const motor_pair_frame_t *frame) { captured = *frame; return due; }
/**
 * @description: 主机测试替身：停止交接窗口继续写入。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Timing_HandoffStop(void) { stops++; }
/**
 * @description: 主机测试替身：写入一对电流帧及本拍 α/β 电压。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} alpha 输出静止坐标 alpha 轴电压 (V)
 * @param {float} beta 输出静止坐标 beta 轴电压 (V)
 * @return {void} 无
 */
void Motor_Timing_PairPush(float alpha, float beta)
{
    assert(alpha == 1.0f && beta == 2.0f);
    pushes++;
}
/**
 * @description: 主机测试替身：拷贝默认实例已计算字段的轻量快照。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_sensorless_light_snapshot_t *} out 输出缓冲或结果
 * @return {void} 无
 */
void Motor_Sensorless_GetLightSnapshot(motor_sensorless_light_snapshot_t *out)
{
    out->v_alpha = 1.0f; out->v_beta = 2.0f;
}
#include "half_capture_under_test.inc"

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    for (fault_mask = 0U; fault_mask < 16U; fault_mask++) {
        raw_calls = pwm_calls = 0U;
        dma.NDTR = 15U;
        timer.CR1 = 161U; timer.CNT = 1000U; timer.ARR = 5600U;
        timer.CCR1 = 2800U; timer.CCR2 = 2700U; timer.CCR3 = 2900U;
        due = true;
        Motor_DmaInt_CaptureHalf(false);
        assert(captured.reconstructed == fault_mask);
        assert(captured.adc[0] == 2040U && captured.adc[2] == 2042U);
        assert(captured.pwm_sequence == 42U && captured.callback_cr1 == 161U);
        assert(captured.callback_count == 1000U && captured.entry_us == 1000U);
        assert(captured.ia == 0.0f && captured.ib == 0.0f && captured.ic == 0.0f);
        assert(timer.CCR1 == 2800U && timer.CCR2 == 2700U && timer.CCR3 == 2900U);
    }
    assert(pushes == 16U);
    due = false;
    Motor_DmaInt_CaptureHalf(false);
    assert(pushes == 16U);
    raw_calls = pwm_calls = 0U;
    Motor_DmaInt_CaptureHalf(true);
    assert(stops == 1U && raw_calls == 0U && pwm_calls == 0U && pushes == 16U);
    puts("ADC half callback capture: pass");
    return 0;
}
