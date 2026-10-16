/*
 * @Description: BLDC 六步模式的 ADC-DMA 快速入口
 * @Author: lkc
 */
#include <stddef.h>
#include "common_assert.h"
#include "motor_bldc_isr.h"
#include "motor_protect.h"
#include "motor_sense.h"
#include "motor_timing.h"
#include "hal_motor.h"
#include "hal_motor_bldc.h"

/**
 * @description: 执行一拍 BLDC 采样、保护和 BEMF 换相处理。
 * @detail: 在 ADC-DMA 中断调用，不阻塞。保护检查每一帧；BEMF 只消费 sample_upward 选定的 PWM 半周期。
 *          传给无感步进的时间是 2 倍 ADC 周期，对应一个完整 PWM 周期。
 * @param {motor_bldc_sensorless_t *} sensorless BLDC 无感实例，不可为空
 * @param {MOTOR_PARAM_S *} motor                电机采样与保护状态，不可为空
 * @param {bool} sample_upward                   true=使用 PWM 向上计数侧 BEMF 样本
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcIsr_Step(motor_bldc_sensorless_t *sensorless, MOTOR_PARAM_S *motor, bool sample_upward)
{
    uint32_t runtime_start;
    float dt;
    float phase_voltage[3];

    COMMON_ASSERT(sensorless != NULL && motor != NULL);
    runtime_start = Hal_Motor_TimeUs();
    Motor_Sense_Update(motor);
    dt = Motor_Timing_GetAdcDt(motor);
    Motor_BldcSensorless_Stop(sensorless);
    if (Motor_Protect_Run(motor, dt) || !Hal_Motor_BldcBemfSampleReady(sample_upward)) {
        Motor_Timing_RecordRuntime(runtime_start);
        return;
    }

    /* FOC 运行量为 A-C-B；BLDC BEMF 恢复为原理图 SENS1/2/3 的 A-B-C。 */
    phase_voltage[0] = motor->va;
    phase_voltage[1] = motor->vc;
    phase_voltage[2] = motor->vb;
    (void)Motor_BldcSensorless_Step(sensorless, phase_voltage, motor->v_bus, 2.0f * dt);
    /* BLDC 出口继续保持选中，失败时不允许 FOC SAFE 中点覆盖悬空相。 */
    Motor_Timing_RecordRuntime(runtime_start);
}
