/*
 * @Description: 编码器与霍尔反馈的控制拍更新及传感器故障接入
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "common_assert.h"
#include <stdint.h>
#include "motor_sensor_update.h"
#include "motor_anticogging.h"
#include "motor_encoder.h"
#include "motor_hall.h"
#include "motor_hall_angle.h"
#include "motor_protect.h"
#if defined(__CC_ARM)
/* 位于 ADC 中断控制路径，与其他实时模块一致局部 O2；不启用快速浮点，公式不变。 */
#pragma O2
#endif

/* Private variables ---------------------------------------------------------*/
static uint8_t s_hall_adc_div;
static float s_hall_dt_acc_s;
static float s_reference_dt_acc_s;

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 按当前反馈源更新编码器或霍尔运行状态。
 * @detail: 在 FOC 控制拍调用，不写 PWM。编码器失联时按现有条件请求故障。
 *          霍尔按 MOTOR_HALL_ADC_DIVIDER 分频，dt 累加后交给角度估计。
 * @param {MOTOR_PARAM_S *} motor           电机实例，不可为空
 * @param {motor_feedback_source_t} source  当前请求的反馈源
 * @param {float} dt_s                      本控制拍时间 (s)
 * @return {void} 无
 * @author: lkc
 */
void Motor_SensorUpdate_Step(MOTOR_PARAM_S *motor, motor_feedback_source_t source, float dt_s)
{
    COMMON_ASSERT(motor != NULL);

    if (source == MOTOR_FEEDBACK_SRC_ENCODER || Motor_Encoder_IsCompareEnabled()) {
        if (source == MOTOR_FEEDBACK_SRC_ENCODER) {
            s_reference_dt_acc_s = 0.0f;
            Motor_Encoder_Update(dt_s);
        } else {
            /* 仅对照的原始帧为1kHz，不把参考PLL与状态复制放到每个15kHz控制拍。
             * 传递完整累计时间，正式编码器控制仍逐拍更新。 */
            s_reference_dt_acc_s += dt_s;
            if (s_reference_dt_acc_s >= 0.001f) {
                Motor_Encoder_UpdateReference(s_reference_dt_acc_s);
                s_reference_dt_acc_s = 0.0f;
            }
        }
        /* 标定和抗齿槽学习自行管理输出，期间不由正常运行保护抢占。 */
        if (source == MOTOR_FEEDBACK_SRC_ENCODER && motor->ol_enable && Motor_Encoder_IsSignalFault() && !Motor_Encoder_DetectIsActive() &&
            !Motor_Anticogging_IsLearning() && !Motor_Protect_IsFaultLatched()) {
            Motor_Protect_TriggerFault(FAULT_CODE_ENCODER_SPI, motor);
        }
    } else {
        s_reference_dt_acc_s = 0.0f;
    }

    if (source == MOTOR_FEEDBACK_SRC_HALL) {
        s_hall_dt_acc_s += dt_s;
        s_hall_adc_div++;
        if (s_hall_adc_div >= MOTOR_HALL_ADC_DIVIDER) {
            s_hall_adc_div = 0U;
            Motor_Hall_AngleUpdateDefault(motor->hall_table, Motor_Hall_Read(false, motor->hall_extra_samples),
                                          motor->ol_pole_pairs, s_hall_dt_acc_s);
            s_hall_dt_acc_s = 0.0f;
        }
    } else {
        s_hall_adc_div = 0U;
        s_hall_dt_acc_s = 0.0f;
    }
}

#ifdef __cplusplus
}
#endif
