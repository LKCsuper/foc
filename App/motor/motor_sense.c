/*
 * @Description: 电流/相电压/母线采样换算（采样帧 → 物理量）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "motor_sense.h"
#include "motor_math.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MOTOR_CURRENT_OFFSET_MIN     (1000.0f)
#define MOTOR_CURRENT_OFFSET_MAX     (3000.0f)
#define MOTOR_CURRENT_OFFSET_DEFAULT (2045.0f)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 校验电流零点偏移，异常时回退默认中点（历史默认 2045，非严格 2048）
 * @param {float} offset Flash/校准时写入的偏移 (ADC 计数)
 * @return {float} 有效偏移 (ADC 计数)
 * @author: lkc
 */
static float Motor_Sense_CurrentOffsetValid(float offset)
{
    if (offset < MOTOR_CURRENT_OFFSET_MIN || offset > MOTOR_CURRENT_OFFSET_MAX) {
        return MOTOR_CURRENT_OFFSET_DEFAULT;
    }
    return offset;
}

/**
 * @description: 用已捕获的采样帧换算三相电流/相电压/母线，写入运行时
 * @detail: 不访问 ADC 寄存器。电流与相电压均按本板历史 ACB 映射：
 *          ia←CURR1、ib←CURR3、ic←CURR2；va←SENS1、vb←SENS3、vc←SENS2。
 * @param {MOTOR_PARAM_S} *m 电机运行时，不可为空
 * @param {const hal_motor_sample_t} *sample 本周期采样帧，不可为空
 * @param {float} i_scale 电流比例 (A/count)
 * @param {float} v_gain 相电压增益（无量纲）
 * @return {*}
 * @author: lkc
 */
void Motor_Sense_ApplyFrame(MOTOR_PARAM_S *m, const hal_motor_sample_t *sample, float i_scale, float v_gain)
{
    float current_offset0;
    float current_offset1;
    float current_offset2;

    if (m == NULL || sample == NULL) {
        return;
    }

    m->currentAdc[0] = sample->current_adc[0];
    m->currentAdc[1] = sample->current_adc[1];
    m->currentAdc[2] = sample->current_adc[2];

    m->voltAdc[0] = sample->sense_volts[0];
    m->voltAdc[1] = sample->sense_volts[1];
    m->voltAdc[2] = sample->sense_volts[2];

    current_offset0 = Motor_Sense_CurrentOffsetValid(m->currentOffset[0]);
    current_offset1 = Motor_Sense_CurrentOffsetValid(m->currentOffset[1]);
    current_offset2 = Motor_Sense_CurrentOffsetValid(m->currentOffset[2]);

    if ((sample->current_mask & 0x01U) != 0U) {
        m->ia = ((float)sample->current_adc[0] - current_offset0) * i_scale;
    }
    /*
     * 与 Motor_Foc_OutputVoltage 的 ACB PWM（B/C 对调）及相电压 SENS 映射一致：
     *   ib ← CURR3 / SENS3，ic ← CURR2 / SENS2。
     * 电流仍按 ADC 下标 1=B、2=C 换算时，I/F 的 dq 与电压矢量错相，表现为来回摆并易过流。
     */
    if ((sample->current_mask & 0x04U) != 0U) {
        m->ib = ((float)sample->current_adc[2] - current_offset2) * i_scale;
    }
    if ((sample->current_mask & 0x02U) != 0U) {
        m->ic = ((float)sample->current_adc[1] - current_offset1) * i_scale;
    }

    /*
     * 板级相序映射（本板历史约定，非通用）：
     *   va ← SENS1 (index 0)
     *   vb ← SENS3 (index 2)
     *   vc ← SENS2 (index 1)
     * 后续可将映射下沉到 HAL，使帧直接给出相序电压。
     */
    m->va = (sample->sense_volts[0] - m->voltundrivenOffset[0]) * v_gain;
    m->vb = (sample->sense_volts[2] - m->voltundrivenOffset[2]) * v_gain;
    m->vc = (sample->sense_volts[1] - m->voltundrivenOffset[1]) * v_gain;

    UTILS_LP_FAST(m->v_bus, sample->v_bus, 0.1f);
}

/**
 * @description: 捕获采样帧并换算写入运行时（ISR 默认入口）
 * @detail: 经 Hal_Motor_SampleCapture 取帧，再调 ApplyFrame 换算
 * @param {MOTOR_PARAM_S} *m 电机运行时参数，不可为空
 * @return {*}
 * @author: lkc
 */
void Motor_Sense_Update(MOTOR_PARAM_S *m)
{
    hal_motor_sample_t sample;

    if (m == NULL) {
        return;
    }
    if (!Hal_Motor_SampleCapture(&sample)) {
        return;
    }
    Motor_Sense_ApplyFrame(m, &sample, Hal_Motor_CurrentScale(), Hal_Motor_PhaseVoltageGain());
}

#ifdef __cplusplus
}
#endif
