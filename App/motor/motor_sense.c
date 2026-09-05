/*
 * @Description: 电流/相电压/母线采样换算（ADC → 物理量）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "motor_sense.h"
#include "motor_math.h"
#include "hal_motor.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MOTOR_CURRENT_OFFSET_MIN      (1000.0f)
#define MOTOR_CURRENT_OFFSET_MAX      (3000.0f)
#define MOTOR_CURRENT_OFFSET_DEFAULT  (2045.0f)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 校验电流零点偏移，异常时回退默认中点
 * @param {float} offset  Flash/校准时写入的偏移 (ADC 计数)
 * @return {float} 有效偏移
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
 * @description: 从 HAL 读取 ADC，换算三相电流/相电压/母线电压写入运行时
 * @param {MOTOR_PARAM_S} *m  电机运行时参数
 * @return {*}
 * @author: lkc
 */
void Motor_Sense_Update(MOTOR_PARAM_S *m)
{
    float current_offset0;
    float current_offset1;
    float current_offset2;
    float i_scale;
    float v_gain;

    if (m == NULL) {
        return;
    }

    m->currentAdc[0] = Hal_Motor_CurrentAdc(0U);
    m->currentAdc[1] = Hal_Motor_CurrentAdc(1U);
    m->currentAdc[2] = Hal_Motor_CurrentAdc(2U);

    m->voltAdc[0] = Hal_Motor_SenseVolts(0U);
    m->voltAdc[1] = Hal_Motor_SenseVolts(1U);
    m->voltAdc[2] = Hal_Motor_SenseVolts(2U);

    current_offset0 = Motor_Sense_CurrentOffsetValid(m->currentOffset[0]);
    current_offset1 = Motor_Sense_CurrentOffsetValid(m->currentOffset[1]);
    current_offset2 = Motor_Sense_CurrentOffsetValid(m->currentOffset[2]);

    i_scale = Hal_Motor_CurrentScale();
    m->ia = (m->currentAdc[0] - current_offset0) * i_scale;
    m->ib = (m->currentAdc[1] - current_offset1) * i_scale;
    m->ic = (m->currentAdc[2] - current_offset2) * i_scale;

    /*
     * 注意 ADC 通道与相序的映射不是线性的：
     *   va ← SENS1 (offset[0])
     *   vb ← SENS3 (offset[2]) — 交换
     *   vc ← SENS2 (offset[1]) — 交换
     */
    v_gain = Hal_Motor_PhaseVoltageGain();
    m->va = (Hal_Motor_SenseVolts(0U) - m->voltundrivenOffset[0]) * v_gain;
    m->vb = (Hal_Motor_SenseVolts(2U) - m->voltundrivenOffset[2]) * v_gain;
    m->vc = (Hal_Motor_SenseVolts(1U) - m->voltundrivenOffset[1]) * v_gain;

    UTILS_LP_FAST(m->v_bus, Hal_Motor_BusVoltage(), 0.1f);
}

#ifdef __cplusplus
}
#endif
