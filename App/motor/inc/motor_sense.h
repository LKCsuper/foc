/*
 * @Description: 电流/相电压/母线采样换算（采样帧 → 物理量）
 * @Author: lkc
 */
#ifndef _MOTOR_SENSE_H
#define _MOTOR_SENSE_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_param.h"
#include "hal_motor.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 用已捕获的采样帧换算三相电流/相电压/母线，写入运行时。
 * @detail: 不访问 ADC 寄存器。vb/vc 相对 SENS 通道仍按本板历史映射：
 *          va←SENS1、vb←SENS3、vc←SENS2（与 voltundrivenOffset 下标一致）。
 * @param {MOTOR_PARAM_S *} m 电机运行时，不可为空
 * @param {const hal_motor_sample_t *} sample 本周期采样帧，不可为空
 * @param {float} i_scale 电流比例 (A/count)
 * @param {float} v_gain 相电压增益（无量纲）
 * @return {void} 无
 * @author: lkc
 */
void Motor_Sense_ApplyFrame(MOTOR_PARAM_S *m, const hal_motor_sample_t *sample, float i_scale, float v_gain);

/**
 * @description: 捕获采样帧并换算写入运行时（ISR 默认入口）。
 * @param {MOTOR_PARAM_S *} m 电机运行时参数
 * @return {void} 无
 * @author: lkc
 */
void Motor_Sense_Update(MOTOR_PARAM_S *m);

#ifdef __cplusplus
}
#endif
#endif
