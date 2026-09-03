/*
 * @Description: FOC 电流环与观测器增益整定
 * @Author: lkc
 */
#ifndef __MOTOR_CAL_H__
#define __MOTOR_CAL_H__
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 按 VESC 电流环时间常数整定 FOC 电流环 Kp/Ki 与观测器增益。
 * @param {mc_configuration} *mcconf  电机配置，需含 R/L/flux_linkage；结果写回 foc_current_kp/ki 与 foc_observer_gain
 * @param {float} tc                    电流环时间常数 (us)
 * @return {*}
 * @author: lkc
 */
void VESC_Calc_Foc_cc_kp_ki_gain(mc_configuration *mcconf, float tc);

#ifdef __cplusplus
}
#endif
#endif
