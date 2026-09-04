/*
 * @Description: FOC 电流环与观测器增益整定
 * @Author: lkc
 */
#ifndef _MOTOR_CAL_H
#define _MOTOR_CAL_H
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
 * @description: 按 R/L 与电流环时间常数整定 FOC 电流环 Kp/Ki 与观测器增益。
 * @param {MOTOR_PARAM_S} *m  运行时参数；R/L/磁链取自 sensorless_* 字段
 * @param {float} tc          电流环时间常数 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Calc_FocCurrentKpKi(MOTOR_PARAM_S *m, float tc);

/**
 * @description: 计算电机电气速度环 Kp/Ki 的预留接口
 * @return {*}
 * @author: lkc
 */
VOID MOTOR_Cal_Electric_KP_KI(VOID);

#ifdef __cplusplus
}
#endif
#endif
