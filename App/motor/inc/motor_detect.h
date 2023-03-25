/*
 * @Description: 检测参数
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-03-21 23:50:03
 * @FilePath: \foc\App\motor\inc\utils.h
 */
#ifndef _MOTOR_DETECT_H
#define _MOTOR_DETECT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------- nv-----------------------------------*/
int mcpwm_foc_dc_cal(bool cal_undriven);

#ifdef __cplusplus
}
#endif

#endif
