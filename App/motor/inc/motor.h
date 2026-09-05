/*
 * @Description: 电机模块对外调试接口
 * @Author: lkc
 */
#ifndef __MOTOR_H__
#define __MOTOR_H__
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
 * @description: 通过串口打印当前电机偏移等参数。
 * @return {*}
 * @author: lkc
 */
void MOTOR_ShowParam(void);

#ifdef __cplusplus
}
#endif
#endif
