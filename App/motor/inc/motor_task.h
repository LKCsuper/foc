/*
 * @Description: 电机后台任务入口
 * @Author: lkc
 */
#ifndef _MOTOR_TASK_H
#define _MOTOR_TASK_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: PID 控制后台任务入口。
 * @return {*}
 * @author: lkc
 */
void Motor_PidControl(void);

#ifdef __cplusplus
}
#endif

#endif
