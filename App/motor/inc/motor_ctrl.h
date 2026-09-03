/*
 * @Description: 开环拖动与电压输出控制
 * @Author: lkc
 */
#ifndef _MOTOR_CTRL_H
#define _MOTOR_CTRL_H
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
 * @description: 简易开环拖动，按相邻两次调用的时间差对角度积分。
 * @param {float} speed  开环转速指令，决定角度累加速率
 * @return {*}
 * @author: lkc
 */
VOID Motor_Ctrl_OpenLoop(float speed);

/**
 * @description: 开环 SVPWM 占空比输出（定时器直通版）。
 * @return {*}
 * @author: lkc
 */
VOID Motor_Ctrl_OpenLoop1(VOID);

#ifdef __cplusplus
}
#endif
#endif
