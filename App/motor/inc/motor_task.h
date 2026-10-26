/*
 * @Description: 电机后台任务（VESC foc pid：速度/位置外环）
 * @Author: lkc
 *
 * 学习要点：电流环留在 ADC 中断；速度 PI 在本任务，默认 1 kHz，只写 Iq 目标。
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
 * @description: 启动速度/位置外环任务（foc pid），只创建一次。
 * @detail: 须在调度器已运行且 MOTOR_Param_Init 之后调用；不阻塞。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Task_Start(void);

#ifdef __cplusplus
}
#endif

#endif
