/*
 * @Description: 电机模块对外调试接口
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "motor.h"
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 显示电机参数
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void MOTOR_ShowParam(void)
{
    MOTOR_PARAM_S *stMotorNow = &motor; 

    printf("Current Offset [%f] [%f] [%f]\r\n", stMotorNow->currentOffset[0], stMotorNow->currentOffset[1], stMotorNow->currentOffset[2]);
    printf("Voltage Offset [%f] [%f] [%f]\r\n", stMotorNow->voltOffset[0], stMotorNow->voltOffset[1], stMotorNow->voltOffset[2]);
    printf("Volt Un Offset [%f] [%f] [%f]\r\n", stMotorNow->voltundrivenOffset[0], stMotorNow->voltundrivenOffset[1], stMotorNow->voltundrivenOffset[2]);
}
#ifdef __cplusplus
}
#endif
