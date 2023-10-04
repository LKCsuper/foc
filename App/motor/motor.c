/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-03 18:19:51
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-03 18:21:48
 * @detail: 
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
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
VOID MOTOR_ShowParam(VOID)
{
    MOTOR_PARAM_S *stMotorNow = &motor; 

    PRINTF("Current Offset [%f] [%f] [%f]\r\n", stMotorNow->currentOffset[0], stMotorNow->currentOffset[1], stMotorNow->currentOffset[2]);
    PRINTF("Voltage Offset [%f] [%f] [%f]\r\n", stMotorNow->voltOffset[0], stMotorNow->voltOffset[1], stMotorNow->voltOffset[2]);
    PRINTF("Volt Un Offset [%f] [%f] [%f]\r\n", stMotorNow->voltundrivenOffset[0], stMotorNow->voltundrivenOffset[1], stMotorNow->voltundrivenOffset[2]);

    return;
}
#ifdef __cplusplus
}
#endif


