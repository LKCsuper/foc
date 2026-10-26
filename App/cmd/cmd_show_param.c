/*
 * @Description: 显示 Hall 传感器状态
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2025-07-13
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#if MOTOR_CFG_CMD
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 显示三路 Hall 传感器电平
 * @param {int} argc 兼容参数，本实现不使用
 * @param {char *[]} argv 兼容参数，本实现不使用
 * @return {int} 本实现固定返回 0
 * @author: lkc
 */
int Cmd_Show(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    PRINTF("Hall [%x]\r\n", Motor_Hall_Read(false, 1));
    PRINTF("Hall0 [%d]\r\n", GPIO_ReadInputDataBit(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1));
    PRINTF("Hall1 [%d]\r\n", GPIO_ReadInputDataBit(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2));
    PRINTF("Hall2 [%d]\r\n", GPIO_ReadInputDataBit(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3));
    return 0;
}

/* Cmd_Show 注册在 command.c 命令表中 */
#endif /* MOTOR_CFG_CMD */
#ifdef __cplusplus
}
#endif
