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
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 显示三路 Hall 传感器电平
 */
int Cmd_Show(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    PRINTF("Hall [%x]\r\n", utils_read_hall(MOTOR1, 1));
    PRINTF("Hall0 [%d]\r\n", GPIO_ReadInputDataBit(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1));
    PRINTF("Hall1 [%d]\r\n", GPIO_ReadInputDataBit(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2));
    PRINTF("Hall2 [%d]\r\n", GPIO_ReadInputDataBit(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3));
    return 0;
}

/* Cmd_Show 注册在 command.c 命令表中 */
#ifdef __cplusplus
}
#endif
