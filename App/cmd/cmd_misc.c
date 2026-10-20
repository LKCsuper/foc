/*
 * @Description: 系统复位命令
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-06 17:37:56
 * @LastEditors: lkc
 * @LastEditTime: 2025-07-13
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#if MOTOR_CFG_CMD
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 软件复位 MCU
 * @author: lkc
 */
int CMD_Reset(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    Bsp_Misc_Reset();
    return 0;
}

/* CMD_Reset 注册在 command.c 命令表中 */
#endif /* MOTOR_CFG_CMD */
#ifdef __cplusplus
}
#endif
