/*
 * @Description: 编码器检测命令
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-09-07 21:13:14
 * @LastEditors: lkc
 * @LastEditTime: 2025-07-13
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 编码器检测
 *   encoder detect   执行编码器检测
 */
int Cmd_Motor_Encoder(int argc, char *argv[])
{
    if (argc >= 2 && strcmp(argv[1], "detect") == 0) {
        /* TODO: 编码器检测逻辑 */
        PRINTF("encoder detect (TODO)\r\n");
    } else {
        PRINTF("usage: encoder detect\r\n");
        return -1;
    }
    return 0;
}

/* Cmd_Motor_Encoder 注册在 command.c 命令表中 */
#ifdef __cplusplus
}
#endif
