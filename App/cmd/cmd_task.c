/*
 * @Description: FreeRTOS 任务状态查看命令
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2025-07-13
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private variables ---------------------------------------------------------*/
CHAR gacRunInfo[512];
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 打印所有 FreeRTOS 任务状态和运行统计
 * @author: lkc
 */
int Cmd_Task_Show(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    memset(gacRunInfo, 0, sizeof(gacRunInfo));
    PRINTF("=======================================================\r\n");
    PRINTF("任务名      任务状态   优先级  剩余栈  任务序号\r\n");
    vTaskList((char *)&gacRunInfo);
    PRINTF("%s\r\n", gacRunInfo);

    PRINTF("\r\n任务名        运行计数        使用率\r\n");
    vTaskGetRunTimeStats((char *)&gacRunInfo);
    PRINTF("%s\r\n", gacRunInfo);
    return 0;
}

/* Cmd_Task_Show 注册在 command.c 命令表中 */
#ifdef __cplusplus
}
#endif
