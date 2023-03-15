/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-15 19:36:37
 * @FilePath: \foc\Library\Bsp\bsp_dma.c
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CHAR gacRunInfo[512];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 
 * @detail: 
 * @param {void} *pvParameters
 * @return {*}
 * @author: lkc
 */
VOID Cmd_Task_Show(VOID)
{
    memset(gacRunInfo, 0, sizeof(gacRunInfo));				//信息缓冲区清零
    PRINTF("=======================================================\r\n");
    PRINTF("任务名      任务状态   优先级  剩余栈  任务序号\r\n");
    vTaskList((char *)&gacRunInfo);
    PRINTF("%s\r\n", gacRunInfo);

    PRINTF("\r\n任务名        运行计数        使用率\r\n");
    vTaskGetRunTimeStats((char *)&gacRunInfo);
    PRINTF("%s\r\n", gacRunInfo);

    return;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                task, Cmd_Task_Show, show task);

#ifdef __cplusplus
}
#endif
