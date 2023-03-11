/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-11 00:10:00
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
void Cmd_Task_Show(char argc, char *argv)
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

NR_SHELL_CMD_EXPORT(show,  Cmd_Task_Show);

#ifdef __cplusplus
}
#endif
