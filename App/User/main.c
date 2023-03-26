/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-25 23:39:48
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
/* 任务栈 */
static StackType_t taskInitStack[128];
static StaticTask_t taskInitTcb; //CreateAppTask任务控制块 
const osThreadAttr_t taskInitAttr = 
{
    .name = "Task Init",
    /* 注意如果超出这个中断最大,任务将创建失败 */
    .priority = PRI_INIT,
    .cb_mem = &taskInitTcb,
    .cb_size = sizeof(taskInitTcb),
    .stack_mem = &taskInitStack,
    .stack_size = sizeof(taskInitStack),
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 任务初始化
 * @detail: 
 * @param {void} *argument
 * @return {*}
 */
void Task_Init(void *argument)
{
    Bsp_Init();
    PRINTF("Init Task\r\n");

    Command_Init();

    while (1)
    {
        osDelay(1000);
    }
}

/**
 * @description: 主函数
 * @detail: 
 * @return {*}
 */
LONG main(VOID)
{   
    Bsp_RTOS_Init();
	osKernelInitialize();
    osThreadNew(Task_Init, NULL, &taskInitAttr);
	if (osOK != osKernelStart())
    {
        /* PRINTF("Task Start err.\r\n"); */
    }

    while(1){}
}

#ifdef __cplusplus
}
#endif
