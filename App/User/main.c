/*
 * @Description: 主函数文件
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-04 10:41:58
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
static StackType_t taskInitStack[1024];
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
float target;
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
    //Detect_Init();
    PRINTF("Init Task %d\r\n", TIM1->ARR);
    RTT_Debug_Init();
    Command_Init();
    MOTOR_ShowParam();
    Bsp_Tim_StartPwm(MOTOR1);
    while (1)
    {
        //SEGGER_RTT_printf(0,"Test\r\n");
        //PRINTF("TEST %f %f %f %f\r\n", motor.ia, motor.ib, motor.ic, rtU.Udc);
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
    /* 任务bsp初始化 */
    Bsp_RTOS_Init();
    /* 内核初始化 */
    osKernelInitialize();

    Motor_Simple_BldcInit();

    /* 创建新任务 */
    osThreadNew(Task_Init, NULL, &taskInitAttr);

    /* 任务启动 */
	if (osOK != osKernelStart())
    {
        /* PRINTF("Task Start err.\r\n"); */
    }

#if 0
    Bsp_Init();
    Motor_Simple_BldcInit();
    //Bsp_Tim_StartPwm(MOTOR1);
    target = 0.05;
    while(1)
    {
        move(target);
    }
#endif
}

#ifdef __cplusplus
}
#endif
