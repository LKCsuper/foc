/*
 * @Description: 主函数文件
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-24 23:23:22
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
static StackType_t taskInitStack[2048];
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
    Bsp_Init();             // 初始化硬件
    Motor_Encoder_Init();   // TIM4 AB 编码器
    MOTOR_Param_Init();     // 初始化Flash参数（直接结构体读写，无需EasyFlash）

    /*
     * 开机只做电流/电压偏移校准。会阻塞约 5 秒并撤销运行命令，结束后保持停机，
     * 需要转动时由 Shell 的 olspeed/olstart 明确启动。
     */
    Detect_Init();          // 开机偏移校准
    MOTOR_ShowParam();      // 显示参数（在Shell启动前打印完，避免抢UART）
    PRINTF("Init Task %d\r\n", TIM1->ARR);
    // RTT_Debug_Init();
    Command_Init();         // 命令行初始化（最后启动，避免Shell抢UART）
    Bsp_Tim_StartPwm(MOTOR1);   // 先恢复三相 PWM 模式，此时仍没有运行命令
    while (1)
    {
        osDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @description: 主函数
 * @detail:
 * @return {*}
 */
LONG main(VOID)
{
    Bsp_RTOS_Init();    // 初始化RTOS
    osKernelInitialize();   // 初始化CMSIS-RTOS内核
    Motor_Simple_BldcInit(); // 电机参数初始化

    osThreadNew(Task_Init, NULL, &taskInitAttr);    // 创建初始化任务

    /* 任务启动 */
	if (osOK != osKernelStart())
    {
        /* PRINTF("Task Start err.\r\n"); */
    }
}

#ifdef __cplusplus
}
#endif
