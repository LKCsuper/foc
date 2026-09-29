/*
 * @Description: 主函数文件
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2026-09-07 23:21:04
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
const osThreadAttr_t taskInitAttr = {
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
 * @author: lkc
 */
void Task_Init(void *argument)
{
    Bsp_Init(); // 初始化硬件

    PRINTF("Init MT6701 %s\r\n", Motor_MT6701_InterfaceName());
    Motor_Encoder_Init();
    MOTOR_Param_Init();   // 初始化Flash参数（直接结构体读写，无需EasyFlash）
    Motor_Encoder_ApplyCalibration(motor.encoder_offset_rad, motor.encoder_inverted, motor.encoder_calibrated);

    /*
     * 开机偏移：Flash 已有有效电流零点则直接使用；否则校准约 5 秒并自动存 Flash。
     * 会撤销运行命令，结束后保持停机，需要转动时由 olspeed/olstart 明确启动。
     */
    Detect_Init();     // 开机偏移校准（有 Flash 则跳过）
    MOTOR_ShowParam(); // 显示参数（在Shell启动前打印完，避免抢UART）
    PRINTF("Init Task period=%lu\r\n", (unsigned long)Hal_Motor_PwmPeriodCounts());
    // RTT_Debug_Init();
    Command_Init();       // 命令行初始化（最后启动，避免Shell抢UART）
    Motor_Task_Start();   // 速度/位置外环任务；电流环仍在 ADC 中断
    Hal_Motor_PwmStart(); // 先恢复三相 PWM 模式，此时仍没有运行命令
    while (1) {
        osDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @description: 主函数
 * @detail:
 * @return {*}
 * @author: lkc
 */
LONG main(VOID)
{
    Bsp_RTOS_Init();         // 初始化RTOS
    osKernelInitialize();    // 初始化CMSIS-RTOS内核
    Motor_Simple_BldcInit(); // 电机参数初始化

    osThreadNew(Task_Init, NULL, &taskInitAttr); // 创建初始化任务

    /* 任务启动 */
    if (osOK != osKernelStart()) {
        /* PRINTF("Task Start err.\r\n"); */
    }
}

#ifdef __cplusplus
}
#endif
