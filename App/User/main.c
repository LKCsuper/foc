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
	bool startup_run_enable;
	float startup_target_rpm;

	// osDelay(pdMS_TO_TICKS(5000));
    Bsp_Init();             // 初始化硬件
    MOTOR_Param_Init();     // easyflash初始化，先加载Flash参数

	/*
	 * 偏移检测必须临时撤销运行命令，否则 FOC 中断会和检测代码同时操作
	 * TIM1。这里先保存参数初始化得到的上电运行意图，检测完成后再恢复。
	 */
	startup_run_enable = motor.ol_enable;
	startup_target_rpm = motor.ol_target_rpm;
    Detect_Init();          // 电机参数检测（覆盖Flash中的零值偏移）
    MOTOR_ShowParam();      // 显示参数（在Shell启动前打印完，避免抢UART）
    PRINTF("Init Task %d\r\n", TIM1->ARR);
    // RTT_Debug_Init();
    Command_Init();         // 命令行初始化（最后启动，避免Shell抢UART）
    Bsp_Tim_StartPwm(MOTOR1);   // 先恢复三相 PWM 模式，此时仍没有运行命令

	/*
	 * PWM 准备好以后再恢复目标转速，避免输出关闭期间 PI 提前积分。
	 * 当前调试默认 ol_enable=false，因此上电保持停机，等待 olspeed/olstart。
	 * 若用户明确把默认 ol_enable 设为 true，才恢复保存的目标转速并起转。
	 */
	/* 保留默认目标，方便上电停机后直接 olstart；仅 enable 决定是否自动转动。 */
	if (startup_target_rpm != 0.0f) {
		motor.ol_target_rpm = startup_target_rpm;
		if (startup_run_enable) {
			motor.ol_enable = true;
			PRINTF("Startup run restored: %.0f RPM\r\n", startup_target_rpm);
		}
	}
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
