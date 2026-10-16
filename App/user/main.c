/*
 * @Description: 主函数文件
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2026-10-16 19:22:52
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include "motor_sensor.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 初始化任务动态分配，退出后由空闲任务回收栈和任务控制块。 */
const osThreadAttr_t taskInitAttr = {
    .name = "Task Init",
    /* 注意如果超出这个中断最大,任务将创建失败 */
    .priority = PRI_INIT,
    .stack_size = 2048U * sizeof(StackType_t), // CMSIS-RTOS 栈大小单位为字节
};
float target;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 打印电机参数
 * @detail:
 * @return {*}
 * @author: lkc
 */
void MOTOR_Param_Print(void)
{
    MOTOR_PARAM_S *stMotorNow = &motor;
    PRINTF("Init MT6701 %s\r\n", Motor_MT6701_InterfaceName());
    PRINTF("Init Task period=%lu\r\n", (unsigned long)Hal_Motor_PwmPeriodCounts());

    PRINTF("Current Offset [%f] [%f] [%f]\r\n", stMotorNow->currentOffset[0], stMotorNow->currentOffset[1],
           stMotorNow->currentOffset[2]);
    PRINTF("Voltage Offset [%f] [%f] [%f]\r\n", stMotorNow->voltOffset[0], stMotorNow->voltOffset[1],
           stMotorNow->voltOffset[2]);
    PRINTF("Volt Un Offset [%f] [%f] [%f]\r\n", stMotorNow->voltundrivenOffset[0], stMotorNow->voltundrivenOffset[1],
           stMotorNow->voltundrivenOffset[2]);
    PRINTF("\r\nFOC boot, UART4=%lu 8N1\r\n", (unsigned long)DEBUG_USART_BAUD);
    PRINTF("Firmware built: %s %s\r\n", __DATE__, __TIME__);

    return;
}

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
    MOTOR_Param_Init();  // 初始化Flash参数（直接结构体读写，无需EasyFlash）
    COMMON_ASSERT(Motor_Sensor_Init()); // 初始化传感器、恢复校准并选择默认反馈源
    MOTOR_Param_Print(); // 显示参数（在Shell启动前打印完，避免抢UART）
    Detect_Init();                    // 开机检测电流偏移校准（有 Flash 则跳过）
    Command_Init();                   // 命令行初始化（最后启动，避免Shell抢UART）
    Motor_Task_Start();               // 速度/位置外环任务；电流环仍在 ADC 中断
    Hal_Motor_PwmStart();             // 先恢复三相 PWMp 模式，此时仍没有运行命令
    osThreadExit();                   // 初始化完成；空闲任务获得运行机会后释放动态内存
}

/**
 * @description: 主函数
 * @detail:
 * @return {*}
 * @author: lkc
 */
LONG main(VOID)
{
    Bsp_RTOS_Init();                                                    // 初始化RTOS
    COMMON_ASSERT(osKernelInitialize() == osOK);                        // 初始化内核
    COMMON_ASSERT(osThreadNew(Task_Init, NULL, &taskInitAttr) != NULL); // 创建初始化任务
    COMMON_ASSERT(osKernelStart() == osOK);                             /* 任务启动 */

    return;
}

#ifdef __cplusplus
}
#endif
