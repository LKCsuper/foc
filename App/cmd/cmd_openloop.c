/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-08-30 10:31:14
 * @LastEditors: lkc
 * @LastEditTime: 2023-08-31 21:37:40
 * @detail: 
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* 参数1 */
typedef enum
{
    OPEN_LOOP_NONE,
    OPEN_LOOP_CREATE,                        /* 创建 */
    OPEN_LOOP_DESTORY,                       /* 销毁 */
    OPEN_LOOP_SET,                           /* 设置占空比 */
    OPEN_LOOP_START,
    OPEN_LOOP_STOP,
    OPEN_LOOP_NUM,
}OPEN_LOOP_TYPE;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 任务栈 */
STATIC TaskHandle_t taskPlotHandle;
/*
STATIC StackType_t taskPlotStack[128];
STATIC StaticTask_t taskPlotTcb; //CreateAppTask任务控制块 
*/
STATIC const osThreadAttr_t taskDutyAttr = 
{
    .name = "Open Loop",
    /* 注意如果超出这个中断最大,任务将创建失败 */
    .priority = PRI_INIT,
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 串口绘画任务初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Openloop_ControlTask(VOID *argument)
{
    float angle = 10.0f;
    /* 转换为弧度 */
    angle = DEG2RAD_f(angle);

    PRINTF("\n");
    /* 开启pwm */
	Bsp_Tim_StartPwm(MOTOR1);
    while (1)
    {
        //Motor_Ctrl_OpenLoop(50);
        Motor_Ctrl_OpenLoop1();
        //vTaskDelay(10);
    }
}

/**
 * @description: 
 * @detail: 
 * @param {UCHAR} cTaskStat 任务状态
 * @param {UCHAR} cContStat 控制状态
 * @param {UCHAR} cPrecentage 百分比
 * @return {*}
 * @author: lkc
 */
VOID Cmd_Motor_OpenLoop(int cTaskStat, int cPrecentage)
{
    /* 销毁和创建任务 */
    switch (cTaskStat)
    {
        case OPEN_LOOP_CREATE:
            PRINTF("占空比 创建 任务 \n");
            /* 打印不同格式的任务 */
            taskPlotHandle = osThreadNew(Openloop_ControlTask, NULL, &taskDutyAttr);
            break;
        default:
            break;
    }

    return;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                openloop, Cmd_Motor_OpenLoop, open loop task);
#ifdef __cplusplus
}
#endif
