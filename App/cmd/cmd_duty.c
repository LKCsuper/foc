/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-11 15:29:47
 * @FilePath: \foc\Library\Bsp\bsp_dma.c
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* 参数1 */
typedef enum
{
    PLOT_NONE,
    PLOT_CREATE,                        /* 创建 */
    PLOT_DESTORY,                       /* 销毁 */
    PLOT_NUM,
}PLOT_TYPE;

/* 参数2,显示类型 */
typedef enum
{
    PLOT_CREATE_NONE,
    PLOT_CURRENT,                       /* 电流 */
    PLOT_CREATE_NUM,
}PLOT_CREATE_TYPE;

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
    .name = "Task Plot",
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
VOID Duty_ControlTask(VOID *argument)
{
    PRINTF("\n");
    while (1)
    {
        Bsp_Tim_StartPwm(false);
        TIMER_UPDATE_DUTY_M1(TIM1->ARR / 2, 0, 0);
        osDelay(100);
        TIMER_UPDATE_DUTY_M1(0, TIM1->ARR / 2, 0);
        osDelay(100);
        TIMER_UPDATE_DUTY_M1(0, 0, TIM1->ARR / 2);
        osDelay(100);
    }
}

/**
 * @description: 
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Cmd_Motor_Duty(char argc, char *argv)
{
    if (argc < 2)
    {
        return;
    }

    for (UCHAR i = 1; i < argc; i++)
    {
        PRINTF("PARAM [%d]  [%s]", i, &argv[argv[i]]);
    }
    PRINTF("\n");

    /* 销毁和创建任务 */
    switch (atoi(&argv[argv[1]]))
    {
        case PLOT_CREATE:
            PRINTF("占空比 创建 任务 ");
            /* 打印不同格式的任务 */
            taskPlotHandle = osThreadNew(Duty_ControlTask, NULL, &taskDutyAttr);
            break;
        case PLOT_DESTORY:
            PRINTF("占空比 销毁 任务 ");
            Bsp_Tim_StopPwm(false);
            if (osOK == osThreadTerminate(taskPlotHandle))
            {
                PRINTF("success \n");
            } 
            else
            {
                PRINTF("err \n");
            }
            break;
        default:
            break;
    }

    return;
}

NR_SHELL_CMD_EXPORT(duty,  Cmd_Motor_Duty);

#ifdef __cplusplus
}
#endif
