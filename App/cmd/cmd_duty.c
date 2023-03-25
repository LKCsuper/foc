/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-25 18:17:35
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
    DUTY_NONE,
    DUTY_CREATE,                        /* 创建 */
    DUTY_DESTORY,                       /* 销毁 */
    DUTY_SET,                           /* 设置占空比 */
    DUTY_START,
    DUTY_STOP,
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
	Bsp_Tim_StartPwm(false);
    while (1)
    {
        
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
VOID Cmd_Motor_Duty(int cTaskStat, int cPrecentage)
{
    /* 销毁和创建任务 */
    switch (cTaskStat)
    {
        case DUTY_CREATE:
            PRINTF("占空比 创建 任务 \n");
            /* 打印不同格式的任务 */
            taskPlotHandle = osThreadNew(Duty_ControlTask, NULL, &taskDutyAttr);
            break;
        case DUTY_DESTORY:
            // TODO 创建任务之后 再运行这里Bsp_Tim_StopPwm会导致卡死
            PRINTF("占空比 销毁 任务 \n");
            Bsp_Tim_StopPwm(false);
            if (osOK == osThreadTerminate(taskPlotHandle))
            {
                //PRINTF("success \n");
            } 
            else
            {
                //PRINTF("err \n");
            }
            break;
        case DUTY_SET:
            PRINTF("占空比 百分比[%d]\n", cPrecentage);
            TIMER_UPDATE_DUTY_M1((HW_PWM1_TIM->ARR / 100) * cPrecentage, \
            (HW_PWM1_TIM->ARR / 100) * cPrecentage, (HW_PWM1_TIM->ARR / 100) * cPrecentage);
            break;
        case DUTY_START:
            PRINTF("Duty Start\n");
            TIMER_UPDATE_DUTY_M1(HW_PWM1_TIM->ARR / 5, HW_PWM1_TIM->ARR / 5, HW_PWM1_TIM->ARR / 5);
            Bsp_Tim_StartPwm(false);
            break;
        case DUTY_STOP:
            //PRINTF("Duty Stop\n");
            Bsp_Tim_StopPwm(false);
            break;
        default:
            break;
    }

    return;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                duty, Cmd_Motor_Duty, duty task);
#ifdef __cplusplus
}
#endif
