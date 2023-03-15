/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-15 19:50:14
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
const osThreadAttr_t taskPlotAttr = 
{
    .name = "Task Plot",
    /* 注意如果超出这个中断最大,任务将创建失败 */
    .priority = PRI_INIT,
/* 当前任务为动态创建 */
/*
    .cb_mem = &taskPlotTcb,
    .cb_size = sizeof(taskPlotTcb),
    .stack_mem = &taskPlotStack,
    .stack_size = sizeof(taskPlotStack),
*/
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 串口绘画任务初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Serial_PlotCurentTask(VOID *argument)
{
    PRINTF("\n");
    while (1)
    {
        osDelay(1000);
        // 绘图
        TEXT_PLOT(current, "%f, %f, %f, %f", motor.ia, motor.ib, motor.ic, motor.v_bus);
        TEXT_PLOT(voltage, "%f, %f, %f", motor.va, motor.vb, motor.vc);
    }
}

/**
 * @description: Cmd_Serial_Plot
 * @detail: 
 * @param {char} cTaskStat 任务状态
 * @param {char} cPlotStat 绘画状态
 * @return {*}
 * @author: lkc
 */
VOID Cmd_Serial_Plot(UCHAR cTaskStat, UCHAR cPlotStat)
{
    /* 销毁和创建任务 */
    switch (cTaskStat)
    {
        case PLOT_CREATE:
            /* 打印不同格式的任务 */
            switch (cPlotStat)
            {
                case PLOT_CURRENT:
                    PRINTF("电流 显示 任务\n");
                    taskPlotHandle = osThreadNew(Serial_PlotCurentTask, NULL, &taskPlotAttr);
                    break;
                default:
                    break;
                }
            break;
        case PLOT_DESTORY:
            PRINTF("电流 销毁 任务 ");
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

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                plot, Cmd_Serial_Plot, plot task);
#ifdef __cplusplus
}
#endif
