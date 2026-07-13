/*
 * @Description: 串口绘图命令
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2025-07-13
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private variables ---------------------------------------------------------*/
STATIC TaskHandle_t taskPlotHandle;

const osThreadAttr_t taskPlotAttr =
{
    .name = "Task Plot",
    .priority = PRI_INIT,
};
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 串口绘图任务（当前仅占位，具体数据在循环内填入）
 */
VOID Serial_PlotCurentTask(VOID *argument)
{
    (void)argument;
    PRINTF("\n");
    while (1)
    {
        /* 绘图数据输出 */
        //TEXT_PLOT(current, "%f, %f, %f, %f", motor.ia, motor.ib, motor.ic, motor.v_bus);
        //TEXT_PLOT(voltage, "%f, %f, %f", motor.va, motor.vb, motor.vc);
    }
}

/**
 * @description: 串口绘图控制
 *   plot create current   创建电流绘图任务
 *   plot destory          销毁绘图任务
 */
int Cmd_Serial_Plot(int argc, char *argv[])
{
    if (argc < 2) {
        PRINTF("usage: plot create current | plot destory\r\n");
        return -1;
    }

    if (strcmp(argv[1], "create") == 0) {
        if (argc >= 3 && strcmp(argv[2], "current") == 0) {
            PRINTF("plot: create current task\r\n");
            taskPlotHandle = osThreadNew(Serial_PlotCurentTask, NULL, &taskPlotAttr);
        } else {
            PRINTF("usage: plot create current\r\n");
            return -1;
        }
    } else if (strcmp(argv[1], "destory") == 0) {
        PRINTF("plot: destory task ");
        if (osOK == osThreadTerminate(taskPlotHandle)) {
            PRINTF("success\r\n");
        } else {
            PRINTF("err\r\n");
        }
    } else {
        PRINTF("unknown: %s\r\n", argv[1]);
        return -1;
    }
    return 0;
}

/* Cmd_Serial_Plot 注册在 command.c 命令表中 */
#ifdef __cplusplus
}
#endif
