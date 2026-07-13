/*
 * @Description: duty 占空比控制命令
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2025-07-13
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
STATIC TaskHandle_t taskPlotHandle;

STATIC const osThreadAttr_t taskDutyAttr =
{
    .name = "Task Plot",
    .priority = PRI_INIT,
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 占空比循环翻转任务
 */
VOID Duty_ControlTask(VOID *argument)
{
    (void)argument;
    PRINTF("\n");
    Bsp_Tim_StartPwm(false);
    while (1)
    {
        TIMER_UPDATE_DUTY_M1(HW_PWM1_TIM->ARR / 10, 0, 0);
        TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

        osDelay(pdMS_TO_TICKS(50));
        TIMER_UPDATE_DUTY_M1(0, HW_PWM1_TIM->ARR / 10, 0);
        TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

        osDelay(pdMS_TO_TICKS(50));
        TIMER_UPDATE_DUTY_M1(0, 0, HW_PWM1_TIM->ARR / 10);
        TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
        osDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @description: duty 命令
 *   duty create       创建占空比翻转任务
 *   duty destory      销毁任务并停 PWM
 *   duty set <pct>    设置占空比百分比
 *   duty start        启动 PWM
 *   duty stop         停止 PWM
 */
int Cmd_Motor_Duty(int argc, char *argv[])
{
    int pct;

    if (argc < 2) {
        PRINTF("usage: duty create|destory|set <pct>|start|stop\r\n");
        return -1;
    }

    if (strcmp(argv[1], "create") == 0) {
        PRINTF("duty task create\r\n");
        taskPlotHandle = osThreadNew(Duty_ControlTask, NULL, &taskDutyAttr);
    } else if (strcmp(argv[1], "destory") == 0) {
        PRINTF("duty task destory\r\n");
        Bsp_Tim_StopPwm(false);
        if (osOK == osThreadTerminate(taskPlotHandle)) {
            PRINTF("success\r\n");
        } else {
            PRINTF("err\r\n");
        }
    } else if (strcmp(argv[1], "set") == 0) {
        if (argc < 3) {
            PRINTF("usage: duty set <0..100>\r\n");
            return -1;
        }
        pct = atoi(argv[2]);
        if (pct < 0 || pct > 100) {
            PRINTF("duty percent must be 0..100\r\n");
            return -1;
        }
        PRINTF("duty set [%d%%]\r\n", pct);
        TIMER_UPDATE_DUTY_M1((HW_PWM1_TIM->ARR / 100) * pct,
                             (HW_PWM1_TIM->ARR / 100) * pct,
                             (HW_PWM1_TIM->ARR / 100) * pct);
        TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    } else if (strcmp(argv[1], "start") == 0) {
        PRINTF("duty start\r\n");
        TIMER_UPDATE_DUTY_M1(HW_PWM1_TIM->ARR / 5, HW_PWM1_TIM->ARR / 5, HW_PWM1_TIM->ARR / 5);
        Bsp_Tim_StartPwm(false);
        TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    } else if (strcmp(argv[1], "stop") == 0) {
        Bsp_Tim_StopPwm(false);
    } else {
        PRINTF("unknown: %s\r\n", argv[1]);
        return -1;
    }
    return 0;
}

/* Cmd_Motor_Duty 注册在 command.c 命令表中 */
#ifdef __cplusplus
}
#endif
