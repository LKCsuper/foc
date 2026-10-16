/*
 * @Description: duty 占空比调试命令（非 FOC 主控制路径）
 * @Author: lkc
 *
 * 仅用于硬件探针/开环占空比实验。主路径请用 olstart/olspeed/olstop。
 * 电机使能或故障锁存时拒绝写 PWM，避免绕过控制仲裁。
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

STATIC const osThreadAttr_t taskDutyAttr = {
    .name = "Task Plot",
    .priority = PRI_INIT,
};
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 判断当前是否允许手动改 PWM。
 * @detail: 故障已锁存或电机仍使能时拒绝，并打印原因。
 * @return {bool} true=可以下发占空比
 * @author: lkc
 */
static bool Duty_AllowManualPwm(void)
{
    if (Motor_Protect_IsFaultLatched()) {
        PRINTF("duty blocked: fault latched, use olfault clear\r\n");
        return false;
    }
    if (motor.ol_enable) {
        PRINTF("duty blocked: motor enabled, use olstop first\r\n");
        return false;
    }
    return true;
}

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 占空比循环翻转任务（调试用，非 FOC 主路径）。
 * @author: lkc
 */
VOID Duty_ControlTask(VOID *argument)
{
    (void)argument;
    uint32_t period = Hal_Motor_PwmPeriodCounts();
    uint32_t pulse = period / 10U;

    PRINTF("\n");
    Hal_Motor_PwmStart();
    while (1) {
        if (!Duty_AllowManualPwm()) {
            Hal_Motor_PwmSetSafe();
            osDelay(pdMS_TO_TICKS(200));
            continue;
        }
        Hal_Motor_PwmSetDuty(pulse, 0U, 0U);
        Hal_Motor_PwmCommit();

        osDelay(pdMS_TO_TICKS(50));
        Hal_Motor_PwmSetDuty(0U, pulse, 0U);
        Hal_Motor_PwmCommit();

        osDelay(pdMS_TO_TICKS(50));
        Hal_Motor_PwmSetDuty(0U, 0U, pulse);
        Hal_Motor_PwmCommit();
        osDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @description: duty 调试命令（非 FOC 主控制）。
 *   duty create       创建占空比翻转任务
 *   duty destory      销毁任务并停 PWM
 *   duty set <pct>    设置占空比百分比
 *   duty start        启动 PWM
 *   duty stop         停止 PWM
 * @author: lkc
 */
int Cmd_Motor_Duty(int argc, char *argv[])
{
    int pct;

    if (argc < 2) {
        PRINTF("usage: duty create|destory|set <pct>|start|stop  (debug only)\r\n");
        return -1;
    }

    if (strcmp(argv[1], "create") == 0) {
        if (!Duty_AllowManualPwm()) return -1;
        PRINTF("duty task create (debug)\r\n");
        taskPlotHandle = osThreadNew(Duty_ControlTask, NULL, &taskDutyAttr);
    } else if (strcmp(argv[1], "destory") == 0) {
        PRINTF("duty task destory\r\n");
        Hal_Motor_PwmStop();
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
        if (!Duty_AllowManualPwm()) return -1;
        pct = atoi(argv[2]);
        if (pct < 0 || pct > 100) {
            PRINTF("duty percent must be 0..100\r\n");
            return -1;
        }
        PRINTF("duty set [%d%%]\r\n", pct);
        {
            uint32_t period = Hal_Motor_PwmPeriodCounts();
            uint32_t duty = (period / 100U) * (uint32_t)pct;

            Hal_Motor_PwmSetDuty(duty, duty, duty);
            Hal_Motor_PwmCommit();
        }
    } else if (strcmp(argv[1], "start") == 0) {
        if (!Duty_AllowManualPwm()) return -1;
        PRINTF("duty start (debug)\r\n");
        {
            uint32_t duty = Hal_Motor_PwmPeriodCounts() / 5U;

            Hal_Motor_PwmSetDuty(duty, duty, duty);
            Hal_Motor_PwmStart();
            Hal_Motor_PwmCommit();
        }
    } else if (strcmp(argv[1], "stop") == 0) {
        Hal_Motor_PwmStop();
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
