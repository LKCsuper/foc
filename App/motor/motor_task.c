/*
 * @Description: 电机后台任务：速度/位置外环（对齐 VESC foc pid 线程）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 电流环、SVPWM、霍尔/编码器更新仍在 Motor_DmaInt。
 *   2) 本任务只跑速度 PI（位置环预留），写出 Iq 目标供 ISR 读取。
 *   3) 周期跟 speed_loop_hz；dt 用 Hal_Motor_TimeElapsedSec，不用假定 1 ms。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_TASK_PID_STACK_WORDS 512U
#define MOTOR_TASK_PID_DT_MIN_S    (5.0e-5f)
#define MOTOR_TASK_PID_DT_MAX_S    (0.05f)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static StackType_t s_pid_stack[MOTOR_TASK_PID_STACK_WORDS];
static StaticTask_t s_pid_tcb;
static const osThreadAttr_t s_pid_attr = {
    .name = "foc pid",
    .priority = PRI_SPEED,
    .cb_mem = &s_pid_tcb,
    .cb_size = sizeof(s_pid_tcb),
    .stack_mem = &s_pid_stack,
    .stack_size = sizeof(s_pid_stack),
};
static osThreadId_t s_pid_thread;
/* Private function prototypes -----------------------------------------------*/
static uint32_t Motor_Task_PidPeriodTicks(void);
static void Motor_Task_Pid(void *argument);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 按 speed_loop_hz 计算任务周期（tick）
 * @return {uint32_t} 周期，至少 1 个 tick
 * @author: lkc
 */
static uint32_t Motor_Task_PidPeriodTicks(void)
{
    uint32_t tick_hz;
    float loop_hz;
    uint32_t period;

    tick_hz = osKernelGetTickFreq();
    if (tick_hz < 1U) {
        tick_hz = 1U;
    }
    loop_hz = motor.speed_loop_hz;
    if (loop_hz < 1.0f) {
        loop_hz = 1.0f;
    }
    period = (uint32_t)((float)tick_hz / loop_hz);
    if (period < 1U) {
        period = 1U;
    }
    return period;
}

/**
 * @description: foc pid 任务：睡眠到下一拍后运行速度 PI。
 * @param {void *} argument 未使用
 * @return {*}
 * @author: lkc
 */
static void Motor_Task_Pid(void *argument)
{
    uint32_t tick;
    uint32_t last_us;
    float dt;

    (void)argument;
    tick = osKernelGetTickCount();
    last_us = Hal_Motor_TimeUs();

    for (;;) {
        tick += Motor_Task_PidPeriodTicks();
        (void)osDelayUntil(tick);

        dt = Hal_Motor_TimeElapsedSec(last_us);
        last_us = Hal_Motor_TimeUs();
        if (dt < MOTOR_TASK_PID_DT_MIN_S) {
            dt = MOTOR_TASK_PID_DT_MIN_S;
        } else if (dt > MOTOR_TASK_PID_DT_MAX_S) {
            dt = MOTOR_TASK_PID_DT_MAX_S;
        }

        Motor_SpeedLoop_Run(&motor, dt);
        /* 位置环尚未实现；有编码器位置伺服时与速度环互斥，同 VESC pid_thread。 */
    }
}

/**
 * @description: 启动速度/位置外环任务，只创建一次
 * @return {*}
 * @author: lkc
 */
void Motor_Task_Start(void)
{
    if (s_pid_thread != NULL) {
        return;
    }
    s_pid_thread = osThreadNew(Motor_Task_Pid, NULL, &s_pid_attr);
    if (s_pid_thread == NULL) {
        PRINTF("foc pid task create failed\r\n");
    }
}

#ifdef __cplusplus
}
#endif
