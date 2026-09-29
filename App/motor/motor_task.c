/*
 * @Description: 电机后台任务：速度/位置外环（对齐 VESC foc pid 线程）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 电流环、SVPWM 和编码器角度处理仍在 Motor_DmaInt。
 *   2) 速度 PI 写出 Iq 目标供 ISR 读取；MT6701 总线由独立任务采集。
 *   3) 速度环周期跟 speed_loop_hz；传感器采集周期固定，不与速度环配置耦合。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_TASK_PID_STACK_WORDS 512U
#define MOTOR_TASK_ENCODER_STACK_WORDS 256U
#define MOTOR_TASK_PID_DT_MIN_S    (5.0e-5f)
#define MOTOR_TASK_PID_DT_MAX_S    (0.05f)
#define MOTOR_TASK_ENCODER_RETRY_MS  (20U)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static StackType_t s_pid_stack[MOTOR_TASK_PID_STACK_WORDS];
static StaticTask_t s_pid_tcb;
static StackType_t s_encoder_stack[MOTOR_TASK_ENCODER_STACK_WORDS];
static StaticTask_t s_encoder_tcb;
static const osThreadAttr_t s_pid_attr = {
    .name = "foc pid",
    /* 与 Shell 同级轮转；FOC 电流环仍由 ADC ISR 保证实时性。 */
    .priority = PRI_CMD,
    .cb_mem = &s_pid_tcb,
    .cb_size = sizeof(s_pid_tcb),
    .stack_mem = &s_pid_stack,
    .stack_size = sizeof(s_pid_stack),
};
static osThreadId_t s_pid_thread;
static const osThreadAttr_t s_encoder_attr = {
    .name = "mt6701",
    /* SSI 采样不可长期饿死命令/停机任务。 */
    .priority = PRI_CMD,
    .cb_mem = &s_encoder_tcb,
    .cb_size = sizeof(s_encoder_tcb),
    .stack_mem = &s_encoder_stack,
    .stack_size = sizeof(s_encoder_stack),
};
static osThreadId_t s_encoder_thread;
/* Private function prototypes -----------------------------------------------*/
static uint32_t Motor_Task_PidPeriodTicks(void);
static uint32_t Motor_Task_EncoderPeriodTicks(void);
static void Motor_Task_Pid(void *argument);
static void Motor_Task_Encoder(void *argument);
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
    uint32_t now;
    uint32_t period;
    uint32_t last_us;
    float dt;

    (void)argument;
    tick = osKernelGetTickCount();
    last_us = Hal_Motor_TimeUs();

    for (;;) {
        now = osKernelGetTickCount();
        period = Motor_Task_PidPeriodTicks();
        if ((int32_t)(now - tick) >= 0) {
            tick = now;
        }
        tick += period;
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

static uint32_t Motor_Task_EncoderPeriodTicks(void)
{
    uint32_t tick_hz = osKernelGetTickFreq();
    uint32_t period = tick_hz / MOTOR_MT6701_POLL_HZ;

    return (period < 1U) ? 1U : period;
}

/**
 * @description: MT6701 固定周期采集任务。
 * @detail: I2C/SSI 事务不进入 ADC/FOC ISR，也不随速度环频率配置改变。
 * @param {void *} argument 未使用
 * @return {*}
 */
static void Motor_Task_Encoder(void *argument)
{
    uint32_t tick;
    uint32_t now;
    uint32_t period;

    (void)argument;
    tick = osKernelGetTickCount();
    for (;;) {
        now = osKernelGetTickCount();
        period = Motor_Task_EncoderPeriodTicks();
        if ((int32_t)(now - tick) >= 0) {
            tick = now;
        }
        tick += period;
        (void)osDelayUntil(tick);
        if (!Motor_MT6701_Poll()) {
            /* 设备未应答时退避，禁止高优先级任务连续超时饿死 Shell。 */
            (void)osDelay(pdMS_TO_TICKS(MOTOR_TASK_ENCODER_RETRY_MS));
            tick = osKernelGetTickCount();
        }
    }
}

/**
 * @description: 启动速度/位置外环任务，只创建一次
 * @return {*}
 * @author: lkc
 */
void Motor_Task_Start(void)
{
    if (s_pid_thread == NULL) {
        s_pid_thread = osThreadNew(Motor_Task_Pid, NULL, &s_pid_attr);
        if (s_pid_thread == NULL) {
            PRINTF("foc pid task create failed\r\n");
        }
    }
    if (s_encoder_thread == NULL) {
        s_encoder_thread = osThreadNew(Motor_Task_Encoder, NULL, &s_encoder_attr);
        if (s_encoder_thread == NULL) {
            PRINTF("mt6701 task create failed\r\n");
        }
    }
}

#ifdef __cplusplus
}
#endif
