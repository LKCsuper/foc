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
    /* 速度/位置环必须抢占 Shell 的浮点格式化和串口排队，避免打印改变控制周期。 */
    .priority = PRI_SPEED,
    .cb_mem = &s_pid_tcb,
    .cb_size = sizeof(s_pid_tcb),
    .stack_mem = &s_pid_stack,
    .stack_size = sizeof(s_pid_stack),
};
static osThreadId_t s_pid_thread;
static const osThreadAttr_t s_encoder_attr = {
    .name = "mt6701",
    /* 编码器反馈优先于 Shell；每拍事务有界并主动延时，不会持续占用 CPU。 */
    .priority = PRI_SPEED,
    .cb_mem = &s_encoder_tcb,
    .cb_size = sizeof(s_encoder_tcb),
    .stack_mem = &s_encoder_stack,
    .stack_size = sizeof(s_encoder_stack),
};
static osThreadId_t s_encoder_thread;
/* Private function prototypes -----------------------------------------------*/
static uint32_t Motor_Task_PidPeriodTicks(void);
static uint32_t Motor_Task_EncoderPeriodTicks(void);
static void Motor_Task_DelayNextTick(uint32_t *tick, uint32_t period);
static void Motor_Task_Pid(void *argument);
static void Motor_Task_Encoder(void *argument);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 推进周期基准并休眠到下一拍。
 * @detail: 用有符号 tick 差判断是否错过本拍，计数回绕后仍成立。错过则把基准拉到 now，避免连续补拍。
 * @param {uint32_t *} tick    周期基准（tick），读改写，不可为空
 * @param {uint32_t} period    本拍周期（tick），调用方保证至少为 1
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Task_DelayNextTick(uint32_t *tick, uint32_t period)
{
    uint32_t now = osKernelGetTickCount();

    if ((int32_t)(now - *tick) >= 0) {
        *tick = now;
    }
    *tick += period;
    (void)osDelayUntil(*tick);
}

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
        Motor_Task_DelayNextTick(&tick, Motor_Task_PidPeriodTicks());

        dt = Hal_Motor_TimeElapsedSec(last_us);
        last_us = Hal_Motor_TimeUs();
        if (dt < MOTOR_TASK_PID_DT_MIN_S) {
            dt = MOTOR_TASK_PID_DT_MIN_S;
        } else if (dt > MOTOR_TASK_PID_DT_MAX_S) {
            dt = MOTOR_TASK_PID_DT_MAX_S;
        }

        Motor_Position_Run(dt);
        Motor_SpeedLoop_Run(&motor, dt);
    }
}

/**
 * @description: 按 MT6701 轮询频率计算采集周期。
 * @detail: MOTOR_MT6701_POLL_HZ 是编译期常量，tick 频率在内核启动后不变，任务里只算一次。
 * @return {uint32_t} 周期（tick），至少 1
 * @author: lkc
 */
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
    const uint32_t period = Motor_Task_EncoderPeriodTicks();

    (void)argument;
    tick = osKernelGetTickCount();
    for (;;) {
        motor_mt6701_status_t before;
        motor_mt6701_status_t after;

        Motor_Task_DelayNextTick(&tick, period);
        Motor_MT6701_GetStatus(&before);
        if (!Motor_MT6701_Poll()) {
            Motor_MT6701_GetStatus(&after);
            if (after.transfer_error_count != before.transfer_error_count) {
                /* 仅总线超时退避；CRC/状态坏帧须按原采样周期重试，避免单帧异常制造采样超时。 */
                (void)osDelay(pdMS_TO_TICKS(MOTOR_TASK_ENCODER_RETRY_MS));
                tick = osKernelGetTickCount();
            }
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
        Motor_Position_Init();
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
