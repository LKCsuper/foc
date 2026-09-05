/*
 * @Description: 三相霍尔读取与霍尔表标定
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "motor_hall.h"
#include "motor_param.h"
#include "motor_protect.h"
#include "motor_math.h"
#include "hal_motor.h"
#include "hal_sensor.h"
#include "motor.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct {
    volatile bool active;
    volatile bool abort;
    float theta_rad;   /* 标定期间强制电角度 (rad) */
    float id_target;   /* D 轴目标电流 (A) */
    float iq_target;   /* Q 轴目标电流 (A) */
} motor_hall_detect_ctx_t;
/* Private define ------------------------------------------------------------*/
#define MOTOR_HALL_DETECT_ALIGN_STEPS   (1000U)
#define MOTOR_HALL_DETECT_SWEEP_LOOPS    (3U)
#define MOTOR_HALL_DETECT_DEG_STEPS      (360U)
#define MOTOR_HALL_DETECT_STEP_DELAY_MS  (5U)
#define MOTOR_HALL_DETECT_MIN_SAMPLES    (30U)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_hall_detect_ctx_t s_hall_detect;
/* Private function prototypes -----------------------------------------------*/
static void Motor_Hall_DetectCtxReset(void);
static bool Motor_Hall_DetectShouldStop(void);
static bool Motor_Hall_DetectWait(float theta_rad, float id_target, uint32_t delay_ms);
static void Motor_Hall_StoreTableEntry(uint8_t *hall_table, int hall,
                                       float sin_acc, float cos_acc, int samples);
static void Motor_Hall_TableToMotor(const uint8_t *hall_table, MOTOR_PARAM_S *m);
static void Motor_Hall_DetectFinish(MOTOR_PARAM_S *m);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 清除霍尔标定运行时上下文
 * @return {*}
 * @author: lkc
 */
static void Motor_Hall_DetectCtxReset(void)
{
    s_hall_detect.active = false;
    s_hall_detect.abort = false;
    s_hall_detect.theta_rad = 0.0f;
    s_hall_detect.id_target = 0.0f;
    s_hall_detect.iq_target = 0.0f;
}

/**
 * @description: 标定任务是否应退出
 * @return {bool} true 表示用户中止或保护已锁存故障
 * @author: lkc
 */
static bool Motor_Hall_DetectShouldStop(void)
{
    return s_hall_detect.abort || Motor_Protect_IsFaultLatched();
}

/**
 * @description: 下发本拍角度/电流并等待 ISR 执行
 * @param {float} theta_rad   强制电角度 (rad)
 * @param {float} id_target   D 轴目标电流 (A)
 * @param {uint32_t} delay_ms 等待时间 (ms)
 * @return {bool} true 可以继续；false 应中止
 * @author: lkc
 */
static bool Motor_Hall_DetectWait(float theta_rad, float id_target, uint32_t delay_ms)
{
    s_hall_detect.theta_rad = theta_rad;
    s_hall_detect.id_target = id_target;
    osDelay(pdMS_TO_TICKS(delay_ms));
    return !Motor_Hall_DetectShouldStop();
}

/**
 * @description: 根据某霍尔状态的 sin/cos 累加值计算表项
 * @param {uint8_t} *hall_table  8 项输出表
 * @param {int} hall             霍尔状态 0~7
 * @param {float} sin_acc        sin 累加
 * @param {float} cos_acc        cos 累加
 * @param {int} samples          该状态采样次数
 * @return {*}
 * @author: lkc
 */
static void Motor_Hall_StoreTableEntry(uint8_t *hall_table, int hall,
                                       float sin_acc, float cos_acc, int samples)
{
    float ang;

    if (samples > (int)MOTOR_HALL_DETECT_MIN_SAMPLES) {
        ang = RAD2DEG_f(atan2f(sin_acc, cos_acc));
        utils_norm_angle(&ang);
        hall_table[hall] = (uint8_t)(ang * 200.0f / 360.0f);
    } else {
        hall_table[hall] = 255U;
    }
}

/**
 * @description: 把 uint8 霍尔表写入 MOTOR_PARAM_S（255 → -1）
 * @param {uint8_t} *hall_table  标定输出表
 * @param {MOTOR_PARAM_S} *m     运行时参数
 * @return {*}
 * @author: lkc
 */
static void Motor_Hall_TableToMotor(const uint8_t *hall_table, MOTOR_PARAM_S *m)
{
    int i;

    for (i = 0; i < (int)MOTOR_HALL_TABLE_SIZE; i++) {
        if (hall_table[i] == 255U) {
            m->hall_table[i] = MOTOR_HALL_TABLE_INVALID;
        } else {
            m->hall_table[i] = (int8_t)hall_table[i];
        }
    }
}

/**
 * @description: 标定结束：清上下文、停机并关闭 PWM
 * @param {MOTOR_PARAM_S} *m  运行时参数
 * @return {*}
 * @author: lkc
 */
static void Motor_Hall_DetectFinish(MOTOR_PARAM_S *m)
{
    Motor_Hall_DetectCtxReset();
    (void)Motor_RequestStop(m);
    Hal_Motor_PwmStop();
}

/**
 * @description: 读取三相霍尔状态（bit0=A, bit1=B, bit2=C）
 * @param {bool} is_second_motor  true 时读第二路霍尔口
 * @param {int} samples           额外滤波次数，0 表示只采一次
 * @return {int} 霍尔状态 0~7
 * @author: lkc
 */
int Motor_Hall_Read(bool is_second_motor, int samples)
{
    return Hal_Hall_ReadRaw(is_second_motor, samples);
}

/**
 * @description: 霍尔标定是否占用 FOC 中断路径
 * @return {bool} true 表示标定中
 * @author: lkc
 */
bool Motor_Hall_DetectIsActive(void)
{
    return s_hall_detect.active;
}

/**
 * @description: 中止正在进行的霍尔标定
 * @return {*}
 * @author: lkc
 */
void Motor_Hall_DetectAbort(void)
{
    if (s_hall_detect.active) {
        s_hall_detect.abort = true;
    }
}

/**
 * @description: 霍尔标定期间 ISR 读取本周期电角度与 d/q 电流目标
 * @param {MOTOR_PARAM_S} *m         电机运行时参数
 * @param {float} *theta_rad         输出电角度 (rad)
 * @param {float} *id_target         输出 D 轴目标电流 (A)
 * @param {float} *iq_target         输出 Q 轴目标电流 (A)
 * @return {bool} true 表示标定中且目标有效
 * @author: lkc
 */
bool Motor_Hall_DetectGetTargets(MOTOR_PARAM_S *m, float *theta_rad,
                                 float *id_target, float *iq_target)
{
    (void)m;

    if (!s_hall_detect.active || s_hall_detect.abort) {
        return false;
    }

    *theta_rad = s_hall_detect.theta_rad;
    *id_target = s_hall_detect.id_target;
    *iq_target = s_hall_detect.iq_target;
    return true;
}

/**
 * @description: 开环转一圈，标定霍尔状态与电角度对应表
 * @detail: 任务上下文推进角度；ISR 通过 DetectGetTargets 输出 D 轴电流与固定角。
 * @param {float} current        对齐电流 (A)
 * @param {uint8_t} *hall_table  输出 8 项霍尔表
 * @param {bool} *result         true 表示标定成功
 * @return {int} 故障码
 * @author: lkc
 */
int Motor_Hall_Detect(float current, uint8_t *hall_table, bool *result)
{
    MOTOR_PARAM_S *m = &motor;
    float sin_hall[MOTOR_HALL_TABLE_SIZE];
    float cos_hall[MOTOR_HALL_TABLE_SIZE];
    int hall_iterations[MOTOR_HALL_TABLE_SIZE];
    int fault = FAULT_CODE_NONE;
    int hall;
    int fails;
    int i;
    int j;
    int z;
    float s;
    float c;
    uint32_t step;
    bool stopped = false;

    if (hall_table == NULL || result == NULL || current <= 0.0f) {
        return FAULT_CODE_NONE;
    }

    printf("HALL DETECT\r\n");
    memset(sin_hall, 0, sizeof(sin_hall));
    memset(cos_hall, 0, sizeof(cos_hall));
    memset(hall_iterations, 0, sizeof(hall_iterations));
    *result = false;

    m->ol_enable = true;
    m->ol_target_rpm = 1.0f;
    (void)Motor_ClearRuntimeControl(m);
    s_hall_detect.abort = false;
    s_hall_detect.iq_target = 0.0f;
    s_hall_detect.theta_rad = 0.0f;
    s_hall_detect.id_target = 0.0f;
    s_hall_detect.active = true;
    Hal_Motor_PwmStart();

    for (step = 0; step < MOTOR_HALL_DETECT_ALIGN_STEPS; step++) {
        if (!Motor_Hall_DetectWait(0.0f,
                                   (float)step * current / (float)MOTOR_HALL_DETECT_ALIGN_STEPS,
                                   1U)) {
            stopped = true;
            break;
        }
    }

    z = 0;
    m->lastHallABC = 0U;
    for (i = 0; !stopped && i < (int)MOTOR_HALL_DETECT_SWEEP_LOOPS; i++) {
        for (j = 0; j <= (int)MOTOR_HALL_DETECT_DEG_STEPS; j++) {
            if (!Motor_Hall_DetectWait(DEG2RAD_f((float)j), current,
                                       MOTOR_HALL_DETECT_STEP_DELAY_MS)) {
                stopped = true;
                break;
            }

            hall = Motor_Hall_Read(false, m->hall_extra_samples);
            m->curHallABC = (uint8_t)hall;
            if ((m->curHallABC != m->lastHallABC) && (z < 6)) {
                m->lastHallABC = m->curHallABC;
                m->hallABC[z] = (uint8_t)hall;
                printf("diff [%d] [%d] [%d] [%d]\r\n", z, m->hallABC[z], m->curHallABC, hall);
                z++;
            }

            arm_sin_cos_f32(s_hall_detect.theta_rad, &s, &c);
            sin_hall[hall] += s;
            cos_hall[hall] += c;
            hall_iterations[hall]++;
        }
    }

    for (i = 0; !stopped && i < (int)MOTOR_HALL_DETECT_SWEEP_LOOPS; i++) {
        for (j = (int)MOTOR_HALL_DETECT_DEG_STEPS; j >= 0; j--) {
            if (!Motor_Hall_DetectWait(DEG2RAD_f((float)j), current,
                                       MOTOR_HALL_DETECT_STEP_DELAY_MS)) {
                stopped = true;
                break;
            }

            hall = Motor_Hall_Read(false, m->hall_extra_samples);
            arm_sin_cos_f32(s_hall_detect.theta_rad, &s, &c);
            sin_hall[hall] += s;
            cos_hall[hall] += c;
            hall_iterations[hall]++;
        }
    }

    if (Motor_Protect_IsFaultLatched()) {
        fault = Motor_Protect_GetFault();
        printf("HALL DETECT aborted by fault: %s\r\n", Motor_Protect_GetFaultName(fault));
    } else if (s_hall_detect.abort) {
        printf("HALL DETECT aborted\r\n");
    } else {
        fails = 0;
        for (i = 0; i < (int)MOTOR_HALL_TABLE_SIZE; i++) {
            Motor_Hall_StoreTableEntry(hall_table, i, sin_hall[i], cos_hall[i], hall_iterations[i]);
            if (hall_table[i] == 255U) {
                fails++;
            }
        }
        *result = (fails == 2);
        if (*result) {
            Motor_Hall_TableToMotor(hall_table, m);
        }
    }

    Motor_Hall_DetectFinish(m);
    return fault;
}

#ifdef __cplusplus
}
#endif
