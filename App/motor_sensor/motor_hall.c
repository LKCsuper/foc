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
#include "motor_foc.h"
#include "hal_motor.h"
#include "hal_sensor.h"
#include "motor.h"
#include "motor_hall_angle.h"
#include "motor_hall_calibration.h"
#include "motor_crit.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct {
    volatile bool active;
    volatile bool abort;
    float theta_rad;   /* 标定期间强制电角度 (rad) */
    float id_target;   /* D 轴目标电流 (A) */
    float iq_target;   /* Q 轴目标电流 (A) */
} motor_hall_detect_ctx_t;
/* Private variables ---------------------------------------------------------*/
static motor_hall_detect_ctx_t s_hall_detect;
/* Private function prototypes -----------------------------------------------*/
static void Motor_Hall_DetectCtxReset(void);
static void Motor_Hall_TableToMotor(const uint8_t *hall_table, MOTOR_PARAM_S *m);
static void Motor_Hall_DetectFinish(MOTOR_PARAM_S *m);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 清除霍尔标定运行时上下文（active/abort/角度与电流目标）
 * @return {void} 无
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
 * @description: 把 uint8 霍尔表写入 MOTOR_PARAM_S（255 → MOTOR_HALL_TABLE_INVALID）
 * @param {const uint8_t} *hall_table  标定输出表，不可为空
 * @param {MOTOR_PARAM_S} *m           运行时参数，不可为空
 * @return {void} 无
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
 * @description: 标定结束：清上下文、请求停机并关闭 PWM
 * @param {MOTOR_PARAM_S} *m  运行时参数，不可为空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Hall_DetectFinish(MOTOR_PARAM_S *m)
{
    Motor_Hall_DetectCtxReset();
    /* 标定可能改写同一张表，令默认估算器重新执行结构校验并清除旧速度。 */
    Motor_Hall_AngleResetDefault();
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
 * @return {bool} true 表示标定中，motor_it 走专用电流环
 * @author: lkc
 */
bool Motor_Hall_DetectIsActive(void)
{
    return s_hall_detect.active;
}

/**
 * @description: 中止正在进行的霍尔标定（olstop 或故障时调用）
 * @return {void} 无
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

    if (!s_hall_detect.active || s_hall_detect.abort) return false;

    *theta_rad = s_hall_detect.theta_rad;
    *id_target = s_hall_detect.id_target;
    *iq_target = s_hall_detect.iq_target;
    return true;
}

/**
 * @description: 开环转一圈，标定霍尔状态与电角度对应表
 * @detail: 兼容任务包装：纯 Calibration 状态机推进角度，ISR 读取 d/q 目标。
 *          最长 120 s；终态先停机，再验证并应用表；取消/故障/超时保留旧表。
 *          结果写入 hall_table；成功时同步写入 MOTOR_PARAM_S.hall_table
 * @param {float} current        对齐/拖动电流 (A)；对齐用 Id，扫角用 Iq（I/F）
 * @param {uint8_t} *hall_table  输出 8 项霍尔表，255 表示无效扇区
 * @param {bool} *result         true 表示标定成功（6 个有效扇区）
 * @return {int} 故障码，FAULT_CODE_NONE 表示无故障
 * @author: lkc
 */
int Motor_Hall_Detect(float current, uint8_t *hall_table, bool *result)
{
    MOTOR_PARAM_S *m = &motor;
    motor_hall_calibration_t calibration;
    int8_t candidate[MOTOR_HALL_TABLE_SIZE];
    int fault = FAULT_CODE_NONE;
    int hall;
    int z = 0;
    unsigned i;
    uint32_t key;
    uint32_t last_us;
    uint32_t elapsed_ms = 0U;
    uint32_t remainder_us = 0U;

    if (result != NULL) *result = false;
    if (hall_table == NULL || result == NULL) return FAULT_CODE_NONE;
    if (!Motor_Hall_CalibrationStart(&calibration, current, 0U, 120000U)) return FAULT_CODE_NONE;
    if (Motor_Hall_DetectIsActive()) return FAULT_CODE_NONE;
    if (Motor_Protect_IsFaultLatched()) {
        printf("HALL DETECT blocked by fault: %s — run olfault clear\r\n",
               Motor_Protect_GetFaultName(Motor_Protect_GetFault()));
        return Motor_Protect_GetFault();
    }
    printf("HALL DETECT I/F align+Iq=%.2fA slow crawl\r\n", (double)current);
    Motor_Protect_ClearFault();
    (void)Motor_ClearRuntimeControl(m);
    key = Motor_Crit_Enter();
    Motor_Hall_DetectCtxReset();
    m->ol_enable = true;
    m->ol_target_rpm = 1.0f;
    s_hall_detect.active = true;
    Motor_Crit_Exit(key);
    m->lastHallABC = 0U;
    last_us = Hal_Motor_TimeUs();
    Hal_Motor_PwmStart();

    while (Motor_Hall_CalibrationIsActive(&calibration)) {
        uint32_t now_us;
        uint32_t delta_us;
        /* 等待仅存在于兼容任务包装；裸机直接调用 CalibrationStart/Step。 */
        osDelay(pdMS_TO_TICKS(1U));
        now_us = Hal_Motor_TimeUs();
        delta_us = now_us - last_us;
        last_us = now_us;
        elapsed_ms += delta_us / 1000U;
        remainder_us += delta_us % 1000U;
        elapsed_ms += remainder_us / 1000U;
        remainder_us %= 1000U;
        hall = Motor_Hall_Read(false, m->hall_extra_samples);
        Motor_Hall_CalibrationStep(&calibration, elapsed_ms, hall, s_hall_detect.abort,
                                   Motor_Protect_IsFaultLatched());
        /* 目标整体交接，避免 ISR 读到跨阶段的角度/电流组合。终态立即撤销输出权。 */
        key = Motor_Crit_Enter();
        s_hall_detect.theta_rad = calibration.theta_rad;
        s_hall_detect.id_target = calibration.id_target;
        s_hall_detect.iq_target = calibration.iq_target;
        if (!Motor_Hall_CalibrationIsActive(&calibration)) {
            /* 禁用命令与释放标定权一起发布，防止 ISR 在间隙回到普通运行路径。 */
            m->ol_enable = false;
            s_hall_detect.active = false;
        }
        Motor_Crit_Exit(key);
        if (calibration.sampled_forward) {
            m->curHallABC = (uint8_t)hall;
            if (m->curHallABC != m->lastHallABC && z < 6) {
                m->lastHallABC = m->curHallABC;
                m->hallABC[z] = (uint8_t)hall;
                printf("diff [%d] [%d] [%d] [%d]\r\n", z, m->hallABC[z], m->curHallABC, hall);
                ++z;
            }
        }
    }
    /* 在结果检查和打印前停机；失败、取消及超时不覆盖已存在的有效表。 */
    Motor_Hall_DetectFinish(m);
    if (Motor_Protect_IsFaultLatched() || calibration.stage == MOTOR_HALL_CAL_FAULT) {
        fault = Motor_Protect_GetFault();
        printf("HALL DETECT aborted by fault: %s\r\n", Motor_Protect_GetFaultName(fault));
    } else if (calibration.stage == MOTOR_HALL_CAL_DONE) {
        memcpy(hall_table, calibration.table, sizeof(calibration.table));
        for (i = 0; i < MOTOR_HALL_TABLE_SIZE; ++i) candidate[i] = (int8_t)calibration.table[i];
        *result = Motor_Hall_TableReady(candidate);
        if (*result) {
            Motor_Hall_TableToMotor(hall_table, m);
        } else {
            printf("HALL DETECT fail: need 6 distinct sectors. table/samples:\r\n");
            for (i = 0; i < MOTOR_HALL_TABLE_SIZE; ++i)
                printf("  [%u] tab=%u n=%u\r\n", i, (unsigned)hall_table[i],
                       (unsigned)calibration.samples[i]);
        }
    } else if (calibration.stage == MOTOR_HALL_CAL_TIMEOUT) {
        printf("HALL DETECT aborted: timeout\r\n");
    } else if (calibration.stage == MOTOR_HALL_CAL_INVALID) {
        printf("HALL DETECT aborted: invalid hall sample\r\n");
    } else {
        printf("HALL DETECT aborted\r\n");
    }
    return fault;
}

#ifdef __cplusplus
}
#endif
