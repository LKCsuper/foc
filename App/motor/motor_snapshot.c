/*
 * @Description: 电机状态帧捕获；ISR 发布双缓冲，任务无锁读取
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "common_assert.h"
#include "motor.h"
#include "motor_feedback.h"
#include "motor_speed.h"
#include "motor_foc_isr.h"
#include "motor_crit.h"
#include "seqlock.h"
#if defined(__CC_ARM)
#pragma O2
#endif
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SEQLOCK_DECLARE_CHANNEL(motor_status_channel_t, motor_status_t);
static motor_status_channel_t s_motor_status_channel;
static volatile bool s_motor_status_initialized;
/* Private function prototypes -----------------------------------------------*/
static void Motor_StatusCapture(const MOTOR_PARAM_S *motor, motor_status_t *snap);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 从电机对象拷贝运行、反馈与故障字段到状态快照。
 * @param {const MOTOR_PARAM_S *} motor 电机实例
 * @param {motor_status_t *} snap 输出状态快照
 * @return {void} 无
 */
static void Motor_StatusCapture(const MOTOR_PARAM_S *motor, motor_status_t *snap)
{
    memset(snap, 0, sizeof(*snap));
    snap->enable = motor->ol_enable;
    snap->mode = motor->ol_mode;
    snap->target_rpm = motor->ol_target_rpm;
    snap->current_rpm = motor->ol_current_rpm;
    snap->ia = motor->ia;
    snap->ib = motor->ib;
    snap->ic = motor->ic;
    memcpy(snap->current_adc, motor->currentAdc, sizeof(snap->current_adc));
    memcpy(snap->current_offset, motor->currentOffset, sizeof(snap->current_offset));
    snap->id = motor->Id;
    snap->iq = motor->Iq;
    snap->id_target = motor->id_target;
    snap->iq_target = motor->iq_target;
    snap->v_bus = motor->v_bus;
    snap->theta = motor->theta;
    snap->duty1 = motor->duty1;
    snap->duty2 = motor->duty2;
    snap->duty3 = motor->duty3;
    snap->vd_int = motor->ol_vd_int;
    snap->vq_int = motor->ol_vq_int;
    snap->speed_enable = motor->speed_enable;
    Motor_Feedback_SampleDefault(motor, motor->ol_theta, &snap->feedback);
    snap->aligning = Motor_OpenLoop_IsAligning();
    snap->align_elapsed_s = Motor_OpenLoop_GetAlignElapsed();
    snap->fault = Motor_Protect_GetFault();
    snap->fault_latched = Motor_Protect_IsFaultLatched();
}

/**
 * @description: 在 ISR 捕获一拍运行快照并发布到 seqlock 双缓冲。
 * @detail: 不阻塞、不关中断；首帧前初始化通道。
 * @param {const MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @return {void} 无
 */
void Motor_StatusSnapshotPublishFromIsr(const MOTOR_PARAM_S *motor)
{
    motor_status_t snap;

    COMMON_ASSERT(motor != NULL);
    if (!s_motor_status_initialized) {
        SEQLOCK_INIT(&s_motor_status_channel);
        s_motor_status_initialized = true;
    }
    Motor_StatusCapture(motor, &snap);
    SEQLOCK_PUBLISH(&s_motor_status_channel, &snap);
}

/**
 * @description: 获取一致的只读运行快照。
 * @detail: 优先从 seqlock latch 双缓冲读取，不关中断。首帧尚未发布时，
 *          仅为上电早期/主机测试保留一次短临界区现场捕获。
 * @param {const MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {motor_status_t *} status 输出快照，不可为空
 * @return {motor_result_t} MOTOR_OK 或 MOTOR_ERR_NULL
 * @author: lkc
 */
motor_result_t Motor_GetStatus(const MOTOR_PARAM_S *motor, motor_status_t *status)
{
    uint32_t primask;
    motor_status_t snap;
    motor_speed_snapshot_t speed_snap;
    bool speed_valid;

    if (motor == NULL || status == NULL) return MOTOR_ERR_NULL;

    if (!s_motor_status_initialized ||
        !SEQLOCK_TRY_READ(&s_motor_status_channel, &snap, 3U)) {
        primask = Motor_Crit_Enter();
        Motor_StatusCapture(motor, &snap);
        Motor_Crit_Exit(primask);
    }

    primask = Motor_Crit_Enter();
    speed_valid = Motor_SpeedLoop_GetSnapshot(&speed_snap);
    Motor_Crit_Exit(primask);
    if (speed_valid) {
        if (snap.speed_enable) snap.current_rpm = speed_snap.target_rpm;
        snap.measured_rpm = speed_snap.measured_rpm;
        snap.speed_error = speed_snap.error;
        snap.speed_iq = speed_snap.iq_target;
    }
    *status = snap;
    return MOTOR_OK;
}

#ifdef __cplusplus
}
#endif
