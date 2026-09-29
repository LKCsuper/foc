/*
 * @Description: 电机状态帧捕获；临界区内只复制，不格式化、不发送串口
 * @Author: lkc
 */
#include <string.h>

#include "motor.h"
#include "motor_feedback.h"
#include "motor_speed.h"
#include "motor_it.h"

#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || (defined(__GNUC__) && defined(__arm__))
#include "stm32f4xx.h"
#define MOTOR_SNAPSHOT_HAS_IRQ_LOCK 1
#else
#define MOTOR_SNAPSHOT_HAS_IRQ_LOCK 0
#endif

static uint32_t motor_snapshot_lock(void)
{
#if MOTOR_SNAPSHOT_HAS_IRQ_LOCK
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#else
    return 0U;
#endif
}

static void motor_snapshot_unlock(uint32_t primask)
{
#if MOTOR_SNAPSHOT_HAS_IRQ_LOCK
    if (primask == 0U) {
        __enable_irq();
    }
#else
    (void)primask;
#endif
}

/**
 * @description: 获取一致的只读运行快照。
 * @detail: 短临界区拷贝样本与命令相关字段，再读取保护/速度环只读接口。
 *          可在任务上下文调用；不用于 ISR。
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

    if (motor == NULL || status == NULL) {
        return MOTOR_ERR_NULL;
    }

    memset(&snap, 0, sizeof(snap));
    primask = motor_snapshot_lock();
    snap.enable = motor->ol_enable;
    snap.mode = motor->ol_mode;
    snap.target_rpm = motor->ol_target_rpm;
    snap.current_rpm = motor->ol_current_rpm;
    snap.ia = motor->ia;
    snap.ib = motor->ib;
    snap.ic = motor->ic;
    memcpy(snap.current_adc, motor->currentAdc, sizeof(snap.current_adc));
    memcpy(snap.current_offset, motor->currentOffset, sizeof(snap.current_offset));
    snap.id = motor->Id;
    snap.iq = motor->Iq;
    snap.id_target = motor->id_target;
    snap.iq_target = motor->iq_target;
    snap.v_bus = motor->v_bus;
    snap.theta = motor->theta;
    snap.duty1 = motor->duty1;
    snap.duty2 = motor->duty2;
    snap.duty3 = motor->duty3;
    snap.vd_int = motor->ol_vd_int;
    snap.vq_int = motor->ol_vq_int;
    snap.speed_enable = motor->speed_enable;
    Motor_Feedback_SampleDefault(motor, motor->ol_theta, &snap.feedback);
    if (Motor_SpeedLoop_GetSnapshot(&speed_snap)) {
        snap.current_rpm = speed_snap.target_rpm;
        snap.measured_rpm = speed_snap.measured_rpm;
        snap.speed_error = speed_snap.error;
        snap.speed_iq = speed_snap.iq_target;
    }
    motor_snapshot_unlock(primask);

    /* 这些模块的诊断字段不参与速度三元组一致性，避免扩大关中断时间。 */
    snap.aligning = Motor_OpenLoop_IsAligning();
    snap.align_elapsed_s = Motor_OpenLoop_GetAlignElapsed();
    snap.fault = Motor_Protect_GetFault();
    snap.fault_latched = Motor_Protect_IsFaultLatched();

    *status = snap;
    return MOTOR_OK;
}
