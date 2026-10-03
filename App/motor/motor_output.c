/*
 * @Description: FOC 安全输出、反 Park、SVPWM 与板级相序提交
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "motor_output.h"
#include "motor_feedback.h"
#include "motor_foc.h"
#include "motor_hall.h"
#include "motor_math.h"
#include "motor_sensorless.h"
#include "hal_motor.h"

/**
 * @description: 输出三相中点占空比并提交 PWM。
 * @detail: 中点不等于关桥，也不保证无转矩。同时清零无感本拍应用电压记录。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 duty1/2/3
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputSafe(MOTOR_PARAM_S *motor)
{
    uint32_t mid;

    if (motor == NULL) {
        return;
    }
    mid = Hal_Motor_PwmPeriodCounts() / 2U;
    motor->duty1 = mid;
    motor->duty2 = mid;
    motor->duty3 = mid;
    Hal_Motor_PwmSetDuty(motor->duty1, motor->duty2, motor->duty3);
    Hal_Motor_PwmCommit();
    Motor_Sensorless_SetAppliedVoltage(0.0f, 0.0f);
}

/**
 * @description: 反 Park 加调制，将 vd/vq 落到三相 PWM。
 * @detail: 编码器与 Hall 的有感 I/F 使用已验证的 VESC SVM，其余模式保留 svm_hqd。
 *          比较值按当前板 A-C-B 相序提交。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 duty1/2/3
 * @param {float} vd            D 轴电压 (V)
 * @param {float} vq            Q 轴电压 (V)
 * @param {float} theta_rad     电角度 (rad)
 * @param {float} v_bus         母线电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputVoltage(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus)
{
    float v_alpha;
    float v_beta;
    uint32_t sector;
    uint32_t period;
    motor_feedback_source_t feedback_source;
    bool sensored_if;

    if (motor == NULL) {
        return;
    }
    period = Hal_Motor_PwmPeriodCounts();
    inverse_park(vd, vq, theta_rad, &v_alpha, &v_beta);
    Motor_Sensorless_SetAppliedVoltage(v_alpha, v_beta);
    feedback_source = Motor_Feedback_GetSourceDefault();
    sensored_if = (motor->ol_mode == MOTOR_OL_MODE_IF) &&
                  (((feedback_source == MOTOR_FEEDBACK_SRC_HALL) && MOTOR_HALL_USE_VESC_SVM) ||
                   (feedback_source == MOTOR_FEEDBACK_SRC_ENCODER));
    /* 编码器与 Hall 共用同一 VESC 电压归一化，避免 svm_hqd 约 1/3 的实际调制增益。 */
    if (sensored_if) {
        foc_svm_from_voltage(-v_alpha, -v_beta, v_bus, MOTOR_FOC_SVM_MAX_DUTY, period, &motor->duty1,
                             &motor->duty2, &motor->duty3, &sector);
    } else {
        svm_hqd(v_alpha, v_beta, v_bus, period, &motor->duty1, &motor->duty2, &motor->duty3, &sector);
    }
    /* BM75/HD75 当前实机验证相序为 A-C-B。 */
    Hal_Motor_PwmSetDuty(motor->duty1, motor->duty3, motor->duty2);
    Hal_Motor_PwmCommit();
}

/**
 * @description: 将已经仲裁的控制结果应用到 PWM。
 * @detail: 调用前须已经过 Motor_Control_ArbitrateOutput。SAFE 走中点，OUTPUT_DQ 走反 Park。
 * @param {MOTOR_PARAM_S *} motor                  电机对象
 * @param {float} v_bus                            母线电压 (V)
 * @param {const motor_control_result_t *} result  仲裁后的输出请求
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputApplyControlResult(MOTOR_PARAM_S *motor, float v_bus, const motor_control_result_t *result)
{
    if (motor == NULL || result == NULL) {
        return;
    }
    if (result->action == MOTOR_CONTROL_ACTION_SAFE) {
        Motor_OutputSafe(motor);
    } else if (result->action == MOTOR_CONTROL_ACTION_OUTPUT_DQ) {
        Motor_OutputVoltage(motor, result->vd, result->vq, result->theta, v_bus);
    }
}

#ifdef __cplusplus
}
#endif
