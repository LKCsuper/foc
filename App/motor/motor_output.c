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
#include <math.h>
#include "motor_output.h"
#include "motor_feedback.h"
#include "motor_foc.h"
#include "motor_hall.h"
#include "motor_math.h"
#include "motor_sensorless.h"
#include "hal_motor.h"
#if defined(__CC_ARM)
#pragma O2
#endif

/* 仅诊断RAM参数：有效死区秒数×PWM频率，不修改硬件死区或PWM命令。 */
static float s_observer_deadtime_fraction;
static float s_observer_current_a;
static float s_observer_current_b;
static float s_observer_current_c;

/**
 * @description: 设置仅 RAM 的观测电压死区补偿比例。
 * @detail: 应用任务调用，不阻塞，不改 PWM 比较值或硬件死区。比例为有效死区秒数乘 PWM 频率，
 *          合法范围 [0, 0.02]，0 表示禁用。非法值保持原设置。写入成功时清零观测电流滤波。
 * @param {float} fraction 补偿比例，无量纲
 * @return {bool} 已写入返回 true
 * @author: lkc
 */
bool Motor_OutputSetObserverDeadtime(float fraction)
{
    if (!isfinite(fraction) || fraction < 0.0f || fraction > 0.02f) {
        return false;
    }
    s_observer_deadtime_fraction = fraction;
    s_observer_current_a = s_observer_current_b = s_observer_current_c = 0.0f;
    return true;
}

/**
 * @description: 读取当前观测电压死区补偿比例。
 * @detail: 任意上下文可读，不阻塞，不访问硬件。
 * @return {float} 补偿比例，无量纲；未设置时为 0
 * @author: lkc
 */
float Motor_OutputGetObserverDeadtime(void)
{
    return s_observer_deadtime_fraction;
}



/**
 * @description: 由逻辑相序的 ABC 比较值估计平均 α/β 电压。
 * @detail: 纯计算，不阻塞，不访问硬件。deadtime_fraction 为有效死区秒数乘 PWM 频率，0 表示不补偿。
 *          非零时使用 motor 的 ia/ib/ic (A)，不做电流滤波，不含管压降。参数非法时返回 false，
 *          并把非空输出清零。估计结果不能直接当作 PWM 命令。
 * @param {const MOTOR_PARAM_S *} motor 电机对象，读取 duty 与相电流 (A)
 * @param {uint32_t} period             PWM 周期计数，须大于 0
 * @param {float} v_bus                 母线电压 (V)，须为有限正值
 * @param {float} deadtime_fraction     死区补偿比例，无量纲，范围 [0, 0.5)
 * @param {float *} v_alpha             输出 α 轴电压 (V)，不可为空
 * @param {float *} v_beta              输出 β 轴电压 (V)，不可为空
 * @return {bool} 估计成功返回 true
 * @author: lkc
 */
bool Motor_OutputEstimateVoltage(const MOTOR_PARAM_S *motor, uint32_t period, float v_bus,
                                 float deadtime_fraction, float *v_alpha, float *v_beta)
{
    motor_output_sample_t sample;
    if (motor == NULL) {
        return Motor_OutputCore_EstimateVoltage(NULL, period, v_bus, deadtime_fraction, v_alpha, v_beta);
    }
    sample.duty1 = motor->duty1; sample.duty2 = motor->duty2; sample.duty3 = motor->duty3;
    sample.ia = motor->ia; sample.ib = motor->ib; sample.ic = motor->ic;
    return Motor_OutputCore_EstimateVoltage(&sample, period, v_bus, deadtime_fraction, v_alpha, v_beta);
}

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
    s_observer_current_a = s_observer_current_b = s_observer_current_c = 0.0f;
}

/**
 * @description: 反 Park 加调制，将 vd/vq 落到三相 PWM。
 * @detail: 有感 I/F 保持已验证的 VESC SVM。无感 I/F 也用同一套电压换算，观测器记录PWM平均电压估计。
 *          V/F 仍用 svm_hqd。比较值按当前板 A-C-B 相序提交。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 duty1/2/3
 * @param {float} vd            D 轴电压 (V)
 * @param {float} vq            Q 轴电压 (V)
 * @param {float} theta_rad     电角度 (rad)
 * @param {float} v_bus         母线电压 (V)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_OutputVoltageInternal(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus,
                                        bool force_linear_svm)
{
    float v_alpha;
    float v_beta;
    uint32_t sector;
    uint32_t period;
    motor_feedback_source_t feedback_source;
    bool sensored_if;
    bool sensorless_if;

    if (motor == NULL) {
        return;
    }
    period = Hal_Motor_PwmPeriodCounts();
    inverse_park(vd, vq, theta_rad, &v_alpha, &v_beta);
    feedback_source = Motor_Feedback_GetSourceDefault();
    /* 编码器与 Hall 的条件保持原样。 */
    sensored_if = (motor->ol_mode == MOTOR_OL_MODE_IF) &&
                  (((feedback_source == MOTOR_FEEDBACK_SRC_HALL) && MOTOR_HALL_USE_VESC_SVM) ||
                   (feedback_source == MOTOR_FEEDBACK_SRC_ENCODER));
    /* 无感拖转和观测器接管都走 I/F。电压必须和 VESC SVM 一致，否则磁链积分会放大。 */
    sensorless_if = (motor->ol_mode == MOTOR_OL_MODE_IF) &&
                    (feedback_source == MOTOR_FEEDBACK_SRC_SENSORLESS ||
                     feedback_source == MOTOR_FEEDBACK_SRC_OPENLOOP);
    if (sensorless_if) {
        Motor_Sensorless_SetAppliedVoltage(-v_alpha, -v_beta);
    } else {
        Motor_Sensorless_SetAppliedVoltage(v_alpha, v_beta);
    }
    if (force_linear_svm || sensored_if || sensorless_if) {
        foc_svm_from_voltage(-v_alpha, -v_beta, v_bus, MOTOR_FOC_SVM_MAX_DUTY, period, &motor->duty1,
                             &motor->duty2, &motor->duty3, &sector);
    } else {
        svm_hqd(v_alpha, v_beta, v_bus, period, &motor->duty1, &motor->duty2, &motor->duty3, &sector);
    }
    /* BM75/HD75 当前实机验证相序为 A-C-B。 */
    if (sensorless_if && period != 0U) {
        /* 使用逻辑ABC比较值重建平均电压；交换到物理ACB前计算，不含死区/管压降。 */
        float reconstructed_a;
        float reconstructed_b;
        /* 先重建理想调制电压，保持重建误差诊断独立于实验补偿。 */
        (void)Motor_OutputEstimateVoltage(motor, period, v_bus, 0.0f, &reconstructed_a, &reconstructed_b);
        Motor_Sensorless_SetModulatedVoltage(reconstructed_a, reconstructed_b);
        if (s_observer_deadtime_fraction > 0.0f && isfinite(motor->ia) &&
            isfinite(motor->ib) && isfinite(motor->ic)) {
            /* 固定FOC周期下约百Hz滤波；只用于补偿方向，不反馈电流环。
             * 使用逻辑ABC电流，在物理ACB交换前计算，与重建电压相同坐标系。 */
            s_observer_current_a += 0.01f * (motor->ia - s_observer_current_a);
            s_observer_current_b += 0.01f * (motor->ib - s_observer_current_b);
            s_observer_current_c += 0.01f * (motor->ic - s_observer_current_c);
            Motor_OutputCore_SubtractDeadtime(s_observer_current_a, s_observer_current_b,
                                         s_observer_current_c, v_bus, s_observer_deadtime_fraction,
                                         &reconstructed_a, &reconstructed_b);
        }
        /* 诊断先比较指令与调制结果，再将本拍PWM平均电压留给下一拍观测。
         * 死区只按显式RAM参数估计，不含管压降。 */
        Motor_Sensorless_SetAppliedVoltage(reconstructed_a, reconstructed_b);
    }
    Hal_Motor_PwmSetDuty(motor->duty1, motor->duty3, motor->duty2);
    Hal_Motor_PwmCommit();
}

/**
 * @description: 反 Park 加调制，将 vd/vq 落到三相 PWM。
 * @detail: 有感 I/F 保持已验证的 VESC SVM。无感 I/F 使用同一套电压换算。V/F 仍用 svm_hqd。
 *          比较值按当前板 A-C-B 相序提交。为空则直接返回。
 * @param {MOTOR_PARAM_S *} motor     电机对象，写入 duty1/2/3
 * @param {float} vd                  D 轴电压 (V)
 * @param {float} vq                  Q 轴电压 (V)
 * @param {float} theta_rad           电角度 (rad)
 * @param {float} v_bus               母线电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputVoltage(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus)
{
    Motor_OutputVoltageInternal(motor, vd, vq, theta_rad, v_bus, false);
}

/**
 * @description: 使用 VESC 线性 SVM 电压归一化输出 dq 电压。
 * @detail: 供需要线性电压归一化的诊断路径使用；不依赖当前控制模式或反馈源。
 *          实际端电压仍受死区与管压降影响，不能将指令电压当作实测注入电压。
 * @param {MOTOR_PARAM_S *} motor     电机对象，写入 duty1/2/3
 * @param {float} vd                  D 轴电压 (V)
 * @param {float} vq                  Q 轴电压 (V)
 * @param {float} theta_rad           电角度 (rad)
 * @param {float} v_bus               母线电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputVoltageLinear(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus)
{
    Motor_OutputVoltageInternal(motor, vd, vq, theta_rad, v_bus, true);
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
