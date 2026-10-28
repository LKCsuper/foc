/*
 * @Description: FOC 安全输出、反 Park、SVPWM 与板级相序提交
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "common_assert.h"
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include "motor_output.h"
#include "motor_feedback.h"
#include "motor_foc.h"
#include "motor_hall.h"
#include "motor_math.h"
#include "motor_sensorless.h"
#include "motor_timing.h"
#include "hal_motor.h"
#if defined(__CC_ARM)
#pragma O3
#endif
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#if defined(__CC_ARM)
/* 输出路径保持独立源码与公共接口，内联同文件的重建和提交编排。 */
#define MOTOR_OUTPUT_INLINE __forceinline
#else
#define MOTOR_OUTPUT_INLINE
#endif
/* Private variables ---------------------------------------------------------*/
/* 仅诊断 RAM 参数：有效死区秒数乘 PWM 频率，不修改硬件死区或 PWM 命令。 */
static float s_observer_deadtime_fraction;
static float s_observer_current_a;
static float s_observer_current_b;
static float s_observer_current_c;
/* Private function prototypes -----------------------------------------------*/
static void Motor_OutputCommit(MOTOR_PARAM_S *motor);
static void Motor_OutputInversePark(const MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float *v_alpha,
                                    float *v_beta);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 反 Park，同实例本拍 Park 角度原码相同时复用其正余弦。
 * @detail: 角度不同或该实例尚未做过 Park 时按 theta 直接计算，结果与 inverse_park 一致。
 *          调用者保证 motor 与输出指针非空。
 * @param {const MOTOR_PARAM_S *} motor 电机对象，读取 park_* 字段
 * @param {float} vd                    D 轴电压 (V)
 * @param {float} vq                    Q 轴电压 (V)
 * @param {float} theta_rad             电角度 (rad)
 * @param {float *} v_alpha             输出 α 轴电压 (V)
 * @param {float *} v_beta              输出 β 轴电压 (V)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_OutputInversePark(const MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float *v_alpha,
                                    float *v_beta)
{
    if (motor->park_sincos_valid && utils_float_same_bits(motor->park_theta, theta_rad)) {
        inverse_park_sincos(vd, vq, motor->park_sin, motor->park_cos, v_alpha, v_beta);
    } else {
        inverse_park(vd, vq, theta_rad, v_alpha, v_beta);
    }
}

/**
 * @description: 设置仅 RAM 的观测电压死区补偿比例。
 * @detail: 应用任务调用，不阻塞，不改 PWM 比较值或硬件死区。比例为有效死区秒数乘 PWM 频率，
 *          合法范围 [0, 0.02]，0 表示禁用。非法值保持原设置。写入成功时清零观测电流滤波。
 * @param {float} fraction 补偿比例，无量纲
 * @return {bool} 写入成功返回 true；非法值返回 false
 * @author: lkc
 */
bool Motor_OutputSetObserverDeadtime(float fraction)
{
    if (!isfinite(fraction) || fraction < 0.0f || fraction > 0.02f) return false;
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
 *          非零时使用 motor 的 ia/ib/ic (A)，不做电流滤波，不含管压降。motor 为空、输出为空、
 *          period 为 0、母线或比例非法、或比较值超过 period 时返回 false，并把非空输出清零。
 *          估计结果不能直接当作 PWM 命令。
 * @param {const MOTOR_PARAM_S *} motor 电机对象，读取 duty 与相电流 (A)；空则失败
 * @param {uint32_t} period             PWM 周期计数，须大于 0
 * @param {float} v_bus                 母线电压 (V)，须为有限正值
 * @param {float} deadtime_fraction     死区补偿比例，无量纲，范围 [0, 0.5)
 * @param {float *} v_alpha             输出 α 轴电压 (V)；空则失败
 * @param {float *} v_beta              输出 β 轴电压 (V)；空则失败
 * @return {bool} 估计成功返回 true
 * @author: lkc
 */
MOTOR_OUTPUT_INLINE bool Motor_OutputEstimateVoltage(const MOTOR_PARAM_S *motor, uint32_t period, float v_bus,
                                 float deadtime_fraction, float *v_alpha, float *v_beta)
{
    motor_output_sample_t sample;
    if (motor == NULL) return Motor_OutputCore_EstimateVoltage(NULL, period, v_bus, deadtime_fraction, v_alpha, v_beta);
    sample.duty1 = motor->duty1; sample.duty2 = motor->duty2; sample.duty3 = motor->duty3;
    sample.ia = motor->ia; sample.ib = motor->ib; sample.ic = motor->ic;
    return Motor_OutputCore_EstimateVoltage(&sample, period, v_bus, deadtime_fraction, v_alpha, v_beta);
}

/**
 * @description: 输出三相中点占空比并提交 PWM。
 * @detail: 中点不等于关桥，也不保证无转矩。同时清零无感本拍应用电压记录和观测电流滤波。
 *          motor 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 duty1/2/3（计数）
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputSafe(MOTOR_PARAM_S *motor)
{
    uint32_t mid;

    COMMON_ASSERT(motor != NULL);
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
 * @description: 按 BM75/HD75 的 A-C-B 相序提交 PWM。
 * @detail: 主动电压路径共用此出口。比较值顺序为 duty1、duty3、duty2。
 *          调用者保证 motor 非空。不改占空比数值。
 * @param {MOTOR_PARAM_S *} motor 电机对象，读取 duty1/2/3（计数）
 * @return {void} 无
 * @author: lkc
 */
static void Motor_OutputCommit(MOTOR_PARAM_S *motor)
{
    Hal_Motor_PwmSetDuty(motor->duty1, motor->duty3, motor->duty2);
    Hal_Motor_PwmCommit();
}

/**
 * @description: V/F 输出，使用 HQD 调制。
 * @detail: 反 Park 后把原符号 α/β 记入无感应用电压，再用 svm_hqd 写三相比较值，按 A-C-B 提交。
 *          不重建观测电压。motor 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 duty1/2/3（计数）
 * @param {float} vd              D 轴电压 (V)
 * @param {float} vq              Q 轴电压 (V)
 * @param {float} theta_rad       电角度 (rad)
 * @param {float} v_bus           母线电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputVoltageVf(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus)
{
    float v_alpha;
    float v_beta;
    uint32_t sector;
    uint32_t period;

    COMMON_ASSERT(motor != NULL);
    period = Hal_Motor_PwmPeriodCounts();
    Motor_OutputInversePark(motor, vd, vq, theta_rad, &v_alpha, &v_beta);
    Motor_Sensorless_SetAppliedVoltage(v_alpha, v_beta);
    svm_hqd(v_alpha, v_beta, v_bus, period, &motor->duty1, &motor->duty2, &motor->duty3, &sector);
    Motor_OutputCommit(motor);
}

/**
 * @description: 有感 I/F 输出，使用 VESC 线性 SVM。
 * @detail: 反 Park 原符号 α/β 记入应用电压；SVM 使用取反后的 α/β。不重建观测电压，按 A-C-B 提交。
 *          霍尔是否改走 V/F 由上层选择。motor 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 duty1/2/3（计数）
 * @param {float} vd              D 轴电压 (V)
 * @param {float} vq              Q 轴电压 (V)
 * @param {float} theta_rad       电角度 (rad)
 * @param {float} v_bus           母线电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputVoltageSensoredIf(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus)
{
    float v_alpha;
    float v_beta;
    uint32_t sector;
    uint32_t period;

    COMMON_ASSERT(motor != NULL);
    period = Hal_Motor_PwmPeriodCounts();
    Motor_OutputInversePark(motor, vd, vq, theta_rad, &v_alpha, &v_beta);
    Motor_Sensorless_SetAppliedVoltage(v_alpha, v_beta);
    foc_svm_from_voltage(-v_alpha, -v_beta, v_bus, MOTOR_FOC_SVM_MAX_DUTY, period, &motor->duty1,
                         &motor->duty2, &motor->duty3, &sector);
    Motor_OutputCommit(motor);
}

/**
 * @description: 无感及开环 I/F 输出，使用 VESC 线性 SVM 并重建观测电压。
 * @detail: SVM 使用反 Park 取反后的 α/β，并按 A-C-B 提交。PWM 周期计数为 0 时不重建。
 *          否则先用无死区重建电压做调制误差诊断，再把本拍平均电压留给下一拍观测。
 *          死区只在 RAM 比例大于 0 且三相电流有限时，按逻辑 ABC 电流的一阶滤波估计，
 *          系数 0.01，不含管压降，不改已算出的比较值。
 *          无感反馈且未授权接管时不重建：观测器不积分，比较值保持本拍 SVM 的结果。
 *          motor 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 duty1/2/3（计数）
 * @param {float} vd              D 轴电压 (V)
 * @param {float} vq              Q 轴电压 (V)
 * @param {float} theta_rad       电角度 (rad)
 * @param {float} v_bus           母线电压 (V)
 * @return {void} 无
 * @author: lkc
 */
MOTOR_OUTPUT_INLINE void Motor_OutputVoltageSensorlessIf(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad,
                                                        float v_bus)
{
    float v_alpha;
    float v_beta;
    uint32_t sector;
    uint32_t period;

    COMMON_ASSERT(motor != NULL);
    period = Hal_Motor_PwmPeriodCounts();
    Motor_OutputInversePark(motor, vd, vq, theta_rad, &v_alpha, &v_beta);
    /* 暂存与调制输入同符号的指令，供 SetModulatedVoltage 比较；之后用重建电压覆盖。 */
    Motor_Sensorless_SetAppliedVoltage(-v_alpha, -v_beta);
    foc_svm_from_voltage(-v_alpha, -v_beta, v_bus, MOTOR_FOC_SVM_MAX_DUTY, period, &motor->duty1,
                         &motor->duty2, &motor->duty3, &sector);
    if (period != 0U &&
        (Motor_Feedback_GetSourceDefault() != MOTOR_FEEDBACK_SRC_SENSORLESS || motor->sensorless_enable)) {
        /* 使用逻辑ABC比较值重建平均电压；交换到物理ACB前计算，不含死区/管压降。 */
        float reconstructed_a;
        float reconstructed_b;
        /* 与采样、保护同一无感 I/F 运行窗口；停机和开环不计入。时间含在 OUTPUT 内。 */
        bool profile = Motor_Timing_ProfileDetailedIsActive() && motor->ol_enable && motor->sensorless_enable &&
                       motor->ol_mode == MOTOR_OL_MODE_IF &&
                       Motor_Feedback_GetSourceDefault() == MOTOR_FEEDBACK_SRC_SENSORLESS;
        uint32_t stage_start = profile ? Motor_Timing_ProfileTimeUs() : 0U;
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
        if (profile) Motor_Timing_ProfileRecord(MOTOR_TIMING_VOLTAGE_REBUILD, stage_start);
    }
    Motor_OutputCommit(motor);
}

/**
 * @description: 旧电压输出接口，按模式和反馈源分派。
 * @detail: 正常 FOC 由上层选明确路径。非 I/F 走 V/F。I/F 且无感或开环走无感 I/F。
 *          I/F 且编码器，或霍尔且启用 VESC SVM，走有感 I/F。其余走 V/F。
 *          本函数不复制调制。motor 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 duty1/2/3（计数）
 * @param {float} vd              D 轴电压 (V)
 * @param {float} vq              Q 轴电压 (V)
 * @param {float} theta_rad       电角度 (rad)
 * @param {float} v_bus           母线电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputVoltage(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus)
{
    motor_feedback_source_t source;
    COMMON_ASSERT(motor != NULL);
    source = Motor_Feedback_GetSourceDefault();
    if (motor->ol_mode != MOTOR_OL_MODE_IF) {
        Motor_OutputVoltageVf(motor, vd, vq, theta_rad, v_bus);
    } else if (source == MOTOR_FEEDBACK_SRC_SENSORLESS || source == MOTOR_FEEDBACK_SRC_OPENLOOP) {
        Motor_OutputVoltageSensorlessIf(motor, vd, vq, theta_rad, v_bus);
    } else if (source == MOTOR_FEEDBACK_SRC_ENCODER ||
               (source == MOTOR_FEEDBACK_SRC_HALL && MOTOR_HALL_USE_VESC_SVM)) {
        Motor_OutputVoltageSensoredIf(motor, vd, vq, theta_rad, v_bus);
    } else {
        Motor_OutputVoltageVf(motor, vd, vq, theta_rad, v_bus);
    }
}

/**
 * @description: 参数检测用的线性调制入口。
 * @detail: 两条路径都用 VESC 线性 SVM，不走 HQD。I/F 且无感或开环时保留无感观测电压重建，
 *          其余走有感 I/F。不改硬件死区或相序。指令电压不是实测端电压。
 *          motor 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 duty1/2/3（计数）
 * @param {float} vd              D 轴电压 (V)
 * @param {float} vq              Q 轴电压 (V)
 * @param {float} theta_rad       电角度 (rad)
 * @param {float} v_bus           母线电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputVoltageLinear(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus)
{
    motor_feedback_source_t source;
    COMMON_ASSERT(motor != NULL);
    source = Motor_Feedback_GetSourceDefault();
    if (motor->ol_mode == MOTOR_OL_MODE_IF &&
        (source == MOTOR_FEEDBACK_SRC_SENSORLESS || source == MOTOR_FEEDBACK_SRC_OPENLOOP)) {
        Motor_OutputVoltageSensorlessIf(motor, vd, vq, theta_rad, v_bus);
    } else {
        Motor_OutputVoltageSensoredIf(motor, vd, vq, theta_rad, v_bus);
    }
}

/**
 * @description: 将完成最终故障仲裁的结果按显式路径提交。
 * @detail: SAFE 不看路径，直接输出中点。NONE 不更新 PWM。OUTPUT_DQ 按路径走 V/F、
 *          有感 I/F 或无感 I/F；非法路径改为 SAFE。本函数不检查故障锁存，
 *          调用者不得在故障时提交 OUTPUT_DQ。
 *          motor 与 result 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor                 电机对象
 * @param {float} v_bus                           母线电压 (V)
 * @param {const motor_control_result_t *} result 仲裁后的输出请求，不可为空
 * @param {motor_output_path_t} path             主动电压路径
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputApplyControlResultForPath(MOTOR_PARAM_S *motor, float v_bus,
                                         const motor_control_result_t *result, motor_output_path_t path)
{
    COMMON_ASSERT(motor != NULL && result != NULL);
    if (result->action == MOTOR_CONTROL_ACTION_SAFE) {
        Motor_OutputSafe(motor);
        return;
    }
    if (result->action != MOTOR_CONTROL_ACTION_OUTPUT_DQ) return;

    switch (path) {
    case MOTOR_OUTPUT_PATH_VF:
        Motor_OutputVoltageVf(motor, result->vd, result->vq, result->theta, v_bus);
        break;
    case MOTOR_OUTPUT_PATH_SENSORED_IF:
        Motor_OutputVoltageSensoredIf(motor, result->vd, result->vq, result->theta, v_bus);
        break;
    case MOTOR_OUTPUT_PATH_SENSORLESS_IF:
        Motor_OutputVoltageSensorlessIf(motor, result->vd, result->vq, result->theta, v_bus);
        break;
    default:
        Motor_OutputSafe(motor);
        break;
    }
}

/**
 * @description: 将已经仲裁的控制结果应用到 PWM。
 * @detail: 调用前须已经过 Motor_Control_ArbitrateOutput。SAFE 走中点，OUTPUT_DQ 经旧接口分派后反 Park。
 *          NONE 不更新 PWM。本函数不检查故障锁存，故障时不得传入 OUTPUT_DQ。
 *          motor 与 result 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor                 电机对象
 * @param {float} v_bus                           母线电压 (V)
 * @param {const motor_control_result_t *} result 仲裁后的输出请求，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputApplyControlResult(MOTOR_PARAM_S *motor, float v_bus, const motor_control_result_t *result)
{
    COMMON_ASSERT(motor != NULL && result != NULL);
    if (result->action == MOTOR_CONTROL_ACTION_SAFE) {
        Motor_OutputSafe(motor);
    } else if (result->action == MOTOR_CONTROL_ACTION_OUTPUT_DQ) {
        Motor_OutputVoltage(motor, result->vd, result->vq, result->theta, v_bus);
    }
}

#ifdef __cplusplus
}
#endif
