/*
 * @Description: FOC 控制结果到三相 PWM 的唯一输出边界
 * @Author: lkc
 */
#ifndef MOTOR_OUTPUT_H
#define MOTOR_OUTPUT_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "motor_control.h"
#include "motor_param.h"

/**
 * @description: 设置仅 RAM 的观测电压死区补偿比例。
 * @detail: 应用任务调用，不阻塞，不改 PWM 比较值或硬件死区。比例为有效死区秒数乘 PWM 频率，
 *          合法范围 [0, 0.02]，0 表示禁用。非法值保持原设置。写入成功时清零观测电流滤波。
 * @param {float} fraction 补偿比例，无量纲
 * @return {bool} 写入成功返回 true；非法值返回 false
 * @author: lkc
 */
bool Motor_OutputSetObserverDeadtime(float fraction);

/**
 * @description: 读取当前观测电压死区补偿比例。
 * @detail: 任意上下文可读，不阻塞，不访问硬件。
 * @return {float} 补偿比例，无量纲；未设置时为 0
 * @author: lkc
 */
float Motor_OutputGetObserverDeadtime(void);

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
bool Motor_OutputEstimateVoltage(const MOTOR_PARAM_S *motor, uint32_t period, float v_bus,
                                 float deadtime_fraction, float *v_alpha, float *v_beta);

/**
 * @description: 输出三相中点占空比并提交 PWM。
 * @detail: 中点不等于关桥，也不保证无转矩。同时清零无感本拍应用电压记录和观测电流滤波。
 *          motor 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 duty1/2/3（计数）
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputSafe(MOTOR_PARAM_S *motor);

/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 主动电压输出路径。
 * @detail: 由控制集成层选择。显式路径入口不读取控制模式或反馈源。
 */
typedef enum {
    MOTOR_OUTPUT_PATH_VF,
    MOTOR_OUTPUT_PATH_SENSORED_IF,
    MOTOR_OUTPUT_PATH_SENSORLESS_IF
} motor_output_path_t;

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
void Motor_OutputVoltageVf(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus);

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
void Motor_OutputVoltageSensoredIf(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus);

/**
 * @description: 无感及开环 I/F 输出，使用 VESC 线性 SVM 并重建观测电压。
 * @detail: SVM 使用反 Park 取反后的 α/β，并按 A-C-B 提交。PWM 周期计数为 0 时不重建。
 *          否则先用无死区重建电压做调制误差诊断，再把本拍平均电压留给下一拍观测。
 *          死区只在 RAM 比例大于 0 且三相电流有限时，按逻辑 ABC 电流的一阶滤波估计，
 *          系数 0.01，不含管压降，不改已算出的比较值。
 *          motor 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 duty1/2/3（计数）
 * @param {float} vd              D 轴电压 (V)
 * @param {float} vq              Q 轴电压 (V)
 * @param {float} theta_rad       电角度 (rad)
 * @param {float} v_bus           母线电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputVoltageSensorlessIf(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus);

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
                                         const motor_control_result_t *result, motor_output_path_t path);

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
void Motor_OutputVoltage(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus);

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
void Motor_OutputVoltageLinear(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus);

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
void Motor_OutputApplyControlResult(MOTOR_PARAM_S *motor, float v_bus, const motor_control_result_t *result);

#ifdef __cplusplus
}
#endif
#endif
