/*
 * @Description: FOC 控制结果到三相 PWM 的唯一输出边界
 * @Author: lkc
 */
#ifndef MOTOR_OUTPUT_H
#define MOTOR_OUTPUT_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "motor_control.h"
#include "motor_param.h"

/**
 * @description: 设置仅 RAM 的观测电压死区补偿比例。
 * @detail: 应用任务调用，不阻塞，不改 PWM 比较值或硬件死区。比例为有效死区秒数乘 PWM 频率，
 *          合法范围 [0, 0.02]，0 表示禁用。非法值保持原设置。写入成功时清零观测电流滤波。
 * @param {float} fraction 补偿比例，无量纲
 * @return {bool} 已写入返回 true
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
                                 float deadtime_fraction, float *v_alpha, float *v_beta);

/**
 * @description: 输出三相中点占空比并提交 PWM。
 * @detail: 中点不等于关桥，也不保证无转矩。同时清零无感本拍应用电压记录。为空则直接返回。
 * @param {MOTOR_PARAM_S *} motor 电机对象，写入 duty1/2/3
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputSafe(MOTOR_PARAM_S *motor);

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
void Motor_OutputVoltage(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus);

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
void Motor_OutputVoltageLinear(MOTOR_PARAM_S *motor, float vd, float vq, float theta_rad, float v_bus);

/**
 * @description: 将已经仲裁的控制结果应用到 PWM。
 * @detail: 调用前须已经过 Motor_Control_ArbitrateOutput。SAFE 走中点，OUTPUT_DQ 走反 Park。
 *          故障锁存时不得传入 OUTPUT_DQ。motor 或 result 为空则直接返回。
 * @param {MOTOR_PARAM_S *} motor                  电机对象
 * @param {float} v_bus                            母线电压 (V)
 * @param {const motor_control_result_t *} result  仲裁后的输出请求
 * @return {void} 无
 * @author: lkc
 */
void Motor_OutputApplyControlResult(MOTOR_PARAM_S *motor, float v_bus, const motor_control_result_t *result);

#ifdef __cplusplus
}
#endif
#endif
