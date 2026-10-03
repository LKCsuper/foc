/*
 * @Description: FOC 控制结果到三相 PWM 的唯一输出边界
 * @Author: lkc
 */
#ifndef MOTOR_OUTPUT_H
#define MOTOR_OUTPUT_H
#ifdef __cplusplus
extern "C" {
#endif

#include "motor_control.h"
#include "motor_param.h"

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
 * @detail: 编码器与 Hall 的有感 I/F 使用已验证的 VESC SVM，其余模式保留 svm_hqd。
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
