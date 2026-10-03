/*
 * @Description: 快速 V/F 实机诊断路径
 * @Author: lkc
 */
#ifndef MOTOR_FAST_VF_H
#define MOTOR_FAST_VF_H
#ifdef __cplusplus
extern "C" {
#endif

#include "motor_param.h"

/**
 * @description: 推进一次简化 V/F 诊断控制拍并提交电压。
 * @detail: 仅供历史实机基线验证，不代替正常 Motor_Control_Step 路径。
 *          在控制拍调用。频率或极对数无效时改走中点输出。motor 为空则直接返回。
 * @param {MOTOR_PARAM_S *} motor 电机实例，不可为空
 * @param {float} control_hz      实际控制频率 (Hz)
 * @return {void} 无
 * @author: lkc
 */
void Motor_FastVf_Step(MOTOR_PARAM_S *motor, float control_hz);

#ifdef __cplusplus
}
#endif
#endif
