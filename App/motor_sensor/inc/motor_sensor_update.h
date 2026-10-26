/*
 * @Description: 控制拍中的编码器/霍尔反馈更新
 * @Author: lkc
 */
#ifndef MOTOR_SENSOR_UPDATE_H
#define MOTOR_SENSOR_UPDATE_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "motor_feedback.h"
#include "motor_param.h"

/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 按当前反馈源更新编码器或霍尔运行状态。
 * @detail: 在 FOC 控制拍调用，不写 PWM。编码器失联时按现有条件请求故障。
 *          霍尔按 MOTOR_HALL_ADC_DIVIDER 分频，dt 累加后交给角度估计；切走霍尔源时清累计。
 *          motor 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor           电机实例，不可为空
 * @param {motor_feedback_source_t} source  当前请求的反馈源
 * @param {float} dt_s                      本控制拍时间 (s)
 * @return {void} 无
 * @author: lkc
 */
void Motor_SensorUpdate_Step(MOTOR_PARAM_S *motor, motor_feedback_source_t source, float dt_s);

#ifdef __cplusplus
}
#endif
#endif
