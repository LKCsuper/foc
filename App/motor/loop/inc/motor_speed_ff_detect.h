/* @Description: 空载速度电压前馈的线性拟合 */
#ifndef MOTOR_SPEED_FF_DETECT_H
#define MOTOR_SPEED_FF_DETECT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
/* Private typedef -----------------------------------------------------------*/
/** 线性拟合累加器：样本数与各阶累加和。 */
typedef struct {
    uint16_t count;                              /* 样本数 */
    float sum_x, sum_y, sum_xx, sum_xy, sum_yy;  /* 各阶累加和 */
} motor_speed_ff_fit_t;
/** 线性拟合结果：空载电压-转速前馈的斜率、截距与拟合误差。 */
typedef struct {
    float slope_v_per_rpm; /* 斜率 (V/RPM) */
    float boost_v;         /* 截距/启动 boost (V) */
    float rms_error_v;     /* 拟合均方根误差 (V) */
    uint16_t points;       /* 参与拟合的样本数 */
} motor_speed_ff_result_t;

/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 清零线性拟合累加器。
 * @param {motor_speed_ff_fit_t *} fit  累加器，不可为空
 * @return {void} 无
 */
void Motor_SpeedFfFit_Init(motor_speed_ff_fit_t *fit);

/**
 * @description: 累加一个 (|rpm|, |vq|) 样本。
 * @detail: |rpm| 小于 1 或 rpm 与 vq 异号时拒绝，计数到上限后拒绝。
 * @param {motor_speed_ff_fit_t *} fit  累加器，不可为空
 * @param {float} rpm  转速 (RPM)
 * @param {float} vq   q 轴电压 (V)
 * @return {bool} 已累加返回 true
 */
bool Motor_SpeedFfFit_Add(motor_speed_ff_fit_t *fit, float rpm, float vq);

/**
 * @description: 最小二乘求解斜率、截距与拟合误差。
 * @detail: 样本不足 4 个、拟合非法或斜率/截距超出合理范围时失败。
 * @param {const motor_speed_ff_fit_t *} fit  累加器，不可为空
 * @param {motor_speed_ff_result_t *} result  输出结果，不可为空
 * @return {bool} 求解成功返回 true
 */
bool Motor_SpeedFfFit_Solve(const motor_speed_ff_fit_t *fit, motor_speed_ff_result_t *result);
#ifdef __cplusplus
}
#endif
#endif
