/* @Description: 空载速度电压前馈的线性拟合 */
#ifndef MOTOR_SPEED_FF_DETECT_H
#define MOTOR_SPEED_FF_DETECT_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint16_t count;
    float sum_x, sum_y, sum_xx, sum_xy, sum_yy;
} motor_speed_ff_fit_t;
typedef struct {
    float slope_v_per_rpm;
    float boost_v;
    float rms_error_v;
    uint16_t points;
} motor_speed_ff_result_t;
void Motor_SpeedFfFit_Init(motor_speed_ff_fit_t *fit);
bool Motor_SpeedFfFit_Add(motor_speed_ff_fit_t *fit, float rpm, float vq);
bool Motor_SpeedFfFit_Solve(const motor_speed_ff_fit_t *fit, motor_speed_ff_result_t *result);
#ifdef __cplusplus
}
#endif
#endif
