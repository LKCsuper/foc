/*
 * @Description: 电机参数整定（电流环 Kp/Ki 等）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "motor_cal.h"
#include "motor_param.h"
#include "motor_math.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 按 R/L 与电流环时间常数整定 FOC 电流环 Kp/Ki 与观测器增益
 * @param {MOTOR_PARAM_S} *m  运行时参数；R/L/磁链取自 sensorless_* 字段
 * @param {float} tc          电流环时间常数 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Calc_FocCurrentKpKi(MOTOR_PARAM_S *m, float tc)
{
    float r;
    float l;
    float lambda;
    float bw;
    float kp;
    float ki;
    float gain;

    if (m == NULL || tc <= 0.0f) {
        return;
    }

    r = m->sensorless_motor_r;
    l = m->sensorless_motor_l;
    lambda = m->sensorless_flux_linkage;

    bw = 1.0f / tc;
    kp = l * bw;
    ki = r * bw;
    gain = 1.0e-3f / SQ(lambda);

    m->ol_current_kp = kp;
    m->ol_current_ki = ki;
    m->sensorless_observer_gain = gain * 1.0e6f;
}

/**
 * @description: 计算电机电气速度环 Kp/Ki 的预留接口
 * @detail: 当前为空实现（占位），待补充整定公式。
 * @return {*}
 * @author: lkc
 */
void MOTOR_Cal_Electric_KP_KI(void)
{
    return;
}

#ifdef __cplusplus
}
#endif
