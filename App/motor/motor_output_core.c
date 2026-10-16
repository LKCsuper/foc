/* 输出仲裁及电压重建：标准C，无HAL或全局电机依赖。 */
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "motor_output_core.h"
#if defined(__CC_ARM)
#pragma O2
#endif

void Motor_OutputCore_SubtractDeadtime(float ia, float ib, float ic, float v_bus,
                                         float fraction, float *alpha, float *beta)
{
    int sa = (ia > 0.0f) - (ia < 0.0f);
    int sb = (ib > 0.0f) - (ib < 0.0f);
    int sc = (ic > 0.0f) - (ic < 0.0f);
    float scale = (2.0f / 3.0f) * v_bus * fraction;
    /* 符号组合只有小整数；合并后再转换，固定除数改为倒数乘法。 */
    *alpha -= (float)(2 * sa - sb - sc) * (1.0f / 3.0f) * scale;
    *beta -= (float)(sb - sc) * 0.577350269f * scale;
}

bool Motor_OutputCore_EstimateVoltage(const motor_output_sample_t *motor, uint32_t period, float v_bus,
                                 float deadtime_fraction, float *v_alpha, float *v_beta)
{
    float scale;

    if (v_alpha != NULL) *v_alpha = 0.0f;
    if (v_beta != NULL) *v_beta = 0.0f;
    if (motor == NULL || v_alpha == NULL || v_beta == NULL || period == 0U ||
        !isfinite(v_bus) || v_bus <= 0.0f || !isfinite(deadtime_fraction) ||
        deadtime_fraction < 0.0f || deadtime_fraction >= 0.5f ||
        motor->duty1 > period || motor->duty2 > period || motor->duty3 > period) {
        return false;
    }
    if (deadtime_fraction > 0.0f &&
        (!isfinite(motor->ia) || !isfinite(motor->ib) || !isfinite(motor->ic))) {
        return false;
    }
    scale = v_bus / (float)period;
    *v_alpha = (2.0f * motor->duty1 - motor->duty2 - motor->duty3) * (1.0f / 3.0f) * scale;
    *v_beta = ((float)motor->duty2 - motor->duty3) * 0.577350269f * scale;
    if (deadtime_fraction > 0.0f) {
        /* VESC调制以1.5/Vbus归一化；补偿转回SI电压须乘2/3*Vbus。
         * 零流方向不可判定，按0处理，避免静止零电流凭空生成补偿。 */
        Motor_OutputCore_SubtractDeadtime(motor->ia, motor->ib, motor->ic, v_bus,
                                     deadtime_fraction, v_alpha, v_beta);
    }
    return true;
}

motor_control_result_t Motor_Control_ArbitrateOutput(bool fault_latched, const motor_control_result_t *proposed)
{
    motor_control_result_t none;

    if (fault_latched) {
        memset(&none, 0, sizeof(none));
        none.action = MOTOR_CONTROL_ACTION_SAFE;
        return none;
    }
    if (proposed == NULL) {
        memset(&none, 0, sizeof(none));
        none.action = MOTOR_CONTROL_ACTION_NONE;
        return none;
    }
    return *proposed;
}
