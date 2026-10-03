/*
 * @Description: 保留历史实机基线行为的简化 V/F 诊断控制
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stddef.h>
#include "motor_fast_vf.h"
#include "motor_math.h"
#include "motor_output.h"

/**
 * @description: 推进一次简化 V/F 诊断控制拍并提交电压。
 * @detail: 仅供历史实机基线验证。Vd 固定为 0，Vq 取 ol_v_boost 并钳到 [0, ol_vq_limit]。
 *          频率或极对数无效时改走中点输出。
 * @param {MOTOR_PARAM_S *} motor 电机实例，不可为空
 * @param {float} control_hz      实际控制频率 (Hz)
 * @return {void} 无
 * @author: lkc
 */
void Motor_FastVf_Step(MOTOR_PARAM_S *motor, float control_hz)
{
    float rpm;
    float rpm_abs;
    float theta_step;
    float vq;

    if (motor == NULL) {
        return;
    }
    rpm = motor->ol_target_rpm;
    rpm_abs = fabsf(rpm);
    vq = motor->ol_v_boost;

    /* 低于实机可稳定拖动的转速时使用启动转速地板；保留目标方向。 */
    if (rpm_abs > 0.0f && rpm_abs < motor->ol_min_rpm) {
        rpm = (rpm < 0.0f) ? -motor->ol_min_rpm : motor->ol_min_rpm;
    }
    if (control_hz <= 1.0f || motor->ol_pole_pairs <= 0.0f) {
        Motor_OutputSafe(motor);
        return;
    }
    /* 电角步长 (rad/拍) = 机械角速度 (rad/s) × 极对数 / 控制频率 (Hz)。 */
    theta_step = RPM2RADPS_f(rpm) * motor->ol_pole_pairs / control_hz;

    if (vq < 0.0f) {
        vq = 0.0f;
    }
    if (vq > motor->ol_vq_limit) {
        vq = motor->ol_vq_limit;
    }

    motor->ol_current_rpm = rpm;
    motor->theta += theta_step;
    utils_norm_angle_rad(&motor->theta);
    motor->ol_theta = motor->theta;
    Motor_OutputVoltage(motor, 0.0f, vq, motor->theta, motor->v_bus);
}

#ifdef __cplusplus
}
#endif
