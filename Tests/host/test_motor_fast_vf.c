/* 快速 V/F：转速斜坡、电压输出与非法极对数走安全输出。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_fast_vf.h"

static unsigned s_safe_calls;
static unsigned s_voltage_calls;
static float s_vd;
static float s_vq;
static float s_theta;
static float s_vbus;

/**
 * @description: 主机测试替身：输出三相中点占空比并提交 PWM。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_OutputSafe(MOTOR_PARAM_S *motor)
{
    (void)motor;
    s_safe_calls++;
}
/**
 * @description: 主机测试替身：旧电压输出接口，按模式和反馈源分派。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @param {float} vd D 轴电压 (V)
 * @param {float} vq Q 轴电压 (V)
 * @param {float} theta 强制电角度 (rad)
 * @param {float} vbus 母线电压 (V)
 * @return {void} 无
 */
void Motor_OutputVoltage(MOTOR_PARAM_S *motor, float vd, float vq, float theta, float vbus)
{
    (void)motor;
    s_voltage_calls++;
    s_vd = vd;
    s_vq = vq;
    s_theta = theta;
    s_vbus = vbus;
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    MOTOR_PARAM_S motor;
    float expected;

    memset(&motor, 0, sizeof(motor));
    motor.ol_target_rpm = 20.0f;
    motor.ol_min_rpm = 100.0f;
    motor.ol_pole_pairs = 7.0f;
    motor.ol_v_boost = 8.0f;
    motor.ol_vq_limit = 5.0f;
    motor.v_bus = 24.0f;
    Motor_FastVf_Step(&motor, 1000.0f);
    expected = (100.0f * 2.0f * 3.14159265358979323846f / 60.0f) * 7.0f / 1000.0f;
    assert(s_voltage_calls == 1U && s_safe_calls == 0U);
    assert(fabsf(motor.ol_current_rpm - 100.0f) < 1.0e-6f);
    assert(fabsf(motor.theta - expected) < 1.0e-6f);
    assert(s_vd == 0.0f && s_vq == 5.0f && s_theta == motor.theta && s_vbus == 24.0f);

    motor.ol_target_rpm = -20.0f;
    motor.ol_v_boost = -1.0f;
    motor.theta = 0.0f;
    Motor_FastVf_Step(&motor, 1000.0f);
    assert(fabsf(motor.ol_current_rpm + 100.0f) < 1.0e-6f);
    assert(s_vq == 0.0f && fabsf(motor.theta + expected) < 1.0e-6f);

    Motor_FastVf_Step(&motor, 1.0f);
    assert(s_safe_calls == 1U && s_voltage_calls == 2U);
    motor.ol_pole_pairs = 0.0f;
    Motor_FastVf_Step(&motor, 1000.0f);
    assert(s_safe_calls == 2U);
    puts("test_motor_fast_vf: pass");
    return 0;
}
