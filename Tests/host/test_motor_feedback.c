/*
 * @Description: 统一反馈选择主机测试
 * @Author: lkc
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_feedback.h"
#include "motor_param.h"

static float s_sensorless_angle = 1.23f;
static bool s_takeover;
static bool s_enc_zero;
static bool s_enc_speed_valid;
static float s_enc_elec = 4.0f;
static float s_enc_rpm = 100.0f;

/**
 * @description: 主机测试替身：获取默认实例本周期 Park 变换应使用的电角度。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} open_loop_angle 兼容参数，本实现不使用
 * @return {float} 测试记录值 s_sensorless_angle
 */
float Motor_Sensorless_GetControlAngle(float open_loop_angle)
{
    (void)open_loop_angle;
    return s_sensorless_angle;
}

/**
 * @description: 主机测试替身：判断默认实例开环角到观测角的接管是否全部完成。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_takeover
 */
bool Motor_Sensorless_IsTakeoverComplete(void)
{
    return s_takeover;
}

/**
 * @description: 主机测试替身：查询默认实例电角零点是否已标定
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_enc_zero
 */
bool Motor_Encoder_IsZeroValid(void)
{
    return s_enc_zero;
}

/**
 * @description: 主机测试替身：读取默认实例当前电角
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_enc_elec
 */
float Motor_Encoder_GetElecAngle(void)
{
    return s_enc_elec;
}

/**
 * @description: 主机测试替身：读取默认实例已更新的机械转速
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_enc_rpm
 */
float Motor_Encoder_GetMechRpm(void)
{
    return s_enc_rpm;
}

/**
 * @description: 主机测试替身：读取默认实例近零吸附前的机械转速。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_enc_rpm
 */
float Motor_Encoder_GetRawMechRpm(void)
{
    return s_enc_rpm;
}

/**
 * @description: 主机测试替身：读取默认实例距最近一次计数变化的时间。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0.0f
 */
float Motor_Encoder_GetEdgeAge(void)
{
    return 0.0f;
}

/**
 * @description: 主机测试替身：读取默认实例距最近一次 SSI/I2C 新帧的时间。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 0.0001f
 */
float Motor_Encoder_GetSampleAge(void)
{
    return 0.0001f;
}

/**
 * @description: 主机测试替身：查询默认实例是否处于信号故障。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Encoder_IsSignalFault(void)
{
    return false;
}

/**
 * @description: 主机测试替身：查询默认实例本拍转速是否有效
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_enc_speed_valid
 */
bool Motor_Encoder_IsSpeedValid(void)
{
    return s_enc_speed_valid;
}

static bool s_hall_angle_valid;
static bool s_hall_speed_valid;
static float s_hall_theta = 2.0f;
static float s_hall_rpm = 50.0f;
static float s_hall_raw_rpm = 55.0f;
static float s_hall_fast_rpm = 60.0f;
static float s_hall_age = 0.004f;

/**
 * @description: 主机测试替身：默认实例电角是否有效
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_hall_angle_valid
 */
bool Motor_Hall_AngleIsAngleValid(void)
{
    return s_hall_angle_valid;
}

/**
 * @description: 主机测试替身：默认实例电角 (rad)；angle_valid=false 时值勿单独使用
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_hall_theta
 */
float Motor_Hall_AngleGetTheta(void)
{
    return s_hall_theta;
}

/**
 * @description: 主机测试替身：默认实例机械转速 (RPM)
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_hall_rpm
 */
float Motor_Hall_AngleGetMechRpm(void)
{
    return s_hall_rpm;
}

/**
 * @description: 主机测试替身：读取默认实例最近单边沿原始机械转速。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_hall_raw_rpm
 */
float Motor_Hall_AngleGetRawMechRpm(void)
{
    return s_hall_raw_rpm;
}

/**
 * @description: 主机测试替身：读取默认实例 Hall-PLL 机械转速，供控制环使用。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_hall_fast_rpm
 */
float Motor_Hall_AngleGetFastMechRpm(void)
{
    return s_hall_fast_rpm;
}

/**
 * @description: 主机测试替身：读取默认实例距最近一次霍尔边沿的时间。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_hall_age
 */
float Motor_Hall_AngleGetEdgeAge(void)
{
    return s_hall_age;
}

/**
 * @description: 主机测试替身：读取默认实例最近一次有效霍尔状态。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {int} 测试记录值 3
 */
int Motor_Hall_AngleGetState(void)
{
    return 3;
}

/**
 * @description: 主机测试替身：默认实例复位
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Hall_AngleResetDefault(void)
{
}

/**
 * @description: 主机测试替身：默认实例转速是否有效
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_hall_speed_valid
 */
bool Motor_Hall_AngleIsSpeedValid(void)
{
    return s_hall_speed_valid;
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    motor_feedback_t fb;
    motor_feedback_sample_t sample;
    MOTOR_PARAM_S motor;

    memset(&motor, 0, sizeof(motor));
    motor.sensorless_enable = true;
    Motor_Feedback_Reset(&fb);

    /* 默认无感：采用观测角，接管前速度无效 */
    s_takeover = false;
    s_sensorless_angle = 2.5f;
    Motor_Feedback_Sample(&fb, &motor, 0.1f, &sample);
    assert(sample.used_source == MOTOR_FEEDBACK_SRC_SENSORLESS);
    assert(fabsf(sample.elec_angle_rad - 2.5f) < 1e-6f);
    assert(sample.angle_valid);
    assert(!sample.speed_valid);

    s_takeover = true;
    Motor_Feedback_Sample(&fb, &motor, 0.1f, &sample);
    assert(sample.speed_valid);

    /* 无感关闭 → 回退开环 */
    motor.sensorless_enable = false;
    Motor_Feedback_Sample(&fb, &motor, 0.7f, &sample);
    assert(sample.used_source == MOTOR_FEEDBACK_SRC_OPENLOOP);
    assert(fabsf(sample.elec_angle_rad - 0.7f) < 1e-6f);
    assert(!sample.speed_valid);

    /* 强制开环 */
    Motor_Feedback_SetSource(&fb, MOTOR_FEEDBACK_SRC_OPENLOOP);
    motor.sensorless_enable = true;
    Motor_Feedback_Sample(&fb, &motor, 0.3f, &sample);
    assert(sample.used_source == MOTOR_FEEDBACK_SRC_OPENLOOP);
    assert(fabsf(sample.elec_angle_rad - 0.3f) < 1e-6f);
    assert(sample.angle_valid);
    assert(!sample.speed_valid);

    /* 编码器未标定：角回退开环；速度可单独有效 */
    Motor_Feedback_SetSource(&fb, MOTOR_FEEDBACK_SRC_ENCODER);
    s_enc_zero = false;
    s_enc_speed_valid = true;
    s_enc_rpm = 120.0f;
    Motor_Feedback_Sample(&fb, &motor, 0.9f, &sample);
    assert(sample.used_source == MOTOR_FEEDBACK_SRC_OPENLOOP);
    assert(fabsf(sample.elec_angle_rad - 0.9f) < 1e-6f);
    assert(!sample.angle_valid);
    assert(sample.speed_valid);
    assert(fabsf(sample.mech_rpm - 120.0f) < 1e-6f);

    /* 编码器已标定：采用电角 */
    s_enc_zero = true;
    s_enc_elec = 1.57f;
    Motor_Feedback_Sample(&fb, &motor, 0.9f, &sample);
    assert(sample.used_source == MOTOR_FEEDBACK_SRC_ENCODER);
    assert(sample.angle_valid);
    assert(fabsf(sample.elec_angle_rad - 1.57f) < 1e-6f);

    /* 霍尔角有效 */
    Motor_Feedback_SetSource(&fb, MOTOR_FEEDBACK_SRC_HALL);
    s_hall_angle_valid = true;
    s_hall_speed_valid = true;
    s_hall_theta = 2.2f;
    s_hall_rpm = 80.0f;
    Motor_Feedback_Sample(&fb, &motor, 1.1f, &sample);
    assert(sample.used_source == MOTOR_FEEDBACK_SRC_HALL);
    assert(sample.angle_valid);
    assert(fabsf(sample.elec_angle_rad - 2.2f) < 1e-6f);
    assert(sample.speed_valid);
    assert(fabsf(sample.mech_rpm_raw - 55.0f) < 1e-6f);
    assert(fabsf(sample.mech_rpm_fast - 60.0f) < 1e-6f);
    assert(fabsf(sample.sample_age_s - 0.004f) < 1e-6f);
    assert(sample.sensor_state == 3);

    /* 霍尔无效：回退开环 */
    s_hall_angle_valid = false;
    s_hall_speed_valid = false;
    Motor_Feedback_Sample(&fb, &motor, 1.1f, &sample);
    assert(!sample.angle_valid);
    assert(!sample.speed_valid);
    assert(fabsf(sample.elec_angle_rad - 1.1f) < 1e-6f);

    puts("test_motor_feedback: pass");
    return 0;
}
