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

float Motor_Sensorless_GetControlAngle(float open_loop_angle)
{
    (void)open_loop_angle;
    return s_sensorless_angle;
}

bool Motor_Sensorless_IsTakeoverComplete(void)
{
    return s_takeover;
}

bool Motor_Encoder_IsZeroValid(void)
{
    return s_enc_zero;
}

float Motor_Encoder_GetElecAngle(void)
{
    return s_enc_elec;
}

float Motor_Encoder_GetMechRpm(void)
{
    return s_enc_rpm;
}

bool Motor_Encoder_IsSpeedValid(void)
{
    return s_enc_speed_valid;
}

static bool s_hall_angle_valid;
static bool s_hall_speed_valid;
static float s_hall_theta = 2.0f;
static float s_hall_rpm = 50.0f;

bool Motor_Hall_AngleIsAngleValid(void)
{
    return s_hall_angle_valid;
}

float Motor_Hall_AngleGetTheta(void)
{
    return s_hall_theta;
}

float Motor_Hall_AngleGetMechRpm(void)
{
    return s_hall_rpm;
}

bool Motor_Hall_AngleIsSpeedValid(void)
{
    return s_hall_speed_valid;
}

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
