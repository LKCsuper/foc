/*
 * @Description: 统一转子反馈选择（开环/无感/编码器/霍尔）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "motor_feedback.h"
#include "motor_sensorless.h"
#include "motor_encoder.h"
#include "motor_hall_angle.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_feedback_t s_feedback = {
    .source = MOTOR_FEEDBACK_SRC_SENSORLESS,
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 复位反馈选择为默认无感源。
 */
void Motor_Feedback_Reset(motor_feedback_t *fb)
{
    if (fb == NULL) {
        return;
    }
    fb->source = MOTOR_FEEDBACK_SRC_SENSORLESS;
}

/**
 * @description: 设置请求的反馈来源。
 */
void Motor_Feedback_SetSource(motor_feedback_t *fb, motor_feedback_source_t source)
{
    if (fb == NULL) {
        return;
    }
    fb->source = source;
}

/**
 * @description: 读取请求的反馈来源。
 */
motor_feedback_source_t Motor_Feedback_GetSource(const motor_feedback_t *fb)
{
    return fb == NULL ? MOTOR_FEEDBACK_SRC_OPENLOOP : fb->source;
}

/**
 * @description: 按请求源解析本拍电角与有效标志。
 */
void Motor_Feedback_Sample(motor_feedback_t *fb, const MOTOR_PARAM_S *motor, float open_loop_theta,
                           motor_feedback_sample_t *out)
{
    motor_feedback_source_t src;

    if (out == NULL) {
        return;
    }

    out->elec_angle_rad = open_loop_theta;
    out->mech_rpm = 0.0f;
    out->angle_valid = false;
    out->speed_valid = false;
    out->used_source = MOTOR_FEEDBACK_SRC_OPENLOOP;

    src = (fb != NULL) ? fb->source : MOTOR_FEEDBACK_SRC_OPENLOOP;

    switch (src) {
    case MOTOR_FEEDBACK_SRC_OPENLOOP:
        out->elec_angle_rad = open_loop_theta;
        out->angle_valid = true;
        out->speed_valid = false;
        out->used_source = MOTOR_FEEDBACK_SRC_OPENLOOP;
        break;

    case MOTOR_FEEDBACK_SRC_SENSORLESS:
        /*
         * 无感关闭时仍回退开环角，避免无角可用；速度有效仅在接管完成。
         * 速度环发布仍由 sensorless effects 负责，此处不伪造零速。
         */
        if (motor != NULL && !motor->sensorless_enable) {
            out->elec_angle_rad = open_loop_theta;
            out->angle_valid = true;
            out->speed_valid = false;
            out->used_source = MOTOR_FEEDBACK_SRC_OPENLOOP;
        } else {
            out->elec_angle_rad = Motor_Sensorless_GetControlAngle(open_loop_theta);
            out->angle_valid = true;
            out->speed_valid = Motor_Sensorless_IsTakeoverComplete();
            out->used_source = MOTOR_FEEDBACK_SRC_SENSORLESS;
        }
        break;

    case MOTOR_FEEDBACK_SRC_ENCODER:
        /*
         * 零点未标定：电角不可信，回退开环；速度仍可单独有效（相对运动）。
         * 调用方须先 Motor_Encoder_Update(dt)；此处不伪造有效零速。
         */
        (void)motor;
        if (Motor_Encoder_IsZeroValid()) {
            out->elec_angle_rad = Motor_Encoder_GetElecAngle();
            out->angle_valid = true;
            out->used_source = MOTOR_FEEDBACK_SRC_ENCODER;
        } else {
            out->elec_angle_rad = open_loop_theta;
            out->angle_valid = false;
            out->used_source = MOTOR_FEEDBACK_SRC_OPENLOOP;
        }
        out->mech_rpm = Motor_Encoder_GetMechRpm();
        out->speed_valid = Motor_Encoder_IsSpeedValid();
        break;

    case MOTOR_FEEDBACK_SRC_HALL:
        /*
         * 表未就绪或扇区非法时 AngleUpdate 已置 angle_valid=false。
         * 调用方须先更新霍尔角度运行时（motor_it 内 AngleUpdateDefault）。
         */
        (void)motor;
        if (Motor_Hall_AngleIsAngleValid()) {
            out->elec_angle_rad = Motor_Hall_AngleGetTheta();
            out->angle_valid = true;
            out->used_source = MOTOR_FEEDBACK_SRC_HALL;
        } else {
            out->elec_angle_rad = open_loop_theta;
            out->angle_valid = false;
            out->used_source = MOTOR_FEEDBACK_SRC_OPENLOOP;
        }
        out->mech_rpm = Motor_Hall_AngleGetMechRpm();
        out->speed_valid = Motor_Hall_AngleIsSpeedValid();
        break;

    default:
        out->elec_angle_rad = open_loop_theta;
        out->angle_valid = true;
        out->used_source = MOTOR_FEEDBACK_SRC_OPENLOOP;
        break;
    }
}

/**
 * @description: 默认单实例复位。
 */
void Motor_Feedback_ResetDefault(void)
{
    Motor_Feedback_Reset(&s_feedback);
}

/**
 * @description: 默认单实例设置来源。
 */
void Motor_Feedback_SetSourceDefault(motor_feedback_source_t source)
{
    Motor_Feedback_SetSource(&s_feedback, source);
}

/**
 * @description: 读取默认单实例请求源。
 */
motor_feedback_source_t Motor_Feedback_GetSourceDefault(void)
{
    return Motor_Feedback_GetSource(&s_feedback);
}

/**
 * @description: 是否为有感角源。
 */
bool Motor_Feedback_IsSensoredSource(motor_feedback_source_t source)
{
    return source == MOTOR_FEEDBACK_SRC_ENCODER || source == MOTOR_FEEDBACK_SRC_HALL;
}

/**
 * @description: 默认单实例采样。
 */
void Motor_Feedback_SampleDefault(const MOTOR_PARAM_S *motor, float open_loop_theta,
                                  motor_feedback_sample_t *out)
{
    Motor_Feedback_Sample(&s_feedback, motor, open_loop_theta, out);
}

#ifdef __cplusplus
}
#endif
