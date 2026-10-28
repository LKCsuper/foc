/*
 * @Description: 统一转子反馈选择（开环/无感/编码器/霍尔）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "common_assert.h"
#include "motor_feedback.h"
#include "motor_sensorless.h"
#include "motor_encoder.h"
#include "motor_hall_angle.h"
#if defined(__CC_ARM)
/* 位于 ADC 中断控制路径，与其他实时模块一致局部 O2；不启用快速浮点，公式不变。 */
#pragma O2
#endif
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_feedback_t s_feedback = {
    .source = MOTOR_FEEDBACK_SRC_SENSORLESS,
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 复位反馈选择为默认无感源（与历史 GetCurrentTheta 行为一致）
 * @param {motor_feedback_t} *fb  实例，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Feedback_Reset(motor_feedback_t *fb)
{
    COMMON_ASSERT(fb != NULL);
    fb->source = MOTOR_FEEDBACK_SRC_SENSORLESS;
}

/**
 * @description: 设置请求的反馈来源
 * @param {motor_feedback_t} *fb           实例
 * @param {motor_feedback_source_t} source 来源
 * @return {void} 无
 * @author: lkc
 */
void Motor_Feedback_SetSource(motor_feedback_t *fb, motor_feedback_source_t source)
{
    COMMON_ASSERT(fb != NULL);
    fb->source = source;
}

/**
 * @description: 读取请求的反馈来源
 * @param {const motor_feedback_t} *fb  实例
 * @return {motor_feedback_source_t} 当前请求源；fb 空则 OPENLOOP
 * @author: lkc
 */
motor_feedback_source_t Motor_Feedback_GetSource(const motor_feedback_t *fb)
{
    return fb == NULL ? MOTOR_FEEDBACK_SRC_OPENLOOP : fb->source;
}

/**
 * @description: 按请求源解析本拍电角/转速有效标志（不写速度环、不访问 PWM）
 * @detail: ENCODER/HALL 角无效时回退 open_loop_theta；不伪造有效零速
 * @param {motor_feedback_t} *fb           实例
 * @param {const MOTOR_PARAM_S} *motor     电机（查 sensorless_enable 等），可为空
 * @param {float} open_loop_theta          开环/I/F 电角 (rad)
 * @param {motor_feedback_sample_t} *out   输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Feedback_Sample(motor_feedback_t *fb, const MOTOR_PARAM_S *motor, float open_loop_theta,
                           motor_feedback_sample_t *out)
{
    motor_feedback_source_t src;
    bool encoder_fault;

    COMMON_ASSERT(out != NULL);

    out->elec_angle_rad = open_loop_theta;
    out->mech_rpm = 0.0f;
    out->mech_rpm_raw = 0.0f;
    out->mech_rpm_fast = 0.0f;
    out->sample_age_s = 0.0f;
    out->sensor_state = -1;
    out->angle_valid = false;
    out->speed_valid = false;
    out->used_source = MOTOR_FEEDBACK_SRC_OPENLOOP;

    src = (fb != NULL) ? fb->source : MOTOR_FEEDBACK_SRC_OPENLOOP;

    switch (src) {
    case MOTOR_FEEDBACK_SRC_SENSORLESS:
        /*
         * 无感关闭时仍回退开环角，避免无角可用；速度有效仅在接管完成。
         * 速度环发布仍由 sensorless effects 负责，此处不伪造零速。
         */
        if (motor != NULL && !motor->sensorless_enable) {
            out->angle_valid = true;
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
        /* 角度和速度共用同一次故障采样，避免本拍中途标志变化使两者不一致。 */
        encoder_fault = Motor_Encoder_IsSignalFault();
        if (Motor_Encoder_IsZeroValid() && !encoder_fault) {
            out->elec_angle_rad = Motor_Encoder_GetElecAngle();
            out->angle_valid = true;
            out->used_source = MOTOR_FEEDBACK_SRC_ENCODER;
        }
        out->mech_rpm = Motor_Encoder_GetMechRpm();
        out->mech_rpm_raw = Motor_Encoder_GetRawMechRpm();
        out->mech_rpm_fast = out->mech_rpm;
        out->sample_age_s = Motor_Encoder_GetSampleAge();
        out->speed_valid = Motor_Encoder_IsSpeedValid() && !encoder_fault;
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
        }
        out->mech_rpm = Motor_Hall_AngleGetMechRpm();
        out->mech_rpm_raw = Motor_Hall_AngleGetRawMechRpm();
        out->mech_rpm_fast = Motor_Hall_AngleGetFastMechRpm();
        out->sample_age_s = Motor_Hall_AngleGetEdgeAge();
        out->sensor_state = Motor_Hall_AngleGetState();
        out->speed_valid = Motor_Hall_AngleIsSpeedValid();
        break;

    case MOTOR_FEEDBACK_SRC_OPENLOOP:
    default:
        out->angle_valid = true;
        break;
    }
}

/**
 * @description: 默认单实例复位为无感源
 * @return {void} 无
 * @author: lkc
 */
void Motor_Feedback_ResetDefault(void)
{
    Motor_Feedback_Reset(&s_feedback);
}

/**
 * @description: 默认单实例设置来源
 * @param {motor_feedback_source_t} source  来源
 * @return {void} 无
 * @author: lkc
 */
void Motor_Feedback_SetSourceDefault(motor_feedback_source_t source)
{
    if (source == MOTOR_FEEDBACK_SRC_HALL) {
        /* 同一反馈源重新选择也视为新会话，清除旧边沿周期与缓存表状态。 */
        Motor_Hall_AngleResetDefault();
    }
    Motor_Feedback_SetSource(&s_feedback, source);
}

/**
 * @description: 读取默认单实例请求的反馈来源
 * @return {motor_feedback_source_t} 当前请求源
 * @author: lkc
 */
motor_feedback_source_t Motor_Feedback_GetSourceDefault(void)
{
    return Motor_Feedback_GetSource(&s_feedback);
}

/**
 * @description: 是否为有感角源（编码器/霍尔），用于跳过无感更新与开环限速
 * @param {motor_feedback_source_t} source  请求源
 * @return {bool} true=ENCODER 或 HALL
 * @author: lkc
 */
bool Motor_Feedback_IsSensoredSource(motor_feedback_source_t source)
{
    return source == MOTOR_FEEDBACK_SRC_ENCODER || source == MOTOR_FEEDBACK_SRC_HALL;
}

/**
 * @description: 默认单实例采样
 * @param {const MOTOR_PARAM_S} *motor     电机
 * @param {float} open_loop_theta          开环电角 (rad)
 * @param {motor_feedback_sample_t} *out   输出
 * @return {void} 无
 * @author: lkc
 */
void Motor_Feedback_SampleDefault(const MOTOR_PARAM_S *motor, float open_loop_theta,
                                  motor_feedback_sample_t *out)
{
    Motor_Feedback_Sample(&s_feedback, motor, open_loop_theta, out);
}

#ifdef __cplusplus
}
#endif
