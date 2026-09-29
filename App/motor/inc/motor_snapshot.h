/*
 * @Description: 电机只读状态帧定义
 * @Author: lkc
 *
 * 捕获和打印分离：控制仍可继续运行，Shell 只打印已经保存的一帧。
 */
#ifndef MOTOR_SNAPSHOT_H
#define MOTOR_SNAPSHOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "motor_feedback.h"
#include "motor_protect.h"

typedef struct {
    bool enable;
    int mode;
    float target_rpm;  /* RPM */
    float current_rpm; /* 斜坡转速 RPM */
    float ia;          /* A */
    float ib;          /* A */
    float ic;          /* A */
    float current_adc[3];    /* CURR1/2/3 原始 ADC 计数，与 abc 同一快照 */
    float current_offset[3]; /* CURR1/2/3 零偏 ADC 计数 */
    float id;          /* A */
    float iq;          /* A */
    float id_target;   /* 本拍电流环 Id 目标 A */
    float iq_target;   /* 本拍电流环 Iq 目标 A */
    float v_bus;       /* V */
    float theta;       /* 本拍 Park/PWM 电角度 rad */
    uint32_t duty1;    /* TIM1 CCR1 */
    uint32_t duty2;    /* TIM1 CCR2 */
    uint32_t duty3;    /* TIM1 CCR3 */
    float vd_int;      /* V */
    float vq_int;      /* V */
    bool speed_enable;
    float measured_rpm; /* 速度环反馈 RPM；无效时为 0 */
    float speed_error;  /* 与 measured_rpm 同一快照时刻的速度误差 RPM */
    float speed_iq;     /* 速度环 Iq 目标 A */
    motor_feedback_sample_t feedback;
    bool aligning;
    float align_elapsed_s; /* s */
    mc_fault_code fault;
    bool fault_latched;
} motor_status_t;

#ifdef __cplusplus
}
#endif
#endif
