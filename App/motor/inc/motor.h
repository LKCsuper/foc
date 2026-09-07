/*
 * @Description: 电机公共 API（命令提交、状态快照、调试打印）
 * @Author: lkc
 */
#ifndef MOTOR_H
#define MOTOR_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "motor_param.h"
#include "motor_types.h"
#include "motor_settings.h"
#include "motor_feedback.h"
/* Private typedef -----------------------------------------------------------*/

/**
 * @description: 公共 API 返回码
 */
typedef enum {
    MOTOR_OK = 0,
    MOTOR_ERR_NULL = -1,
    MOTOR_ERR_INVALID = -2,
    MOTOR_ERR_FAULT = -3,
    MOTOR_ERR_BUSY = -4, /* 运行中禁止修改停机专用配置 */
} motor_result_t;

/**
 * @description: 运行命令（不持久化）。整体提交，避免 enable 与目标跨周期撕裂。
 */
typedef struct {
    bool enable;       /* true=允许输出；false=停机命令 */
    int mode;          /* MOTOR_OL_MODE_VF 或 MOTOR_OL_MODE_IF */
    float target_rpm;  /* 目标机械转速 (RPM)，负数为反转；零不等于停机 */
    float iq_set;      /* I/F 目标 Iq 幅值 (A)，符号由转速方向决定 */
    float id_set;      /* I/F 目标 Id (A) */
    bool speed_enable; /* true=速度环生成 Iq；仍须观测器反馈有效 */
} motor_command_t;

/**
 * @description: 只读运行快照，供 Shell/日志一次读取，避免跨周期拼接。
 */
typedef struct {
    bool enable;
    int mode;
    float target_rpm;  /* RPM */
    float current_rpm; /* 斜坡转速 RPM */
    float ia;          /* A */
    float ib;          /* A */
    float ic;          /* A */
    float id;          /* A */
    float iq;          /* A */
    float v_bus;       /* V */
    float theta;       /* 电角度 rad */
    uint32_t duty1;    /* TIM1 CCR1 */
    uint32_t duty2;    /* TIM1 CCR2 */
    uint32_t duty3;    /* TIM1 CCR3 */
    float vd_int;      /* V */
    float vq_int;      /* V */
    bool speed_enable;
    float measured_rpm; /* 速度环反馈 RPM；无效时为 0 */
    float speed_iq;     /* 速度环 Iq 目标 A */
    bool aligning;
    float align_elapsed_s; /* s */
    mc_fault_code fault;
    bool fault_latched;
} motor_status_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 从电机对象捕获当前命令字段。
 * @detail: 短临界区拷贝；不修改电机状态。可在任务上下文调用。
 * @param {const MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {motor_command_t *} cmd 输出命令，不可为空
 * @return {motor_result_t} MOTOR_OK 或 MOTOR_ERR_NULL
 * @author: lkc
 */
motor_result_t Motor_GetCommand(const MOTOR_PARAM_S *motor, motor_command_t *cmd);

/**
 * @description: 校验并整体提交命令到电机对象。
 * @detail: 拒绝 NaN/Inf、非法模式与负 Iq 幅值；按 ol_max_rpm 限幅目标转速。
 *          短临界区写入 enable/mode/目标电流/速度环开关。模式变化或停机时清零
 *          电流积分与斜坡转速，并复位速度环状态。不写 Flash。
 *          成功后按 enable 同步功率桥（PwmStart / PwmSetSafe）；Shell 勿再直写 PWM。
 *          可在任务上下文调用；ISR 在下一控制周期边界消费字段。
 * @param {MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {const motor_command_t *} cmd 完整命令，不可为空
 * @return {motor_result_t} MOTOR_OK / MOTOR_ERR_NULL / MOTOR_ERR_INVALID
 * @author: lkc
 */
motor_result_t Motor_SetCommand(MOTOR_PARAM_S *motor, const motor_command_t *cmd);

/**
 * @description: 获取一致的只读运行快照。
 * @detail: 短临界区拷贝样本与命令相关字段，再读取保护/速度环只读接口。
 *          可在任务上下文调用；不用于 ISR。
 * @param {const MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {motor_status_t *} status 输出快照，不可为空
 * @return {motor_result_t} MOTOR_OK 或 MOTOR_ERR_NULL
 * @author: lkc
 */
motor_result_t Motor_GetStatus(const MOTOR_PARAM_S *motor, motor_status_t *status);

/**
 * @description: 提交停机命令并清除斜坡/电流积分（不清故障锁存）。
 * @detail: enable=false、target_rpm=0，保留 mode/iq/id/speed_enable。
 *          不调用 HAL；调用方按需关断 PWM。可在任务上下文调用。
 * @param {MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @return {motor_result_t} MOTOR_OK 或错误码
 * @author: lkc
 */
motor_result_t Motor_RequestStop(MOTOR_PARAM_S *motor);

/**
 * @description: 从电机对象拷贝运行时快照（样本/斜坡/积分等）。
 * @detail: 短临界区；不改电机状态；不包含配置与命令目标。任务上下文。
 * @param {const MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {motor_runtime_t *} runtime 输出快照，不可为空
 * @return {motor_result_t} MOTOR_OK 或 MOTOR_ERR_NULL
 * @author: lkc
 */
motor_result_t Motor_GetRuntime(const MOTOR_PARAM_S *motor, motor_runtime_t *runtime);

/**
 * @description: 清除控制用运行时状态（斜坡转速与 d/q 积分），不改命令与配置。
 * @detail: 短临界区；不清除采样值与校准。可在任务或 ISR 调用。
 * @param {MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @return {motor_result_t} MOTOR_OK 或 MOTOR_ERR_NULL
 * @author: lkc
 */
motor_result_t Motor_ClearRuntimeControl(MOTOR_PARAM_S *motor);

/**
 * @description: 校验并应用配置（不改校准与运行命令目标）。
 * @detail: 委托 Motor_Settings_ApplyConfig；停机字段在运行中返回 MOTOR_ERR_BUSY。
 *          成功后按新 max_rpm 重新限幅当前命令目标。不写 Flash。任务上下文。
 * @param {MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {const motor_config_t *} config 完整配置快照，不可为空
 * @return {motor_result_t} MOTOR_OK / NULL / INVALID / BUSY
 * @author: lkc
 */
motor_result_t Motor_SetConfig(MOTOR_PARAM_S *motor, const motor_config_t *config);

/**
 * @description: 设置转子反馈来源（开环/无感/编码器/霍尔）。
 * @detail: 委托 Motor_Feedback_SetSourceDefault；I/F 下编码器/霍尔在角有效时走有感电流环。
 *          不写 Flash。可在任务上下文调用。
 * @param {motor_feedback_source_t} source 反馈源
 * @return {motor_result_t} MOTOR_OK；非法枚举返回 MOTOR_ERR_INVALID
 * @author: lkc
 */
motor_result_t Motor_SetFeedbackSource(motor_feedback_source_t source);

/**
 * @description: 读取当前请求的反馈来源。
 * @return {motor_feedback_source_t} 当前源
 * @author: lkc
 */
motor_feedback_source_t Motor_GetFeedbackSource(void);

/**
 * @description: 通过串口打印当前电机偏移等参数。
 * @return {*}
 * @author: lkc
 */
void MOTOR_ShowParam(void);

#ifdef __cplusplus
}
#endif
#endif
