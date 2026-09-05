/*
 * @Description: 开环/I/F 主控制编排与输出仲裁（不含 HAL PWM）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 本模块决定“这一拍算什么、输出什么电压矢量”，不写定时器寄存器。
 *   2) 返回 motor_control_result_t：SAFE=请求三相中点占空比（非关桥、非保证无转矩），
 *      OUTPUT_DQ=vd/vq/theta；真正关通道用 Hal_Motor_PwmStop。
 *   2b) 电流环电角经 Motor_Feedback 统一选择（开环/无感/编码器/霍尔）。
 *   3) 启动链（对齐/HFI/I/F）状态收在 motor_control_t，便于实例化与主机测试。
 *   4) motor_it 负责：采样、保护、分频、调用 Step、把 result 落到 HAL。
 */
#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "motor_param.h"
#include "motor_alignment.h"
#include "motor_if_start.h"
#include "motor_hfi_start.h"

/**
 * @description: 启动链状态（对齐 + I/F 派生 + HFI）。
 */
typedef struct {
    motor_alignment_t alignment;
    motor_if_start_t if_start;
    motor_hfi_start_t hfi_start;
} motor_control_startup_t;

/**
 * @description: 控制编排实例（不持久化）。
 */
typedef struct {
    motor_control_startup_t startup;
} motor_control_t;

/**
 * @description: 本拍输出仲裁结果（由平台层应用 PWM）。
 */
typedef enum {
    MOTOR_CONTROL_ACTION_NONE = 0, /* 无 PWM 更新（极少用） */
    MOTOR_CONTROL_ACTION_SAFE,     /* 三相 mid=period/2；非关桥/非无转矩保证 */
    MOTOR_CONTROL_ACTION_OUTPUT_DQ /* 请求 vd/vq + theta 经反 Park/SVPWM */
} motor_control_action_t;

typedef struct {
    motor_control_action_t action;
    float vd;    /* V */
    float vq;    /* V */
    float theta; /* rad，Park/反 Park 用 */
} motor_control_result_t;

/**
 * @description: 初始化控制实例（启动链复位到可启动状态）。
 * @param {motor_control_t *} ctrl  控制实例
 * @return {*}
 * @author: lkc
 */
void Motor_Control_Init(motor_control_t *ctrl);

/**
 * @description: 复位启动链与速度/无感/电流积分显示（不清用户目标命令）。
 * @param {motor_control_t *} ctrl  控制实例
 * @param {MOTOR_PARAM_S *} motor   电机对象
 * @return {*}
 * @author: lkc
 */
void Motor_Control_ResetRuntime(motor_control_t *ctrl, MOTOR_PARAM_S *motor);

/**
 * @description: 计算电压限幅：min(ol_vq_limit, 0.75×Vbus)，且非负。
 * @param {const MOTOR_PARAM_S *} motor  含 ol_vq_limit
 * @param {float} v_bus  母线电压 (V)
 * @return {float} 限幅 (V)
 * @author: lkc
 */
float Motor_Control_GetVoltageLimit(const MOTOR_PARAM_S *motor, float v_bus);

/**
 * @description: 判断本拍是否允许运行开环/I/F。
 * @param {const MOTOR_PARAM_S *} motor  使能与目标
 * @param {float} v_bus  母线电压 (V)
 * @param {bool} hall_detect_active  霍尔标定占用时放宽“目标转速非零”
 * @return {bool} true=可继续控制
 * @author: lkc
 */
bool Motor_Control_IsReady(const MOTOR_PARAM_S *motor, float v_bus, bool hall_detect_active);

/**
 * @description: 钳位开环/无感相关配置字段到合法范围。
 * @param {MOTOR_PARAM_S *} motor  被原地修改
 * @return {*}
 * @author: lkc
 */
void Motor_Control_NormalizeConfig(MOTOR_PARAM_S *motor);

/**
 * @description: 按斜坡更新 ol_current_rpm；无感未完全接管时可限幅目标。
 * @param {MOTOR_PARAM_S *} motor  斜坡状态
 * @param {float} dt  控制周期 (s)
 * @param {bool} sensorless_takeover_complete  true=允许冲向最终目标
 * @return {*}
 * @author: lkc
 */
void Motor_Control_UpdateSpeed(MOTOR_PARAM_S *motor, float dt, bool sensorless_takeover_complete);

/**
 * @description: 用 I/F 启动模块的 angle_rpm 积分电角度。
 * @param {motor_control_t *} ctrl  控制实例
 * @param {MOTOR_PARAM_S *} motor   写 ol_theta
 * @param {float} dt  控制周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Control_UpdateTheta(motor_control_t *ctrl, MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 推进一拍开环/I/F/V/F/霍尔标定编排，返回输出请求。
 * @detail: 不调用 HAL；欠压或未就绪时复位并请求 SAFE。
 * @param {motor_control_t *} ctrl  控制实例
 * @param {MOTOR_PARAM_S *} motor   电机对象
 * @param {float} v_bus  母线电压 (V)
 * @param {float} dt  FOC 控制周期 (s)
 * @param {bool} hall_detect_active  霍尔标定是否占用功率桥
 * @return {motor_control_result_t} 本拍输出仲裁
 * @author: lkc
 */
motor_control_result_t Motor_Control_Step(motor_control_t *ctrl, MOTOR_PARAM_S *motor, float v_bus, float dt,
                                          bool hall_detect_active);

/**
 * @description: 查询是否处于预对齐。
 * @param {const motor_control_t *} ctrl  控制实例
 * @return {bool} true=对齐进行中
 * @author: lkc
 */
bool Motor_Control_IsAligning(const motor_control_t *ctrl);

/**
 * @description: 本次对齐已进行时间。
 * @param {const motor_control_t *} ctrl  控制实例
 * @return {float} 时长 (s)
 * @author: lkc
 */
float Motor_Control_GetAlignElapsed(const motor_control_t *ctrl);

/**
 * @description: HFI 是否在跑。
 * @param {const motor_control_t *} ctrl  控制实例
 * @return {bool} true=扫描中
 * @author: lkc
 */
bool Motor_Control_HfiIsRunning(const motor_control_t *ctrl);

/**
 * @description: HFI 是否已成功。
 * @param {const motor_control_t *} ctrl  控制实例
 * @return {bool} true=识别成功
 * @author: lkc
 */
bool Motor_Control_HfiSucceeded(const motor_control_t *ctrl);

/**
 * @description: HFI 识别电角度。
 * @param {const motor_control_t *} ctrl  控制实例
 * @return {float} 电角度 (rad)
 * @author: lkc
 */
float Motor_Control_HfiGetAngle(const motor_control_t *ctrl);

/**
 * @description: HFI 可信度。
 * @param {const motor_control_t *} ctrl  控制实例
 * @return {float} [0,1]
 * @author: lkc
 */
float Motor_Control_HfiGetConfidence(const motor_control_t *ctrl);

/**
 * @description: 故障仲裁：已锁存故障时强制 SAFE（中点占空比请求，非关桥），禁止 DQ 覆盖。
 * @param {bool} fault_latched  保护是否已锁存
 * @param {const motor_control_result_t *} proposed  Control_Step 提议的输出
 * @return {motor_control_result_t} 最终允许应用的输出
 * @author: lkc
 */
motor_control_result_t Motor_Control_ArbitrateOutput(bool fault_latched,
                                                     const motor_control_result_t *proposed);

#ifdef __cplusplus
}
#endif

#endif
