/*
 * @Description: 开环/I/F 主控制编排与输出仲裁（不含 HAL）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <string.h>
#include <stddef.h>

#include "motor_control.h"
#include "motor_current.h"
#include "motor_foc.h"
#include "motor_math.h"
#include "motor_speed.h"
#include "motor_sensorless.h"
#include "motor_feedback.h"
#include "motor_hall.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static motor_control_result_t Motor_Control_ResultSafe(void);
static motor_control_result_t Motor_Control_ResultDq(float vd, float vq, float theta);
static motor_control_result_t Motor_Control_PublishTheta(MOTOR_PARAM_S *motor, motor_control_result_t result);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 构造 SAFE 输出请求（三相 mid 占空比语义，非关桥保证）。
 * @return {motor_control_result_t} action=SAFE，vd/vq/theta 清零
 * @author: lkc
 */
static motor_control_result_t Motor_Control_ResultSafe(void)
{
    motor_control_result_t result;

    memset(&result, 0, sizeof(result));
    result.action = MOTOR_CONTROL_ACTION_SAFE;
    return result;
}

/**
 * @description: 构造 DQ 电压输出请求。
 * @param {float} vd  D 轴电压 (V)
 * @param {float} vq  Q 轴电压 (V)
 * @param {float} theta  电角度 (rad)
 * @return {motor_control_result_t} action=OUTPUT_DQ
 * @author: lkc
 */
static motor_control_result_t Motor_Control_ResultDq(float vd, float vq, float theta)
{
    motor_control_result_t result;

    result.action = MOTOR_CONTROL_ACTION_OUTPUT_DQ;
    result.vd = vd;
    result.vq = vq;
    result.theta = theta;
    return result;
}

/**
 * @description: 把本拍 Park/PWM 电角写回 motor->theta，供 olstatus 快照。
 * @detail: I/F 只积分 ol_theta；若不回写，状态页 Theta 会停在旧值而 CCR 仍在变。
 * @param {MOTOR_PARAM_S *} motor  电机对象
 * @param {motor_control_result_t} result  本拍控制结果
 * @return {motor_control_result_t} 原样返回 result
 */
static motor_control_result_t Motor_Control_PublishTheta(MOTOR_PARAM_S *motor, motor_control_result_t result)
{
    if (motor != NULL && result.action == MOTOR_CONTROL_ACTION_OUTPUT_DQ) {
        motor->theta = result.theta;
    }
    return result;
}

/**
 * @description: 初始化控制实例。
 * @param {motor_control_t} *ctrl  控制实例
 * @return {*}
 * @author: lkc
 */
void Motor_Control_Init(motor_control_t *ctrl)
{
    if (ctrl == NULL) {
        return;
    }
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->startup.alignment.stopped = true;
    ctrl->startup.if_start.first_cycle = true;
    ctrl->startup.if_start.iq_scale = 1.0f;
    ctrl->startup.phase = MOTOR_STARTUP_CURRENT_RAMP;
    ctrl->last_mode = -1;
}

/**
 * @description: 复位启动链与速度/无感/电流积分显示。
 * @param {motor_control_t} *ctrl  控制实例
 * @param {MOTOR_PARAM_S} *motor   电机对象
 * @return {*}
 * @author: lkc
 */
void Motor_Control_ResetRuntime(motor_control_t *ctrl, MOTOR_PARAM_S *motor)
{
    if (motor == NULL) {
        return;
    }
    /* 与 Motor_ClearRuntimeControl 一致：清斜坡与 d/q 积分，保留命令目标。 */
    motor->ol_current_rpm = 0.0f;
    motor->ol_vd_int = 0.0f;
    motor->ol_vq_int = 0.0f;
    motor->Id = 0.0f;
    motor->Iq = 0.0f;
    Motor_SpeedLoop_Reset();
    Motor_Sensorless_Reset();
    if (ctrl != NULL) {
        MotorAlignment_Reset(&ctrl->startup.alignment);
        MotorIfStart_Reset(&ctrl->startup.if_start);
        MotorHfiStart_Reset(&ctrl->startup.hfi_start);
        ctrl->startup.phase = MOTOR_STARTUP_CURRENT_RAMP;
        ctrl->startup.soft_current_scale = 0.0f;
        ctrl->startup.drag_current_scale = 0.0f;
    }
}

/**
 * @description: 电压限幅 = min(配置, Vbus/sqrt(3))。
 * @param {const MOTOR_PARAM_S} *motor  配置
 * @param {float} v_bus  母线 (V)
 * @return {float} 限幅 (V)
 * @author: lkc
 */
float Motor_Control_GetVoltageLimit(const MOTOR_PARAM_S *motor, float v_bus)
{
    float limit;

    if (motor == NULL) {
        return 0.0f;
    }
    limit = motor->ol_vq_limit;
    if (limit < 0.0f) {
        limit = 0.0f;
    }
    /* 当前 alpha/beta 定义下，Vbus/sqrt(3) 保证任意角度都处在线性调制区。 */
    if (limit > v_bus * ONE_BY_SQRT3) {
        limit = v_bus * ONE_BY_SQRT3;
    }
    return limit;
}

/**
 * @description: 使能 + 目标（或霍尔标定）+ 母线足够。
 * @param {const MOTOR_PARAM_S} *motor  命令
 * @param {float} v_bus  母线 (V)
 * @param {bool} hall_detect_active  霍尔标定
 * @return {bool} true=可运行
 * @author: lkc
 */
bool Motor_Control_IsReady(const MOTOR_PARAM_S *motor, float v_bus, bool hall_detect_active)
{
    if (motor == NULL) {
        return false;
    }
    if (hall_detect_active) {
        return motor->ol_enable && v_bus >= 1.0f;
    }
    return motor->ol_enable && motor->ol_target_rpm != 0.0f && v_bus >= 1.0f;
}

/**
 * @description: 钳位开环/无感配置。
 * @param {MOTOR_PARAM_S} *motor  被修改
 * @return {*}
 * @author: lkc
 */
void Motor_Control_NormalizeConfig(MOTOR_PARAM_S *motor)
{
    motor_alignment_config_t alignment_config;
    motor_hfi_start_config_t hfi_config;

    if (motor == NULL) {
        return;
    }

    if (motor->ol_pole_pairs < 1.0f) {
        motor->ol_pole_pairs = 1.0f;
    }
    if (motor->ol_min_rpm < 0.0f) {
        motor->ol_min_rpm = 0.0f;
    }
    if (motor->ol_ramp_rpm_s < 0.0f) {
        motor->ol_ramp_rpm_s = 0.0f;
    }

    /* 对齐电流/时间为 0 时视为关闭预对齐，保留旧的一拍相位补偿作为回退。 */
    alignment_config.enabled = motor->ol_align_enable;
    alignment_config.current = motor->ol_align_current;
    alignment_config.duration = motor->ol_align_time;
    alignment_config.angle = motor->ol_align_angle;
    MotorAlignment_NormalizeConfig(&alignment_config);
    motor->ol_align_current = alignment_config.current;
    motor->ol_align_time = alignment_config.duration;
    utils_norm_angle_rad(&motor->ol_align_angle);

    hfi_config.enabled = motor->hfi_start_enable;
    hfi_config.voltage = motor->hfi_start_voltage;
    hfi_config.cycles_per_axis = motor->hfi_start_cycles;
    hfi_config.min_confidence = motor->hfi_start_min_confidence;
    hfi_config.max_bus_voltage_ratio = 0.20f;
    MotorHfiStart_NormalizeConfig(&hfi_config);
    motor->hfi_start_voltage = hfi_config.voltage;
    motor->hfi_start_cycles = hfi_config.cycles_per_axis;
    motor->hfi_start_min_confidence = hfi_config.min_confidence;

    if (motor->ol_max_rpm <= 0.0f) {
        motor->ol_max_rpm = 1.0f;
    }
    if (motor->ol_target_rpm > motor->ol_max_rpm) {
        motor->ol_target_rpm = motor->ol_max_rpm;
    }
    if (motor->ol_target_rpm < -motor->ol_max_rpm) {
        motor->ol_target_rpm = -motor->ol_max_rpm;
    }

    if (motor->speed_kp < 0.0f) {
        motor->speed_kp = 0.0f;
    }
    if (motor->speed_ki < 0.0f) {
        motor->speed_ki = 0.0f;
    }
    if (motor->speed_iq_limit < 0.0f) {
        motor->speed_iq_limit = -motor->speed_iq_limit;
    }
    if (motor->speed_loop_hz < 1.0f) {
        motor->speed_loop_hz = 1.0f;
    }
    if (motor->sensorless_motor_r < 0.0f) {
        motor->sensorless_motor_r = 0.0f;
    }
    if (motor->sensorless_motor_l < 0.0f) {
        motor->sensorless_motor_l = 0.0f;
    }
    if (motor->sensorless_flux_linkage < 1.0e-6f) {
        motor->sensorless_flux_linkage = 1.0e-6f;
    }
    if (motor->sensorless_observer_gain < 0.0f) {
        motor->sensorless_observer_gain = 0.0f;
    }
    if (motor->sensorless_pll_kp < 0.0f) {
        motor->sensorless_pll_kp = 0.0f;
    }
    if (motor->sensorless_pll_ki < 0.0f) {
        motor->sensorless_pll_ki = 0.0f;
    }
    if (motor->sensorless_transition_rpm < 0.0f) {
        motor->sensorless_transition_rpm = 0.0f;
    }
    if (motor->sensorless_fallback_rpm < 0.0f) {
        motor->sensorless_fallback_rpm = 0.0f;
    }
    if (motor->sensorless_fallback_rpm > motor->sensorless_transition_rpm) {
        motor->sensorless_fallback_rpm = motor->sensorless_transition_rpm;
    }
    if (motor->sensorless_openloop_max_rpm < 0.0f) {
        motor->sensorless_openloop_max_rpm = -motor->sensorless_openloop_max_rpm;
    }
    if (motor->sensorless_openloop_max_rpm > 0.0f &&
        motor->sensorless_openloop_max_rpm < motor->sensorless_transition_rpm) {
        motor->sensorless_openloop_max_rpm = motor->sensorless_transition_rpm;
    }
    if (motor->sensorless_openloop_max_rpm > motor->ol_max_rpm) {
        motor->sensorless_openloop_max_rpm = motor->ol_max_rpm;
    }
    if (motor->sensorless_lock_time < 0.0f) {
        motor->sensorless_lock_time = 0.0f;
    }
    if (motor->sensorless_loss_time < 0.0f) {
        motor->sensorless_loss_time = 0.0f;
    }
    if (motor->sensorless_blend_time < 0.0f) {
        motor->sensorless_blend_time = 0.0f;
    }
    if (motor->sensorless_phase_tolerance < 0.0f) {
        motor->sensorless_phase_tolerance = 0.0f;
    }
    /* 接管相位差必须小于 90°，否则电流坐标可能跨过最大转矩点。 */
    if (motor->sensorless_phase_tolerance > 1.50f) {
        motor->sensorless_phase_tolerance = 1.50f;
    }
    if (motor->sensorless_speed_filter < 0.0f) {
        motor->sensorless_speed_filter = 0.0f;
    }
    if (motor->sensorless_speed_filter > 1.0f) {
        motor->sensorless_speed_filter = 1.0f;
    }
}

/**
 * @description: 转速斜坡；无感未完全接管时可限制目标。
 * @param {MOTOR_PARAM_S} *motor  斜坡
 * @param {float} dt  周期 (s)
 * @param {bool} sensorless_takeover_complete  是否已完全用观测角
 * @return {*}
 * @author: lkc
 */
void Motor_Control_UpdateSpeed(MOTOR_PARAM_S *motor, float dt, bool sensorless_takeover_complete)
{
    float target;
    float step;

    if (motor == NULL) {
        return;
    }

    target = motor->ol_target_rpm;
    step = motor->ol_ramp_rpm_s * dt;

    /*
     * 无感尚未 CLOSED 且渐变未完成时，I/F 只是启动手段，不能无限强拖到目标高速。
     * 将 openloop_max 设为 0 可关闭此保护。
     */
    if (motor->ol_mode == MOTOR_OL_MODE_IF && !sensorless_takeover_complete &&
        motor->sensorless_openloop_max_rpm > 0.0f && fabsf(target) > motor->sensorless_openloop_max_rpm) {
        target = SIGN(target) * motor->sensorless_openloop_max_rpm;
    }

    if (step <= 0.0f) {
        motor->ol_current_rpm = target;
        return;
    }

    if (motor->ol_current_rpm < target) {
        motor->ol_current_rpm += step;
        if (motor->ol_current_rpm > target) {
            motor->ol_current_rpm = target;
        }
    } else if (motor->ol_current_rpm > target) {
        motor->ol_current_rpm -= step;
        if (motor->ol_current_rpm < target) {
            motor->ol_current_rpm = target;
        }
    }
}

/**
 * @description: 积分电角度。
 * @param {motor_control_t} *ctrl  控制实例
 * @param {MOTOR_PARAM_S} *motor   写 ol_theta
 * @param {float} dt  周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Control_UpdateTheta(motor_control_t *ctrl, MOTOR_PARAM_S *motor, float dt)
{
    if (ctrl == NULL || motor == NULL) {
        return;
    }
    motor->ol_theta += RPM2RADPS_f(ctrl->startup.if_start.angle_rpm) * motor->ol_pole_pairs * dt;
    utils_norm_angle_rad(&motor->ol_theta);
}

/**
 * @description: V/F 启动：同步斜坡生成的转速，不额外制造最低转速跳变。
 * @param {motor_control_t} *ctrl  控制实例
 * @param {MOTOR_PARAM_S} *motor   开环转速状态
 * @return {*}
 * @author: lkc
 */
static void Motor_Control_StartupVf(motor_control_t *ctrl, MOTOR_PARAM_S *motor)
{
    MotorIfStart_SetVfRpm(&ctrl->startup.if_start, motor->ol_current_rpm);
}

/**
 * @description: 推进预对齐一步；进行中写对齐角并冻结开环转速。
 * @param {motor_control_t} *ctrl  控制实例
 * @param {MOTOR_PARAM_S} *motor   电机对象
 * @param {float} dt  控制周期 (s)
 * @return {bool} true=对齐仍在进行（调用方应跳过后续启动）
 * @author: lkc
 */
static bool Motor_Control_RunAlignment(motor_control_t *ctrl, MOTOR_PARAM_S *motor, float dt)
{
    motor_alignment_config_t config = {motor->ol_align_enable, motor->ol_align_current, motor->ol_align_time,
                                       motor->ol_align_angle};
    motor_alignment_output_t output;
    motor_alignment_result_t result = MotorAlignment_Step(&ctrl->startup.alignment, &config, dt, &output);

    if (result == MOTOR_ALIGNMENT_STARTED) {
        MotorIfStart_SetNeutral(&ctrl->startup.if_start);
        motor->ol_current_rpm = 0.0f;
        motor->ol_vd_int = 0.0f;
        motor->ol_vq_int = 0.0f;
        Motor_SpeedLoop_Reset();
        Motor_Sensorless_Reset();
    }

    if (result == MOTOR_ALIGNMENT_STARTED || result == MOTOR_ALIGNMENT_RUNNING) {
        motor->ol_theta = output.angle;
        motor->ol_current_rpm = 0.0f;
        MotorIfStart_SetNeutral(&ctrl->startup.if_start);
        return true;
    }

    if (result == MOTOR_ALIGNMENT_FINISHED) {
        motor->ol_vd_int = 0.0f;
        motor->ol_vq_int = 0.0f;
        Motor_SpeedLoop_Reset();
        Motor_Sensorless_Reset();
    }
    return false;
}

/**
 * @description: I/F 启动一步：同步平滑转速，定角建立后不再叠加旧式相位提升。
 * @param {motor_control_t} *ctrl  控制实例
 * @param {MOTOR_PARAM_S} *motor   电机对象
 * @return {*}
 * @author: lkc
 */
static void Motor_Control_StartupIf(motor_control_t *ctrl, MOTOR_PARAM_S *motor)
{
    /* 前两阶段已经用 D 轴建立方向；拖转从 0 RPM 连续起步并由 Id→Iq 交接控制转矩。 */
    motor_if_start_config_t config = {
        0.0f,
        0.0f,
        1.0f,
    };
    motor_if_start_input_t input = {motor->ol_target_rpm, motor->ol_current_rpm, true};

    MotorIfStart_Step(&ctrl->startup.if_start, &config, &input);
}

/**
 * @description: 取本拍电流环电角度（开环角+相位提升，经反馈源选择）。
 * @detail: 默认无感；编码器/霍尔在角有效时走有感电流环电角。
 * @param {motor_control_t} *ctrl  控制实例
 * @param {MOTOR_PARAM_S} *motor   电机对象
 * @return {float} 电角度 (rad)
 * @author: lkc
 */
static float Motor_Control_GetCurrentTheta(motor_control_t *ctrl, MOTOR_PARAM_S *motor)
{
    float theta = motor->ol_theta + ctrl->startup.if_start.phase_boost;
    motor_feedback_sample_t feedback;

    utils_norm_angle_rad(&theta);
    /*
     * 统一反馈选择：默认无感；编码器/霍尔在角有效时走有感电流环电角。
     */
    Motor_Feedback_SampleDefault(motor, theta, &feedback);
    return feedback.elec_angle_rad;
}

/**
 * @description: 有感模式下发布编码器/霍尔转速。
 * @detail: 须角、速均有效才置 feedback_valid，避免无零点/非法扇区时开速度环。
 * @param {MOTOR_PARAM_S} *motor  电机对象
 * @return {*}
 * @author: lkc
 */
static void Motor_Control_PublishSensoredSpeed(MOTOR_PARAM_S *motor)
{
    motor_feedback_sample_t feedback;
    float open = motor->ol_theta;
    bool publish;

    utils_norm_angle_rad(&open);
    Motor_Feedback_SampleDefault(motor, open, &feedback);
    publish = feedback.angle_valid && feedback.speed_valid;
    /* Hall 用快速支路闭环；六边沿滤波值保留给状态显示和稳定性审核。 */
    Motor_SpeedFeedback_Set(publish ? feedback.mech_rpm_fast : 0.0f, publish);
}

/**
 * @description: 判断当前转速是否已达到速度 PI 的平滑接管门槛
 * @detail: 门槛至少为启动转速的一半，并随斜坡目标提高到 50%；未达到时禁止 PI 后台积分。
 */
static bool Motor_Control_SpeedLoopReady(const MOTOR_PARAM_S *motor)
{
    float release_rpm;

    if (motor == NULL || !motor->speed_enable || !Motor_SpeedLoop_IsFeedbackValid()) {
        return false;
    }
    release_rpm = fmaxf(0.5f * fabsf(motor->ol_min_rpm), 0.5f * fabsf(motor->ol_current_rpm));
    return fabsf(Motor_SpeedLoop_GetMeasuredRpm()) >= release_rpm;
}

/**
 * @description: 填充电流环目标上下文（对齐、I/F 缩放、速度环 Iq）。
 * @param {motor_control_t} *ctrl  控制实例
 * @param {MOTOR_PARAM_S} *motor   电机对象
 * @param {motor_current_target_ctx_t} *ctx  输出上下文
 * @return {*}
 * @author: lkc
 */
static void Motor_Control_FillCurrentCtx(motor_control_t *ctrl, MOTOR_PARAM_S *motor, motor_current_target_ctx_t *ctx)
{
    bool dragging = ctrl->startup.phase == MOTOR_STARTUP_DRAG_RAMP;

    ctx->alignment_running = !dragging;
    ctx->if_iq_scale = ctrl->startup.if_start.iq_scale;
    /*
     * “反馈有效的零速”不等于“已经起转”。低于启动门槛时保留 oliq 启动转矩；
     * 达到门槛后才让速度 PI 接管，运行中失速跌回门槛以下也自动恢复。
     */
    ctx->speed_loop_active = Motor_Control_SpeedLoopReady(motor);
    ctx->speed_iq_target = ctx->speed_loop_active ? Motor_SpeedLoop_GetIqTarget() : 0.0f;
    if (!dragging) {
        ctx->alignment_current_scale = ctrl->startup.soft_current_scale;
        ctx->soft_current_scale = ctrl->startup.soft_current_scale;
        return;
    }
    if (!motor->ol_align_enable) {
        /* 未开对齐：没有定角 Id 可交接，按 drag 比例给运行 Iq，避免 0.5 A 直轴把转子按进齿槽。 */
        ctx->alignment_current_scale = 0.0f;
        ctx->soft_current_scale = ctrl->startup.drag_current_scale;
        return;
    }
    ctx->alignment_current_scale = 1.0f - ctrl->startup.drag_current_scale;
    ctx->soft_current_scale = ctrl->startup.drag_current_scale;
}

/**
 * @description: 推进电流软起比例（约 0.8 s 到满幅），停机时清零
 * @param {motor_control_t} *ctrl  控制实例
 * @param {const MOTOR_PARAM_S} *motor  使能状态
 * @param {float} dt  控制周期 (s)
 * @return {*}
 * @author: lkc
 */
static void Motor_Control_UpdateSoftCurrent(motor_control_t *ctrl, const MOTOR_PARAM_S *motor, float dt)
{
    const float soft_time_s = 0.8f;

    if (ctrl == NULL || motor == NULL) {
        return;
    }
    if (!motor->ol_enable) {
        ctrl->startup.soft_current_scale = 0.0f;
        return;
    }
    if (dt <= 0.0f) {
        return;
    }
    ctrl->startup.soft_current_scale += dt / soft_time_s;
    if (ctrl->startup.soft_current_scale > 1.0f) {
        ctrl->startup.soft_current_scale = 1.0f;
    }
}

/**
 * @description: 推进 Id→Iq 交接比例，0.25 s 内平滑进入拖转电流。
 * @param {motor_control_t} *ctrl 控制实例
 * @param {float} dt 控制周期 (s)
 * @return {*}
 */
static void Motor_Control_UpdateDragCurrent(motor_control_t *ctrl, float dt)
{
    const float transition_time_s = 0.25f;

    if (ctrl == NULL || dt <= 0.0f) {
        return;
    }
    ctrl->startup.drag_current_scale += dt / transition_time_s;
    if (ctrl->startup.drag_current_scale > 1.0f) {
        ctrl->startup.drag_current_scale = 1.0f;
    }
}

/**
 * @description: I/F 路径：选角、填目标上下文并跑电流环，返回 DQ 输出。
 * @param {motor_control_t} *ctrl  控制实例
 * @param {MOTOR_PARAM_S} *motor   电机对象
 * @param {float} v_bus  母线 (V)
 * @param {float} dt  控制周期 (s)
 * @return {motor_control_result_t} DQ 电压请求
 * @author: lkc
 */
static motor_control_result_t Motor_Control_RunIf(motor_control_t *ctrl, MOTOR_PARAM_S *motor, float v_bus, float dt)
{
    float theta = Motor_Control_GetCurrentTheta(ctrl, motor);
    float v_limit = Motor_Control_GetVoltageLimit(motor, v_bus);
    motor_current_target_ctx_t ctx;
    motor_dq_voltage_t voltage;

    Motor_Control_FillCurrentCtx(ctrl, motor, &ctx);
    /* 电流目标已按 soft/drag 比例爬升；电压上限不再叠乘 soft，避免积分顶满后过流。 */
    voltage = Motor_Current_Step(motor, theta, &ctx, dt, v_limit, NULL, NULL);
    /*
     * I/F 的 Park 角是开环磁场角，不是转子角。ωψf 前馈必须在转子 dq 上；
     * 加在 I/F 轴上会随转速把 Id/Iq 打偏（72 RPM 还行，300 RPM 见过 0.17 A 或 1.0/1.1 A）。
     */
    return Motor_Control_ResultDq(voltage.vd, voltage.vq, theta);
}

/**
 * @description: V/F 路径：按电气频率计算 Vq，固定 Vd=0，用开环电角输出。
 * @detail: 电压软起并受统一线性调制电压上限约束。
 * @param {motor_control_t} *ctrl  控制实例（取软起比例）
 * @param {MOTOR_PARAM_S} *motor  电机对象
 * @param {float} v_bus  母线 (V)
 * @return {motor_control_result_t} DQ 电压请求
 * @author: lkc
 */
static motor_control_result_t Motor_Control_RunVf(motor_control_t *ctrl, MOTOR_PARAM_S *motor, float v_bus)
{
    float vq_limit = Motor_Control_GetVoltageLimit(motor, v_bus);
    float f_elec = fabsf(motor->ol_current_rpm) / 60.0f * motor->ol_pole_pairs;
    float soft;
    float vq;

    if (motor->ol_v_boost < 0.0f) {
        motor->ol_v_boost = 0.0f;
    }
    if (motor->ol_kv_f < 0.0f) {
        motor->ol_kv_f = 0.0f;
    }

    soft = (ctrl != NULL) ? ctrl->startup.soft_current_scale : 1.0f;
    if (soft < 0.0f) {
        soft = 0.0f;
    }
    if (soft > 1.0f) {
        soft = 1.0f;
    }

    vq = (motor->ol_v_boost + motor->ol_kv_f * f_elec) * soft;
    if (vq > vq_limit) {
        vq = vq_limit;
    }
    return Motor_Control_ResultDq(0.0f, vq, motor->ol_theta);
}

/**
 * @description: 霍尔标定路径：I/F 电流闭环（对齐用 Id，扫角用 Iq）+ 强制电角慢转。
 * @detail: 开环电压易过流；纯 Id 扫角易卡齿槽。扫角阶段用 Iq 拖动，PI 限流。
 * @param {MOTOR_PARAM_S} *motor  电机对象
 * @param {float} v_bus  母线 (V)
 * @param {float} dt  控制周期 (s)
 * @return {motor_control_result_t} DQ 或 SAFE（无有效目标时）
 * @author: lkc
 */
static motor_control_result_t Motor_Control_RunHallDetect(MOTOR_PARAM_S *motor, float v_bus, float dt)
{
    float theta;
    float id_target;
    float iq_target;
    float v_limit;
    motor_dq_voltage_t voltage;

    if (!Motor_Hall_DetectGetTargets(motor, &theta, &id_target, &iq_target)) {
        return Motor_Control_ResultSafe();
    }

    v_limit = Motor_Control_GetVoltageLimit(motor, v_bus);
    if (v_limit > v_bus * 0.35f) {
        v_limit = v_bus * 0.35f;
    }
    if (v_limit > 10.0f) {
        v_limit = 10.0f;
    }
    Motor_Current_FeedbackUpdateDq(motor, theta);
    voltage = Motor_Current_RunPi(motor, id_target, iq_target, dt, v_limit);
    return Motor_Control_ResultDq(voltage.vd, voltage.vq, theta);
}

/**
 * @description: 推进 HFI 启动扫描；占用本拍输出时写 result。
 * @detail: 扫描中每拍输出探测电压；结束时请求 SAFE，成功则标记对齐完成。
 * @param {motor_control_t} *ctrl  控制实例
 * @param {MOTOR_PARAM_S} *motor   电机对象
 * @param {float} v_bus  母线 (V)
 * @param {motor_control_result_t} *result  本拍输出（仅本函数返回 true 时有效）
 * @return {bool} true=本拍已给出 result（调用方应直接返回）
 * @author: lkc
 */
static bool Motor_Control_RunHfi(motor_control_t *ctrl, MOTOR_PARAM_S *motor, float v_bus,
                                 motor_control_result_t *result)
{
    motor_hfi_start_config_t config = {motor->hfi_start_enable, motor->hfi_start_voltage, motor->hfi_start_cycles,
                                       motor->hfi_start_min_confidence, 0.20f};
    motor_hfi_start_output_t output;
    motor_hfi_start_result_t hfi_result;
    bool starting;

    if (!config.enabled || config.voltage <= 0.0f || ctrl->startup.hfi_start.attempted) {
        return false;
    }
    starting = !ctrl->startup.hfi_start.running;

    if (starting) {
        motor->ol_vd_int = 0.0f;
        motor->ol_vq_int = 0.0f;
        Motor_SpeedLoop_Reset();
        Motor_Sensorless_Reset();
    }

    clarke_transform(motor->ia, motor->ib, motor->ic, &motor->Ialpha, &motor->Ibeta);
    hfi_result = MotorHfiStart_Step(&ctrl->startup.hfi_start, &config, motor->Ialpha, motor->Ibeta, v_bus, &output);
    if (hfi_result == MOTOR_HFI_START_RUNNING) {
        *result = Motor_Control_ResultDq(output.vd, 0.0f, output.theta);
        return true;
    }
    if (hfi_result == MOTOR_HFI_START_FINISHED) {
        *result = Motor_Control_ResultSafe();
        if (ctrl->startup.hfi_start.succeeded) {
            motor->ol_theta = ctrl->startup.hfi_start.angle;
            MotorAlignment_MarkComplete(&ctrl->startup.alignment);
            MotorIfStart_SetNeutral(&ctrl->startup.if_start);
        }
        return true;
    }
    return false;
}

/**
 * @description: 推进一拍控制编排并返回输出仲裁。
 * @param {motor_control_t} *ctrl  控制实例
 * @param {MOTOR_PARAM_S} *motor   电机
 * @param {float} v_bus  母线 (V)
 * @param {float} dt  FOC 周期 (s)
 * @param {bool} hall_detect_active  霍尔标定
 * @return {motor_control_result_t} 输出请求
 * @author: lkc
 */
motor_control_result_t Motor_Control_Step(motor_control_t *ctrl, MOTOR_PARAM_S *motor, float v_bus, float dt,
                                          bool hall_detect_active)
{
    motor_control_result_t result;
    motor_control_result_t hfi_result;

    memset(&result, 0, sizeof(result));
    result.action = MOTOR_CONTROL_ACTION_NONE;

    if (ctrl == NULL || motor == NULL) {
        return Motor_Control_ResultSafe();
    }

    /* 模式变化由控制实例自己识别，确保内部启动上下文也被完整复位。 */
    if (ctrl->last_mode >= 0 && ctrl->last_mode != motor->ol_mode) {
        Motor_Control_ResetRuntime(ctrl, motor);
    }
    ctrl->last_mode = motor->ol_mode;

    /* 欠压：复位并请求安全输出。 */
    if (v_bus < 1.0f) {
        Motor_Control_ResetRuntime(ctrl, motor);
        return Motor_Control_ResultSafe();
    }

    if (!Motor_Control_IsReady(motor, v_bus, hall_detect_active)) {
        Motor_Control_ResetRuntime(ctrl, motor);
        return Motor_Control_ResultSafe();
    }

    if (hall_detect_active) {
        result = Motor_Control_RunHallDetect(motor, v_bus, dt);
        if (result.action == MOTOR_CONTROL_ACTION_SAFE) {
            Motor_Control_ResetRuntime(ctrl, motor);
        }
        return Motor_Control_PublishTheta(motor, result);
    }

    Motor_Control_NormalizeConfig(motor);

    if (motor->ol_mode == MOTOR_OL_MODE_IF) {
        if (Motor_Control_RunHfi(ctrl, motor, v_bus, &hfi_result)) {
            return Motor_Control_PublishTheta(motor, hfi_result);
        }
        if (!motor->ol_align_enable && ctrl->startup.phase == MOTOR_STARTUP_CURRENT_RAMP) {
            /* 验证默认 olalign 0：定角灌 Id 会把转子吸进齿槽，随后 10 RPM/s 的 Iq 拖不动。 */
            ctrl->startup.phase = MOTOR_STARTUP_DRAG_RAMP;
            ctrl->startup.drag_current_scale = 0.0f;
        } else if (ctrl->startup.phase == MOTOR_STARTUP_CURRENT_RAMP) {
            Motor_Control_UpdateSoftCurrent(ctrl, motor, dt);
            motor->ol_current_rpm = 0.0f;
            MotorIfStart_SetNeutral(&ctrl->startup.if_start);
            if (ctrl->startup.soft_current_scale >= 1.0f) {
                if (motor->ol_align_enable && !ctrl->startup.alignment.complete) {
                    ctrl->startup.phase = MOTOR_STARTUP_ANGLE_HOLD;
                } else {
                    MotorAlignment_MarkComplete(&ctrl->startup.alignment);
                    ctrl->startup.phase = MOTOR_STARTUP_DRAG_RAMP;
                }
            }
        } else if (ctrl->startup.phase == MOTOR_STARTUP_ANGLE_HOLD) {
            if (!Motor_Control_RunAlignment(ctrl, motor, dt)) {
                ctrl->startup.phase = MOTOR_STARTUP_DRAG_RAMP;
                ctrl->startup.drag_current_scale = 0.0f;
            }
        }

        if (ctrl->startup.phase == MOTOR_STARTUP_DRAG_RAMP) {
            bool sensored = Motor_Feedback_IsSensoredSource(Motor_Feedback_GetSourceDefault());
            Motor_Control_UpdateDragCurrent(ctrl, dt);
            Motor_Control_UpdateSpeed(motor, dt, sensored || Motor_Sensorless_IsTakeoverComplete());
            Motor_Control_StartupIf(ctrl, motor);
            Motor_Control_UpdateTheta(ctrl, motor, dt);
            if (sensored) {
                Motor_Control_PublishSensoredSpeed(motor);
            } else {
                Motor_Sensorless_Update(motor, dt);
            }
            if (Motor_Control_SpeedLoopReady(motor)) {
                Motor_SpeedLoop_Run(motor, dt);
            } else {
                /* 启动 Iq 正在接管时不得让未使用的 PI 输出在后台积分饱和。 */
                Motor_SpeedLoop_Reset();
            }
        }
        return Motor_Control_PublishTheta(motor, Motor_Control_RunIf(ctrl, motor, v_bus, dt));
    }

    /* V/F：电压软起 + 幅值上限；与转速同时斜坡（不做高压直流励磁）。 */
    Motor_SpeedLoop_Reset();
    Motor_Sensorless_Reset();
    Motor_Control_UpdateSoftCurrent(ctrl, motor, dt);
    Motor_Control_UpdateSpeed(motor, dt, true);
    Motor_Control_StartupVf(ctrl, motor);
    Motor_Control_UpdateTheta(ctrl, motor, dt);
    return Motor_Control_PublishTheta(motor, Motor_Control_RunVf(ctrl, motor, v_bus));
}

/**
 * @description: 查询是否处于预对齐。
 * @param {const motor_control_t} *ctrl  控制实例
 * @return {bool} true=对齐进行中
 * @author: lkc
 */
bool Motor_Control_IsAligning(const motor_control_t *ctrl)
{
    return ctrl != NULL && ctrl->startup.alignment.running;
}

/**
 * @description: 本次对齐已进行时间。
 * @param {const motor_control_t} *ctrl  控制实例
 * @return {float} 时长 (s)
 * @author: lkc
 */
float Motor_Control_GetAlignElapsed(const motor_control_t *ctrl)
{
    return ctrl == NULL ? 0.0f : ctrl->startup.alignment.elapsed;
}

/**
 * @description: HFI 是否在跑。
 * @param {const motor_control_t} *ctrl  控制实例
 * @return {bool} true=扫描中
 * @author: lkc
 */
bool Motor_Control_HfiIsRunning(const motor_control_t *ctrl)
{
    return ctrl != NULL && ctrl->startup.hfi_start.running;
}

/**
 * @description: HFI 是否已成功。
 * @param {const motor_control_t} *ctrl  控制实例
 * @return {bool} true=识别成功
 * @author: lkc
 */
bool Motor_Control_HfiSucceeded(const motor_control_t *ctrl)
{
    return ctrl != NULL && ctrl->startup.hfi_start.succeeded;
}

/**
 * @description: HFI 识别电角度。
 * @param {const motor_control_t} *ctrl  控制实例
 * @return {float} 电角度 (rad)
 * @author: lkc
 */
float Motor_Control_HfiGetAngle(const motor_control_t *ctrl)
{
    return ctrl == NULL ? 0.0f : ctrl->startup.hfi_start.angle;
}

/**
 * @description: HFI 可信度。
 * @param {const motor_control_t} *ctrl  控制实例
 * @return {float} 可信度 [0,1]
 * @author: lkc
 */
float Motor_Control_HfiGetConfidence(const motor_control_t *ctrl)
{
    return ctrl == NULL ? 0.0f : ctrl->startup.hfi_start.confidence;
}

/**
 * @description: 故障仲裁：已锁存故障时强制 SAFE，禁止 DQ 覆盖安全输出。
 * @param {bool} fault_latched  保护是否已锁存
 * @param {const motor_control_result_t} *proposed  Control_Step 提议
 * @return {motor_control_result_t} 最终输出
 * @author: lkc
 */
motor_control_result_t Motor_Control_ArbitrateOutput(bool fault_latched, const motor_control_result_t *proposed)
{
    motor_control_result_t none;

    if (fault_latched) {
        return Motor_Control_ResultSafe();
    }
    if (proposed == NULL) {
        memset(&none, 0, sizeof(none));
        none.action = MOTOR_CONTROL_ACTION_NONE;
        return none;
    }
    return *proposed;
}

#ifdef __cplusplus
}
#endif
