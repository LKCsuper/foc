/* 控制状态机：电压限幅、速度斜坡、对齐、I/F、V/F、反馈选择、诊断步与故障仲裁。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_control.h"
#include "motor_param.h"
#include "motor_current.h"
#include "motor_feedback.h"
#include "motor_timing.h"
#include "motor_math.h"

/**
 * @description: 控制行为测试不采集平台耗时；计时与逐拍记录由独立timing测试覆盖。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Timing_ProfileIsActive(void) { return false; }
/**
 * @description: 主机测试替身：查询本拍细分计时是否有效。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Timing_ProfileDetailedIsActive(void) { return false; }
/**
 * @description: 主机测试替身：读取分段计时用的当前时间戳。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 本实现固定返回 0U
 */
uint32_t Motor_Timing_ProfileTimeUs(void) { return 0U; }
/**
 * @description: 主机测试替身：把一个阶段从 start_us 到现在的耗时累进当前窗口。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_timing_stage_t} stage 兼容参数，本实现不使用
 * @param {uint32_t} start_us 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Timing_ProfileRecord(motor_timing_stage_t stage, uint32_t start_us)
{
    (void)stage;
    (void)start_us;
}

static float s_last_current_theta;
static float s_last_id_fw;
static int s_current_step_calls;
static int s_sensorless_updates;
static bool s_speed_fb_valid;
static float s_speed_fb_rpm;
static float s_speed_iq_out;
static bool s_last_speed_loop_active;
static float s_last_speed_iq_target;
static bool s_last_alignment_running;
static float s_last_soft_current_scale;
static float s_last_if_iq_scale;
static float s_speed_prime_iq;
static float s_sensorless_correction;
static bool s_takeover_complete;
static bool s_complete_during_update;
static unsigned s_handoff_marks;
/**
 * @description: 主机测试替身：标记下一拍为交接完成拍。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Timing_HandoffMark(void) { s_handoff_marks++; }
static float s_last_frame_correction;

/**
 * @description: 主机测试替身：计算两角度之差并限制在 [-π, π]。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} a a 参数
 * @param {float} b b 参数
 * @return {float} 测试记录值 delta
 */
float utils_angle_difference_rad(float a, float b)
{
    float delta = a - b;
    while (delta > 3.14159265f) delta -= 6.2831853f;
    while (delta < -3.14159265f) delta += 6.2831853f;
    return delta;
}

/**
 * @description: 主机测试替身：清除默认实例速度 PI 状态
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_SpeedLoop_Reset(void)
{
    s_speed_iq_out = 0.0f;
}

/**
 * @description: 主机测试替身：复位默认实例观测器，并声明速度反馈无效。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Sensorless_Reset(void)
{
}

/**
 * @description: 主机测试替身：判断默认实例开环角到观测角的接管是否全部完成。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_takeover_complete
 */
bool Motor_Sensorless_IsTakeoverComplete(void)
{
    return s_takeover_complete;
}

/**
 * @description: 主机测试替身：用当前等效 Iq 预置默认实例速度 PI
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} iq Q 轴电流 (A)
 * @return {void} 无
 */
void Motor_SpeedLoop_PrimeIq(float iq)
{
    s_speed_prime_iq = iq;
}

/**
 * @description: 主机测试替身：默认实例每个 FOC 周期更新 Ortega/PLL/接管，并落地速度环/保护副作用。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 电机实例
 * @param {float} dt 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Sensorless_Update(MOTOR_PARAM_S *motor, float dt)
{
    motor->ol_theta += s_sensorless_correction;
    if (s_complete_during_update) s_takeover_complete = true;
    (void)dt;
    s_sensorless_updates++;
}

/**
 * @description: 主机测试替身：更新默认实例的真实机械转速反馈
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} rpm 机械转速 (RPM)
 * @param {bool} valid valid 参数
 * @return {void} 无
 */
void Motor_SpeedFeedback_Set(float rpm, bool valid)
{
    s_speed_fb_rpm = rpm;
    s_speed_fb_valid = valid;
    if (!valid) {
        s_speed_iq_out = 0.0f;
    }
}

/**
 * @description: 主机测试替身：运行默认实例速度 PI 外环（委托 State 版本）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 电机实例
 * @param {float} dt 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_SpeedLoop_Run(MOTOR_PARAM_S *motor, float dt)
{
    (void)dt;
    if (motor != NULL && motor->speed_enable && s_speed_fb_valid) {
        s_speed_iq_out = 1.25f; /* 模拟速度 PI 输出 Iq */
    } else {
        s_speed_iq_out = 0.0f;
    }
}

/**
 * @description: 主机测试替身：允许或禁止默认实例速度 PI（由 ISR 在接管门槛处设置）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {bool} allowed allowed 参数
 * @return {void} 无
 */
void Motor_SpeedLoop_SetRunAllowed(bool allowed)
{
    if (!allowed) {
        s_speed_iq_out = 0.0f;
    }
}

/**
 * @description: 主机测试替身：查询默认实例速度反馈是否可信
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_speed_fb_valid
 */
bool Motor_SpeedLoop_IsFeedbackValid(void)
{
    return s_speed_fb_valid;
}

/**
 * @description: 主机测试替身：读取默认实例实测机械转速
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_speed_fb_rpm
 */
float Motor_SpeedLoop_GetMeasuredRpm(void)
{
    return s_speed_fb_rpm;
}

/**
 * @description: 主机测试替身：读取默认实例当前 Iq 目标
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_speed_iq_out
 */
float Motor_SpeedLoop_GetIqTarget(void)
{
    return s_speed_iq_out;
}

/**
 * @description: 主机测试替身：禁用制动时，把最终电压或电流限制成不与转速命令反向。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} command_rpm command_rpm，单位 RPM
 * @param {float} output 输出数据
 * @param {bool} allow_braking allow_braking 参数
 * @return {float} 限幅后的输出，单位与 output 相同
 */
float Motor_Speed_LimitNoBrakeOutput(float command_rpm, float output, bool allow_braking)
{
    if (allow_braking) {
        return output;
    }
    if (command_rpm > 0.0f && output < 0.0f) {
        return 0.0f;
    }
    if (command_rpm < 0.0f && output > 0.0f) {
        return 0.0f;
    }
    return output;
}

/**
 * @description: 主机测试替身：获取默认实例本周期 Park 变换应使用的电角度。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} open_loop_angle open_loop_angle 参数
 * @return {float} 测试记录值 open_loop_angle + 9.0f
 */
float Motor_Sensorless_GetControlAngle(float open_loop_angle)
{
    return open_loop_angle + 9.0f; /* 无感路径可区分 */
}

static bool s_enc_zero;
static float s_enc_elec = 1.57f;
static float s_enc_rpm = 321.0f;
static bool s_enc_speed_ok = true;
static int s_enc_angle_reads;

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
    s_enc_angle_reads++;
    return s_enc_elec;
}

/**
 * @description: 主机测试替身：读取默认实例最新已验收的机械角（不使用 PLL 预测位置）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 s_enc_elec
 */
float Motor_Encoder_GetMechAngle(void)
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
 * @return {bool} 测试记录值 s_enc_speed_ok
 */
bool Motor_Encoder_IsSpeedValid(void)
{
    return s_enc_speed_ok;
}

/**
 * @description: 主机测试替身：默认实例电角是否有效
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Hall_AngleIsAngleValid(void)
{
    return false;
}

/**
 * @description: 主机测试替身：默认实例电角 (rad)；angle_valid=false 时值勿单独使用
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0.0f
 */
float Motor_Hall_AngleGetTheta(void)
{
    return 0.0f;
}

/**
 * @description: 主机测试替身：默认实例机械转速 (RPM)
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0.0f
 */
float Motor_Hall_AngleGetMechRpm(void)
{
    return 0.0f;
}

/**
 * @description: 主机测试替身：默认实例转速是否有效
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Hall_AngleIsSpeedValid(void)
{
    return false;
}

/**
 * @description: 主机测试替身：读取默认实例最近单边沿原始机械转速。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0.0f
 */
float Motor_Hall_AngleGetRawMechRpm(void)
{
    return 0.0f;
}

/**
 * @description: 主机测试替身：读取默认实例 Hall-PLL 机械转速，供控制环使用。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0.0f
 */
float Motor_Hall_AngleGetFastMechRpm(void)
{
    return 0.0f;
}

/**
 * @description: 主机测试替身：读取默认实例距最近一次霍尔边沿的时间。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0.0f
 */
float Motor_Hall_AngleGetEdgeAge(void)
{
    return 0.0f;
}

/**
 * @description: 主机测试替身：读取默认实例最近一次有效霍尔状态。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {int} 本实现固定返回 0
 */
int Motor_Hall_AngleGetState(void)
{
    return 0;
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
 * @description: 主机测试替身：霍尔标定期间 ISR 读取本周期电角度与 d/q 电流目标
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} m 兼容参数，本实现不使用
 * @param {float *} theta_rad 兼容参数，本实现不使用
 * @param {float *} id_target 兼容参数，本实现不使用
 * @param {float *} iq_target 兼容参数，本实现不使用
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Hall_DetectGetTargets(MOTOR_PARAM_S *m, float *theta_rad, float *id_target, float *iq_target)
{
    (void)m;
    (void)theta_rad;
    (void)id_target;
    (void)iq_target;
    return false;
}

/**
 * @description: 主机测试替身：读取编码器对齐阶段的固定电角和 Id 目标。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float *} theta_rad 兼容参数，本实现不使用
 * @param {float *} id_target 兼容参数，本实现不使用
 * @param {float *} iq_target 兼容参数，本实现不使用
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Encoder_DetectGetTargets(float *theta_rad, float *id_target, float *iq_target)
{
    (void)theta_rad;
    (void)id_target;
    (void)iq_target;
    return false;
}

/**
 * @description: 主机测试替身：三相电流变换到 αβ，再变换到 d/q，写回反馈字段。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @param {float} theta 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Current_FeedbackUpdateDq(MOTOR_PARAM_S *motor, float theta)
{
    (void)motor;
    (void)theta;
}

/**
 * @description: 主机测试替身：运行 d/q 双轴电流 PI，并做矢量限幅。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @param {float} id_target 兼容参数，本实现不使用
 * @param {float} iq_target 兼容参数，本实现不使用
 * @param {float} dt 兼容参数，本实现不使用
 * @param {float} v_limit 兼容参数，本实现不使用
 * @return {motor_dq_voltage_t} 测试记录值 v
 */
motor_dq_voltage_t Motor_Current_RunPi(MOTOR_PARAM_S *motor, float id_target, float iq_target, float dt, float v_limit)
{
    motor_dq_voltage_t v = {0.0f, 0.0f};
    (void)motor;
    (void)id_target;
    (void)iq_target;
    (void)dt;
    (void)v_limit;
    return v;
}

static int s_ff_calls;

/**
 * @description: 主机测试替身：按电角速度叠加 PMSM 交叉解耦与反电势前馈，再做矢量限幅。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_dq_voltage_t *} v v 参数
 * @param {float} omega_e omega_e 参数
 * @param {float} inductance 兼容参数，本实现不使用
 * @param {float} flux flux 参数
 * @param {float} id_ref 兼容参数，本实现不使用
 * @param {float} iq_ref 兼容参数，本实现不使用
 * @param {float} v_limit 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Current_ApplySpeedFeedforward(motor_dq_voltage_t *v, float omega_e, float inductance, float flux,
                                         float id_ref, float iq_ref, float v_limit)
{
    (void)inductance;
    (void)id_ref;
    (void)iq_ref;
    (void)v_limit;
    s_ff_calls++;
    if (v != NULL) {
        v->vq += omega_e * flux;
    }
}

/**
 * @description: 主机测试替身：电流环一步：反馈变换、取目标、可选坐标交接、PI 及矢量限幅。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @param {float} theta 强制电角度 (rad)
 * @param {const motor_current_target_ctx_t *} ctx ctx 参数
 * @param {float} dt 兼容参数，本实现不使用
 * @param {float} v_limit 兼容参数，本实现不使用
 * @param {float *} id_target_out id_target_out 参数
 * @param {float *} iq_target_out iq_target_out 参数
 * @return {motor_dq_voltage_t} 测试记录值 v
 */
motor_dq_voltage_t Motor_Current_Step(MOTOR_PARAM_S *motor, float theta, const motor_current_target_ctx_t *ctx,
                                      float dt, float v_limit, float *id_target_out, float *iq_target_out)
{
    motor_dq_voltage_t v = {0.1f, 0.2f};
    (void)motor;
    (void)dt;
    (void)v_limit;
    s_current_step_calls++;
    s_last_id_fw = ctx != NULL ? ctx->id_fw : 0.0f;
    s_last_current_theta = theta;
    s_last_frame_correction = ctx != NULL ? ctx->frame_correction : 0.0f;
    s_last_alignment_running = ctx != NULL && ctx->alignment_running;
    s_last_soft_current_scale = ctx != NULL ? ctx->soft_current_scale : 0.0f;
    s_last_if_iq_scale = ctx != NULL ? ctx->if_iq_scale : 0.0f;
    s_last_speed_loop_active = (ctx != NULL) && ctx->speed_loop_active;
    s_last_speed_iq_target = (ctx != NULL) ? ctx->speed_iq_target : 0.0f;
    if (id_target_out != NULL) {
        *id_target_out = 0.0f;
    }
    if (iq_target_out != NULL) {
        *iq_target_out = 0.0f;
    }
    return v;
}

/**
 * @description: 主机测试替身：Clarke 变换，将三相电流投影到 αβ 静止坐标系。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} Ia 兼容参数，本实现不使用
 * @param {float} Ib 兼容参数，本实现不使用
 * @param {float} Ic 兼容参数，本实现不使用
 * @param {float *} Ialpha Ialpha 参数
 * @param {float *} Ibeta Ibeta 参数
 * @return {void} 无
 */
void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
    (void)Ia;
    (void)Ib;
    (void)Ic;
    if (Ialpha != NULL) {
        *Ialpha = 0.0f;
    }
    if (Ibeta != NULL) {
        *Ibeta = 0.0f;
    }
}

/**
 * @description: 对照每拍强制规整，验证缓存对直接字段修改与非有限值的行为。
 * @detail: V/F 路径隔离观测器测试替身；逐字段覆盖、重复拍与实例复位均比较完整电机状态。
 * @return {void} 无
 */
static void test_normalize_cache(void)
{
    motor_control_t cached;
    MOTOR_PARAM_S motor;
    const uint32_t values[] = {0U, 0x80000000U, 0xbf800000U, 0x3f800000U, 0x41a00000U, 0x7fc12345U};
    unsigned i;
    unsigned repeat;
    unsigned submitted;

    for (submitted = 0U; submitted < 2U; ++submitted) {

        memset(&motor, 0, sizeof(motor));
        Motor_Control_Init(&cached);
        Motor_Feedback_SetSourceDefault(MOTOR_FEEDBACK_SRC_OPENLOOP);
        motor.control_config_revision = 11U;
        motor.ol_enable = true;
        motor.ol_target_rpm = 300.0f;
        motor.ol_max_rpm = 1000.0f;
        motor.ol_vq_limit = 6.0f;
        motor.ol_pole_pairs = 7.0f;
        if (submitted)
            (void)Motor_Control_StepSubmitted(&cached, &motor, 24.0f, 1.0f / 15000.0f, false);
        else
            (void)Motor_Control_Step(&cached, &motor, 24.0f, 1.0f / 15000.0f, false);
        assert(cached.normalized_config_valid);

#define MOTOR_NORMALIZE_FIELD(name)                                                                                   \
    for (i = 0U; i < sizeof(values) / sizeof(values[0]); ++i) {                                                         \
        uint32_t bits = values[i];                                                                                    \
        if (sizeof(motor.name) == sizeof(bool)) bits = i % 2U;                                                         \
        if (sizeof(motor.name) == sizeof(uint16_t)) bits = i * 13U;                                                    \
        if (submitted) motor.control_config_revision++;                                                              \
        memcpy(&motor.name, &bits, sizeof(motor.name));                                                                \
        for (repeat = 0U; repeat < 2U; ++repeat) {                                                                     \
            MOTOR_PARAM_S reference_motor = motor;                                                                   \
            motor_control_t reference = cached;                                                                      \
            motor_control_result_t expected;                                                                         \
            motor_control_result_t actual;                                                                           \
            reference.normalized_config_valid = false;                                                               \
            expected = Motor_Control_Step(&reference, &reference_motor, 24.0f, 1.0f / 15000.0f, false);               \
            actual = submitted ? Motor_Control_StepSubmitted(&cached, &motor, 24.0f, 1.0f / 15000.0f, false)             \
                               : Motor_Control_Step(&cached, &motor, 24.0f, 1.0f / 15000.0f, false);                               \
            assert(memcmp(&expected, &actual, sizeof(actual)) == 0);                                                  \
            assert(memcmp(&reference_motor, &motor, sizeof(motor)) == 0);                                              \
            assert(memcmp(&reference.startup, &cached.startup, sizeof(cached.startup)) == 0);                          \
        }                                                                                                            \
    }
#include "motor_control_normalize_fields.inc"
#undef MOTOR_NORMALIZE_FIELD

        Motor_Control_Init(&cached);
        assert(!cached.normalized_config_valid);
    }
}

/**
 * @description: 已提交快路径仍检查动态目标，并在实例变化、未提交或复位时回退。
 * @return {void} 无；断言失败终止测试
 */
static void test_submitted_boundaries(void)
{
    motor_control_t control;
    MOTOR_PARAM_S motor;
    MOTOR_PARAM_S other;
    const float targets[] = {2000.0f, -2000.0f, NAN, -0.0f, 123.0f};
    unsigned i;
    memset(&motor, 0, sizeof(motor));
    Motor_Control_Init(&control);
    Motor_Feedback_SetSourceDefault(MOTOR_FEEDBACK_SRC_OPENLOOP);
    motor.ol_enable = true;
    motor.ol_target_rpm = 300.0f;
    motor.ol_max_rpm = 1000.0f;
    motor.ol_pole_pairs = 7.0f;
    motor.ol_vq_limit = 6.0f;
    motor.control_config_revision = UINT32_MAX;
    (void)Motor_Control_StepSubmitted(&control, &motor, 24.0f, 1.0f / 15000.0f, false);
    for (i = 0U; i < sizeof(targets) / sizeof(targets[0]); ++i) {
        motor_control_t reference = control;
        MOTOR_PARAM_S expected = motor;
        motor_control_result_t a, b;
        motor.ol_target_rpm = expected.ol_target_rpm = targets[i];
        a = Motor_Control_Step(&reference, &expected, 24.0f, 1.0f / 15000.0f, false);
        b = Motor_Control_StepSubmitted(&control, &motor, 24.0f, 1.0f / 15000.0f, false);
        assert(memcmp(&a, &b, sizeof(a)) == 0);
        assert(memcmp(&expected, &motor, sizeof(motor)) == 0);
    }
    /* 回绕后的非零版本必须重新规整；原始接口在非零版本下也识别直接写入。 */
    motor.control_config_revision = 1U;
    motor.ol_align_current = -2.0f;
    (void)Motor_Control_StepSubmitted(&control, &motor, 24.0f, 1.0f / 15000.0f, false);
    assert(motor.ol_align_current == 0.0f && control.normalized_revision == 1U);
    other = motor;
    other.ol_align_current = -3.0f;
    (void)Motor_Control_StepSubmitted(&control, &other, 24.0f, 1.0f / 15000.0f, false);
    assert(other.ol_align_current == 0.0f && control.normalized_owner == &other);
    other.control_config_revision = 0U;
    other.ol_align_current = -4.0f;
    (void)Motor_Control_StepSubmitted(&control, &other, 24.0f, 1.0f / 15000.0f, false);
    assert(other.ol_align_current == 0.0f);
    other.ol_align_current = -5.0f;
    (void)Motor_Control_StepSubmitted(&control, &other, 24.0f, 1.0f / 15000.0f, false);
    assert(other.ol_align_current == 0.0f);
    assert(Motor_Control_StepSubmitted(NULL, &other, 24.0f, 0.001f, false).action == MOTOR_CONTROL_ACTION_SAFE);
    assert(Motor_Control_StepSubmitted(&control, NULL, 24.0f, 0.001f, false).action == MOTOR_CONTROL_ACTION_SAFE);
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    motor_control_t a;
    motor_control_t b;
    MOTOR_PARAM_S motor;
    motor_control_result_t result;
    float limit;
    unsigned i;

    memset(&motor, 0, sizeof(motor));
    Motor_Control_Init(&a);
    Motor_Control_Init(&b);

#ifdef MOTOR_CONTROL_HOLD_ONLY
    /* 2804 关闭编码器速度电压模式，但位置保持的直接 Vq 仍必须生效。 */
    a.startup.phase = MOTOR_STARTUP_DRAG_RAMP;
    a.startup.soft_current_scale = 1.0f;
    a.startup.drag_current_scale = 1.0f;
    motor.ol_enable = true;
    motor.ol_mode = MOTOR_OL_MODE_IF;
    motor.ol_pole_pairs = 7.0f;
    motor.ol_vq_limit = 6.0f;
    motor.speed_enable = true;
    motor.position_voltage_hold = true;
    motor.position_hold_v = 1.25f;
    s_enc_zero = true;
    s_enc_speed_ok = true;
    s_enc_elec = 0.75f;
    s_current_step_calls = 0;
    Motor_Feedback_SetSourceDefault(MOTOR_FEEDBACK_SRC_ENCODER);
    result = Motor_Control_Step(&a, &motor, 16.0f, 0.001f, false);
    assert(result.action == MOTOR_CONTROL_ACTION_OUTPUT_DQ);
    assert(fabsf(result.theta - s_enc_elec) < 1e-5f);
    assert(fabsf(result.vq - 1.25f) < 1e-5f);
    assert(s_current_step_calls == 0);
    puts("test_motor_control_hold_2804: pass");
    return 0;
#endif

    motor.ol_vq_limit = 4.0f;
    limit = Motor_Control_GetVoltageLimit(&motor, 24.0f);
    assert(fabsf(limit - 4.0f) < 1e-4f);
    limit = Motor_Control_GetVoltageLimit(&motor, 3.0f);
    assert(fabsf(limit - 3.0f / sqrtf(3.0f)) < 1e-4f);
    motor.ol_vq_limit = 20.0f;
    limit = Motor_Control_GetVoltageLimit(&motor, 24.0f);
    assert(fabsf(limit - 24.0f / sqrtf(3.0f)) < 1e-4f);
    motor.ol_vq_limit = 10.0f;
    limit = Motor_Control_GetVoltageLimit(&motor, 24.0f);
    assert(fabsf(limit - 10.0f) < 1e-4f);

    motor.ol_enable = true;
    motor.ol_target_rpm = 500.0f;
    assert(Motor_Control_IsReady(&motor, 24.0f, false));
    assert(!Motor_Control_IsReady(&motor, 0.5f, false));
    motor.ol_target_rpm = 0.0f;
    assert(!Motor_Control_IsReady(&motor, 24.0f, false));
    motor.speed_enable = true;
    assert(Motor_Control_IsReady(&motor, 24.0f, false));
    motor.speed_enable = false;
    assert(Motor_Control_IsReady(&motor, 24.0f, true));

    motor.ol_mode = MOTOR_OL_MODE_IF;
    motor.ol_target_rpm = 2000.0f;
    motor.ol_current_rpm = 0.0f;
    motor.ol_ramp_rpm_s = 1000.0f;
    motor.sensorless_openloop_max_rpm = 800.0f;
    for (i = 0; i < 5U; ++i) {
        Motor_Control_UpdateSpeed(&motor, 0.1f, false);
    }
    assert(fabsf(motor.ol_current_rpm - 500.0f) < 1e-3f);
    for (i = 0; i < 20U; ++i) {
        Motor_Control_UpdateSpeed(&motor, 0.1f, false);
    }
    assert(fabsf(motor.ol_current_rpm - 800.0f) < 1e-3f);
    Motor_Control_UpdateSpeed(&motor, 0.1f, true);
    assert(motor.ol_current_rpm > 800.0f);

    /* 速度环给定按 VESC 25000 ERPM/s / 极对数，不走 olramp 50。 */
    motor.speed_enable = true;
    motor.ol_target_rpm = 3000.0f;
    motor.ol_current_rpm = 0.0f;
    motor.ol_ramp_rpm_s = 50.0f;
    motor.ol_pole_pairs = 2.0f;
    Motor_Control_UpdateSpeed(&motor, 0.1f, true);
    assert(fabsf(motor.ol_current_rpm - 1250.0f) < 1e-2f);
    motor.speed_enable = false;

    assert(!Motor_Control_IsAligning(&a));
    a.startup.alignment.running = true;
    a.startup.alignment.elapsed = 0.12f;
    assert(Motor_Control_IsAligning(&a));
    assert(!Motor_Control_IsAligning(&b));
    assert(fabsf(Motor_Control_GetAlignElapsed(&a) - 0.12f) < 1e-6f);

    motor.ol_enable = true;
    motor.ol_target_rpm = 500.0f;
    motor.ol_current_rpm = 300.0f;
    result = Motor_Control_Step(&a, &motor, 0.5f, 0.001f, false);
    assert(result.action == MOTOR_CONTROL_ACTION_SAFE);
    assert(fabsf(motor.ol_current_rpm) < 1e-6f);

    motor.ol_pole_pairs = 0.0f;
    motor.sensorless_phase_tolerance = 3.0f;
    motor.ol_align_current = -1.0f;
    motor.ol_align_time = -2.0f;
    motor.hfi_start_voltage = -1.0f;
    motor.hfi_start_cycles = 1U;
    motor.hfi_start_min_confidence = 2.0f;
    Motor_Control_NormalizeConfig(&motor);
    assert(motor.ol_pole_pairs >= 1.0f);
    assert(motor.sensorless_phase_tolerance <= 1.50f);
    assert(motor.ol_align_current == 0.0f && motor.ol_align_time == 0.0f);
    assert(motor.hfi_start_voltage == 0.0f && motor.hfi_start_cycles == 2U);
    assert(motor.hfi_start_min_confidence == 1.0f);
    motor.ol_align_current = 0.5f;
    motor.ol_align_time = 0.1f;
    motor.hfi_start_voltage = 0.2f;
    motor.hfi_start_cycles = 40U;
    motor.hfi_start_min_confidence = -1.0f;
    Motor_Control_NormalizeConfig(&motor);
    assert(motor.ol_align_current == 0.5f && motor.ol_align_time == 0.1f);
    assert(motor.hfi_start_voltage == 0.2f && motor.hfi_start_cycles == 32U);
    assert(motor.hfi_start_min_confidence == 0.0f);

    /* 故障仲裁：锁存时强制 SAFE，未锁存保留 DQ。 */
    {
        motor_control_result_t proposed;
        motor_control_result_t out;

        proposed.action = MOTOR_CONTROL_ACTION_OUTPUT_DQ;
        proposed.vd = 1.0f;
        proposed.vq = 2.0f;
        proposed.theta = 0.5f;
        out = Motor_Control_ArbitrateOutput(true, &proposed);
        assert(out.action == MOTOR_CONTROL_ACTION_SAFE);
        out = Motor_Control_ArbitrateOutput(false, &proposed);
        assert(out.action == MOTOR_CONTROL_ACTION_OUTPUT_DQ);
        assert(fabsf(out.vq - 2.0f) < 1e-6f);
    }

    /* V/F：电压和转速都连续爬升，不跳到 min_rpm。 */
    {
        motor_control_t ctrl;
        MOTOR_PARAM_S m;
        float theta0;

        memset(&m, 0, sizeof(m));
        Motor_Control_Init(&ctrl);
        m.ol_enable = true;
        m.ol_mode = MOTOR_OL_MODE_VF;
        m.ol_target_rpm = 60.0f;
        m.ol_current_rpm = 0.0f;
        m.ol_pole_pairs = 2.0f;
        m.ol_min_rpm = 20.0f;
        m.ol_ramp_rpm_s = 100.0f;
        m.ol_v_boost = 3.0f;
        m.ol_kv_f = 0.0f;
        m.ol_vq_limit = 18.0f;
        m.ol_theta = 0.0f;
        ctrl.startup.soft_current_scale = 0.5f;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.1f, false);
        assert(result.action == MOTOR_CONTROL_ACTION_OUTPUT_DQ);
        assert(result.vq > 0.0f && result.vq < 3.0f);
        assert(m.ol_current_rpm > 0.0f);
        assert(m.ol_current_rpm < m.ol_min_rpm);
        ctrl.startup.soft_current_scale = 1.0f;
        theta0 = m.ol_theta;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.1f, false);
        assert(fabsf(result.vq - 3.0f) < 1e-4f);
        assert(fabsf(m.ol_theta - theta0) > 1e-4f);
    }

    /* I/F 未开对齐：跳过定角灌 Id，同一拍进入拖转，转速从 0 斜坡、不跳到 min_rpm。 */
    {
        motor_control_t ctrl;
        MOTOR_PARAM_S m;
        float theta0;

        memset(&m, 0, sizeof(m));
        Motor_Control_Init(&ctrl);
        m.ol_enable = true;
        m.ol_mode = MOTOR_OL_MODE_IF;
        m.ol_target_rpm = 40.0f;
        m.ol_current_rpm = 0.0f;
        m.ol_pole_pairs = 2.0f;
        m.ol_min_rpm = 15.0f;
        m.ol_ramp_rpm_s = 100.0f;
        m.ol_align_enable = false;
        m.hfi_start_enable = false;
        m.ol_iq_set = 0.6f;
        m.ol_vq_limit = 18.0f;
        m.ol_theta = 0.4f;
        theta0 = m.ol_theta;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(result.action == MOTOR_CONTROL_ACTION_OUTPUT_DQ);
        assert(ctrl.startup.phase == MOTOR_STARTUP_DRAG_RAMP);
        assert(fabsf(m.ol_theta - theta0) > 1e-6f);
        assert(fabsf(m.theta - result.theta) < 1e-6f);
        assert(m.ol_current_rpm > 0.0f && m.ol_current_rpm < m.ol_min_rpm);
        assert(ctrl.startup.drag_current_scale > 0.0f && ctrl.startup.drag_current_scale < 0.01f);
        assert(!s_last_alignment_running);
        assert(s_last_soft_current_scale > 0.0f && s_last_soft_current_scale < 0.01f);
    }

    /* I/F 开对齐：电流爬升阶段电角与转速保持不动。 */
    {
        motor_control_t ctrl;
        MOTOR_PARAM_S m;
        float theta0;

        memset(&m, 0, sizeof(m));
        Motor_Control_Init(&ctrl);
        m.ol_enable = true;
        m.ol_mode = MOTOR_OL_MODE_IF;
        m.ol_target_rpm = 40.0f;
        m.ol_pole_pairs = 2.0f;
        m.ol_ramp_rpm_s = 100.0f;
        m.ol_align_enable = true;
        m.ol_align_current = 0.5f;
        m.ol_align_time = 0.3f;
        m.hfi_start_enable = false;
        m.ol_iq_set = 0.6f;
        m.ol_vq_limit = 18.0f;
        m.ol_theta = 0.4f;
        theta0 = m.ol_theta;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(result.action == MOTOR_CONTROL_ACTION_OUTPUT_DQ);
        assert(ctrl.startup.phase == MOTOR_STARTUP_CURRENT_RAMP);
        assert(fabsf(m.ol_theta - theta0) < 1e-6f);
        assert(fabsf(m.ol_current_rpm) < 1e-6f);
        assert(s_last_alignment_running);
    }

    /* 对齐保持只在电流爬升完成后计时，并按完整 align_current 保持。 */
    {
        motor_control_t ctrl;
        MOTOR_PARAM_S m;
        unsigned step;

        memset(&m, 0, sizeof(m));
        Motor_Control_Init(&ctrl);
        m.ol_enable = true;
        m.ol_mode = MOTOR_OL_MODE_IF;
        m.ol_target_rpm = 40.0f;
        m.ol_max_rpm = 1000.0f;
        m.ol_pole_pairs = 2.0f;
        m.ol_min_rpm = 15.0f;
        m.ol_ramp_rpm_s = 10.0f;
        m.ol_align_enable = true;
        m.ol_align_current = 0.5f;
        m.ol_align_time = 0.3f;
        m.ol_iq_set = 0.6f;
        m.ol_vq_limit = 18.0f;
        for (step = 0; step < 799U; ++step) {
            (void)Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        }
        assert(ctrl.startup.phase == MOTOR_STARTUP_CURRENT_RAMP);
        assert(!ctrl.startup.alignment.running);
        assert(ctrl.startup.alignment.elapsed == 0.0f);
        (void)Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(ctrl.startup.phase == MOTOR_STARTUP_ANGLE_HOLD);
        (void)Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(ctrl.startup.alignment.running);
        assert(s_last_alignment_running);
        assert(fabsf(s_last_soft_current_scale - 1.0f) < 1e-6f);
        for (step = 0; step < 400U && ctrl.startup.phase != MOTOR_STARTUP_DRAG_RAMP; ++step) {
            (void)Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        }
        assert(ctrl.startup.phase == MOTOR_STARTUP_DRAG_RAMP);
        assert(step >= 299U && step <= 302U);
        assert(m.ol_current_rpm > 0.0f && m.ol_current_rpm < m.ol_min_rpm);
    }

    /* 运行中切换模式会在控制周期内完整复位启动状态。 */
    {
        motor_control_t ctrl;
        MOTOR_PARAM_S m;

        memset(&m, 0, sizeof(m));
        Motor_Control_Init(&ctrl);
        m.ol_enable = true;
        m.ol_mode = MOTOR_OL_MODE_VF;
        m.ol_target_rpm = 100.0f;
        m.ol_max_rpm = 1000.0f;
        m.ol_pole_pairs = 2.0f;
        m.ol_ramp_rpm_s = 100.0f;
        m.ol_vq_limit = 18.0f;
        ctrl.startup.soft_current_scale = 1.0f;
        (void)Motor_Control_Step(&ctrl, &m, 24.0f, 0.01f, false);
        m.ol_mode = MOTOR_OL_MODE_IF;
        m.ol_align_enable = false;
        m.hfi_start_enable = false;
        (void)Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(ctrl.startup.phase == MOTOR_STARTUP_DRAG_RAMP);
        assert(ctrl.startup.soft_current_scale < 0.01f);
        assert(ctrl.startup.drag_current_scale > 0.0f && ctrl.startup.drag_current_scale < 0.01f);
        assert(m.ol_current_rpm > 0.0f && m.ol_current_rpm < 1.0f);
    }

    /* 有感电流+速度环：encoder 角/速有效且 speed_enable → 电流环用速度 Iq */
    {
        motor_control_t ctrl;
        MOTOR_PARAM_S m;

        memset(&m, 0, sizeof(m));
        Motor_Control_Init(&ctrl);
        /* 本用例只验证有感反馈链，直接置于启动完成后的拖转阶段。 */
        ctrl.startup.phase = MOTOR_STARTUP_DRAG_RAMP;
        ctrl.startup.soft_current_scale = 1.0f;
        ctrl.startup.drag_current_scale = 1.0f;
        m.ol_enable = true;
        m.ol_mode = MOTOR_OL_MODE_IF;
        m.ol_target_rpm = 500.0f;
        m.ol_current_rpm = 100.0f;
        m.ol_pole_pairs = 2.0f;
        m.ol_align_enable = false;
        m.hfi_start_enable = false;
        m.ol_min_rpm = 50.0f;
        m.ol_ramp_rpm_s = 0.0f;
        m.ol_theta = 0.25f;
        m.speed_enable = true;
        m.speed_iq_limit = 0.20f;
        s_enc_zero = true;
        s_enc_elec = 1.57f;
        s_enc_rpm = 321.0f;
        s_enc_speed_ok = true;
        s_sensorless_updates = 0;
        s_enc_angle_reads = 0;
        s_last_current_theta = -1.0f;
        s_current_step_calls = 0;
        s_last_speed_loop_active = false;
        s_speed_fb_valid = false;
        Motor_Feedback_SetSourceDefault(MOTOR_FEEDBACK_SRC_ENCODER);
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(result.action == MOTOR_CONTROL_ACTION_OUTPUT_DQ);
        assert(fabsf(result.theta - 1.57f) < 1e-5f);
        assert(s_current_step_calls == 0); /* 电压模式不跑电流 PI */
        assert(fabsf(m.iq_target) < 1e-5f);
        assert(s_enc_angle_reads == 1);
        assert(s_sensorless_updates == 0);
        assert(s_speed_fb_valid);
        assert(fabsf(s_speed_fb_rpm - 321.0f) < 1e-3f);
        assert(fabsf(s_speed_prime_iq) < 1e-5f); /* 电机已在沿目标方向转动，不加启动预置 */
        /* Step 只置允许标志；Iq 由 foc pid 任务（此处手动 Run）在下一拍被电压路径读到。 */
        Motor_SpeedLoop_Run(&m, 0.001f);
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(s_current_step_calls == 0);
        assert(fabsf(m.iq_target - 1.25f) < 1e-5f);

        /* 关节扭矩后端即使当前电机默认使用编码器电压模式，也必须直接进入Iq电流环。 */
        m.joint_current_hold = true;
        m.joint_hold_iq = 0.15f;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(result.action == MOTOR_CONTROL_ACTION_OUTPUT_DQ);
        assert(s_current_step_calls == 1);
        assert(s_last_speed_loop_active);
        assert(fabsf(s_last_speed_iq_target - 0.15f) < 1e-5f);
        s_enc_zero = false;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(result.action == MOTOR_CONTROL_ACTION_SAFE);
        assert(fabsf(m.iq_target) < 1e-5f);
        s_enc_zero = true;
        m.joint_current_hold = false;
        s_current_step_calls = 0;

        /* 角无效则不发布速度，速度环不激活 */
        s_enc_zero = false;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(!s_speed_fb_valid);
        assert(!s_last_speed_loop_active);

        /* 编码器反转也留在速度 PI；电压速度模式不写入启动积分。 */
        s_enc_zero = true;
        s_enc_rpm = 28.0f;
        m.ol_target_rpm = -150.0f;
        m.ol_current_rpm = -150.0f;
        m.ol_min_rpm = 500.0f;
        m.ol_iq_set = 0.6f;
        s_speed_prime_iq = 0.0f;
        s_last_speed_loop_active = false;
        s_last_if_iq_scale = 0.0f;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(s_speed_fb_valid);
        assert(s_current_step_calls == 1);
        assert(fabsf(s_speed_prime_iq) < 1e-5f);

        /* 编码器电压速度模式使用前馈/助推，不向速度 PI 积分写入启动预置。 */
        Motor_Control_Init(&ctrl);
        ctrl.startup.phase = MOTOR_STARTUP_DRAG_RAMP;
        ctrl.startup.soft_current_scale = 1.0f;
        ctrl.startup.drag_current_scale = 1.0f;
        m.ol_target_rpm = 30.0f;
        m.ol_current_rpm = 30.0f;
        m.ol_max_rpm = 1800.0f;
        m.ol_min_rpm = 72.0f;
        m.ol_iq_set = 0.6f;
        m.ol_vq_limit = 18.0f;
        m.speed_kp = 0.001f;
        m.speed_iq_limit = 0.20f;
        s_enc_rpm = 0.0f;
        s_speed_prime_iq = 0.0f;
        s_last_speed_loop_active = false;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(s_current_step_calls == 1);
        assert(fabsf(s_speed_prime_iq) < 1e-5f);

        s_enc_rpm = 25.0f;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(s_current_step_calls == 1);
        assert(fabsf(s_speed_prime_iq) < 1e-5f); /* 接管后不重复预置 */

        /* 齿槽助推：10 RPM 内保持 0.08 A，15 RPM 衰减到 0.04 A，20 RPM 退出。 */
        {
            float ff_30 = MOTOR_ENCODER_SPEED_FF_BOOST_V + MOTOR_ENCODER_SPEED_FF_V_PER_RPM * 30.0f;

            s_enc_rpm = 5.0f;
            s_speed_iq_out = 0.01f;
            result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
            assert(fabsf(m.iq_target - 0.08f) < 1e-5f);
            assert(fabsf(result.vq - (0.08f * MOTOR_ENCODER_SPEED_VOLT_PER_AMP + ff_30)) < 1e-4f);

            s_enc_rpm = 15.0f;
            result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
            assert(fabsf(m.iq_target - 0.04f) < 1e-5f);
            assert(fabsf(result.vq - (0.04f * MOTOR_ENCODER_SPEED_VOLT_PER_AMP + ff_30)) < 1e-4f);

            s_enc_rpm = 20.0f;
            result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
            assert(fabsf(m.iq_target - 0.01f) < 1e-5f);
            assert(fabsf(result.vq - (0.01f * MOTOR_ENCODER_SPEED_VOLT_PER_AMP + ff_30)) < 1e-4f);
        }

        /* 有效有感反馈下，零速命令保持速度闭环，不能退回启动 Iq 或 SAFE。 */
        m.ol_target_rpm = 0.0f;
        m.ol_current_rpm = 0.0f;
        s_enc_rpm = 0.0f;
        s_last_speed_loop_active = false;
        s_speed_iq_out = 0.0f;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(result.action == MOTOR_CONTROL_ACTION_OUTPUT_DQ);
        assert(s_current_step_calls == 1);

        /* C2208 编码器速度环使用电压模式；零速命令时 Vq 为 0。 */
        s_ff_calls = 0;
        m.ol_pole_pairs = 7.0f;
        m.sensorless_flux_linkage = 0.01f;
        m.sensorless_motor_l = 0.0f;
        s_enc_zero = true;
        s_enc_speed_ok = true;
        s_enc_rpm = 150.0f;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(s_ff_calls == 0);
        assert(fabsf(result.vq) < 1e-4f);

        /* 电压前馈跟随目标方向，反转 30 RPM 时为负的基础前馈。 */
        m.ol_target_rpm = -30.0f;
        m.ol_current_rpm = -30.0f;
        s_enc_rpm = -30.0f;
        s_speed_iq_out = 0.0f;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(fabsf(result.vq + (MOTOR_ENCODER_SPEED_FF_BOOST_V + MOTOR_ENCODER_SPEED_FF_V_PER_RPM * 30.0f)) < 1e-4f);
        assert(s_current_step_calls == 1);

        /* 无感来源不加前馈。 */
        Motor_Feedback_SetSourceDefault(MOTOR_FEEDBACK_SRC_SENSORLESS);
        s_ff_calls = 0;
        result = Motor_Control_Step(&ctrl, &m, 24.0f, 0.001f, false);
        assert(s_ff_calls == 0);
        assert(s_current_step_calls == 2);
        assert(fabsf(result.vq - 0.2f) < 1e-4f);

        Motor_Feedback_SetSourceDefault(MOTOR_FEEDBACK_SRC_SENSORLESS);
    }

    {
        MOTOR_PARAM_S m = {0};
        motor_control_t ctrl;
        Motor_Control_Init(&ctrl);
        ctrl.startup.phase = MOTOR_STARTUP_DRAG_RAMP;
        ctrl.startup.soft_current_scale = ctrl.startup.drag_current_scale = 1.0f;
        m.ol_enable = true; m.ol_mode = MOTOR_OL_MODE_IF;
        m.ol_target_rpm = m.ol_current_rpm = 600.0f; m.ol_max_rpm = 800.0f;
        m.ol_pole_pairs = 7.0f; m.ol_vq_limit = 4.0f;
        m.sensorless_enable = true; m.ol_iq_set = 0.30f;
        Motor_Feedback_SetSourceDefault(MOTOR_FEEDBACK_SRC_SENSORLESS);
        s_sensorless_correction = 0.05f; s_takeover_complete = false;
        Motor_Control_Step(&ctrl, &m, 16.0f, 0.001f, false);
        assert(fabsf(s_last_frame_correction - 0.05f) < 1e-5f);
        s_complete_during_update = true;
        s_handoff_marks = 0U;
        Motor_Control_Step(&ctrl, &m, 16.0f, 0.001f, false);
        assert(s_takeover_complete);
        assert(s_handoff_marks == 1U);
        assert(fabsf(s_last_frame_correction - 0.05f) < 1e-5f);
        s_complete_during_update = false;
        /* 完成接管后，即使观测角继续旋转，也不能再迁移积分。 */
        s_takeover_complete = true;
        Motor_Control_Step(&ctrl, &m, 16.0f, 0.001f, false);
        assert(s_last_frame_correction == 0.0f);
        s_takeover_complete = false;
        Motor_Feedback_SetSourceDefault(MOTOR_FEEDBACK_SRC_OPENLOOP);
        Motor_Control_Step(&ctrl, &m, 16.0f, 0.001f, false);
        assert(s_last_frame_correction == 0.0f);
        Motor_Control_ResetRuntime(&ctrl, &m);
        assert(ctrl.current_frame_correction == 0.0f);
        s_sensorless_correction = 0.0f;
    }
    {
        motor_control_t settled;
        memset(&settled, 0, sizeof(settled));
        assert(!Motor_Control_IsIfDragSettled(NULL));
        assert(!Motor_Control_IsIfDragSettled(&settled));
        settled.startup.phase = MOTOR_STARTUP_DRAG_RAMP;
        settled.startup.drag_current_scale = 0.5f;
        assert(!Motor_Control_IsIfDragSettled(&settled));
        settled.startup.drag_current_scale = 1.0f;
        assert(settled.startup.soft_current_scale == 0.0f);
        assert(Motor_Control_IsIfDragSettled(&settled));
        settled.startup.alignment.running = true;
        assert(!Motor_Control_IsIfDragSettled(&settled));
        settled.startup.alignment.running = false;
        settled.startup.phase = MOTOR_STARTUP_ANGLE_HOLD;
        assert(!Motor_Control_IsIfDragSettled(&settled));
    }
    {
        const float targets[] = {0.0f, 300.0f, -300.0f};
        const float currents[] = {0.0f, 100.0f, -100.0f};
        motor_control_t ctrl;
        MOTOR_PARAM_S m;
        unsigned target_index, current_index, first;
        Motor_Feedback_SetSourceDefault(MOTOR_FEEDBACK_SRC_OPENLOOP);
        for (target_index = 0U; target_index < 3U; target_index++) {
            for (current_index = 0U; current_index < 3U; current_index++) {
                for (first = 0U; first < 2U; first++) {
                    motor_if_start_t expected;
                    motor_if_start_config_t config = {0.0f, 0.0f, 1.0f};
                    motor_if_start_input_t input;
                    Motor_Control_Init(&ctrl);
                    memset(&m, 0, sizeof(m));
                    ctrl.startup.phase = MOTOR_STARTUP_DRAG_RAMP;
                    ctrl.startup.if_start.first_cycle = first != 0U;
                    ctrl.startup.if_start.phase_boost = 0.5f;
                    ctrl.startup.if_start.iq_scale = 0.2f;
                    expected = ctrl.startup.if_start;
                    m.ol_enable = true;
                    m.ol_mode = MOTOR_OL_MODE_IF;
                    m.ol_target_rpm = targets[target_index];
                    m.ol_current_rpm = currents[current_index];
                    m.ol_max_rpm = 1000.0f;
                    m.ol_pole_pairs = 7.0f;
                    m.speed_enable = true;
                    Motor_Control_Step(&ctrl, &m, 16.0f, 1.0f / 15000.0f, false);
                    input.target_rpm = m.ol_target_rpm;
                    input.current_rpm = m.ol_current_rpm;
                    input.rotor_position_known = true;
                    MotorIfStart_Step(&expected, &config, &input);
                    assert(ctrl.startup.if_start.first_cycle == expected.first_cycle);
                    assert(ctrl.startup.if_start.phase_boost == expected.phase_boost);
                    assert(ctrl.startup.if_start.iq_scale == expected.iq_scale);
                    assert(ctrl.startup.if_start.angle_rpm == expected.angle_rpm);
                }
            }
        }
    }
    {
        motor_control_t diagnostic_control;
        MOTOR_PARAM_S diagnostic_motor = {0};
        motor_current_target_ctx_t diagnostic_ctx = {0};
        motor_control_result_t result;
        unsigned mode;
        int previous_updates = s_sensorless_updates;
        Motor_Control_Init(&diagnostic_control);
        diagnostic_control.startup.phase = MOTOR_STARTUP_DRAG_RAMP;
        diagnostic_control.startup.drag_current_scale = 1.0f;
        diagnostic_motor.ol_theta = 0.6f;
        diagnostic_motor.ol_vq_limit = 4.0f;
        diagnostic_motor.ol_iq_set = 1.0f;
        Motor_Feedback_SetSourceDefault(MOTOR_FEEDBACK_SRC_OPENLOOP);
        result = Motor_Control_IfDiagnosticStep(&diagnostic_control, &diagnostic_motor, &diagnostic_ctx, 0U);
        assert(result.action == MOTOR_CONTROL_ACTION_SAFE);
        for (mode = 1U; mode <= 2U; ++mode) {
            result = Motor_Control_IfDiagnosticStep(&diagnostic_control, &diagnostic_motor, &diagnostic_ctx, mode);
            assert(fabsf(result.vd - 0.6f) < 1e-6f);
        }
        memset(&diagnostic_ctx, 0xa5, sizeof(diagnostic_ctx));
        result = Motor_Control_IfDiagnosticStep(&diagnostic_control, &diagnostic_motor, &diagnostic_ctx, 3U);
        assert(result.vd == 1.0f && result.vq == 1.0f && !diagnostic_ctx.speed_loop_active);
        assert(!diagnostic_ctx.alignment_running && diagnostic_ctx.alignment_current_scale == 0.0f);
        assert(diagnostic_ctx.speed_iq_target == 0.0f && diagnostic_ctx.id_fw == 0.0f);
        assert(diagnostic_ctx.frame_correction == 0.0f);
        diagnostic_motor.ol_align_enable = true;
        diagnostic_control.startup.drag_current_scale = 0.25f;
        memset(&diagnostic_ctx, 0xa5, sizeof(diagnostic_ctx));
        Motor_Control_IfDiagnosticStep(&diagnostic_control, &diagnostic_motor, &diagnostic_ctx, 3U);
        assert(diagnostic_ctx.alignment_current_scale == 0.75f && diagnostic_ctx.soft_current_scale == 0.25f);
        assert(diagnostic_ctx.id_fw == 0.0f && !diagnostic_ctx.alignment_running);
        diagnostic_control.startup.phase = MOTOR_STARTUP_CURRENT_RAMP;
        diagnostic_control.startup.soft_current_scale = 0.5f;
        memset(&diagnostic_ctx, 0xa5, sizeof(diagnostic_ctx));
        Motor_Control_IfDiagnosticStep(&diagnostic_control, &diagnostic_motor, &diagnostic_ctx, 3U);
        assert(diagnostic_ctx.alignment_running && diagnostic_ctx.alignment_current_scale == 0.5f);
        assert(diagnostic_ctx.soft_current_scale == 0.5f && diagnostic_ctx.id_fw == 0.0f);
        assert(diagnostic_ctx.speed_iq_target == 0.0f && diagnostic_ctx.frame_correction == 0.0f);
        diagnostic_motor.ol_align_enable = false;
        diagnostic_control.startup.phase = MOTOR_STARTUP_DRAG_RAMP;
        diagnostic_control.startup.drag_current_scale = 1.0f;
        Motor_Control_IfDiagnosticStep(&diagnostic_control, &diagnostic_motor, &diagnostic_ctx, 3U);
        diagnostic_control.fw.id = -0.5f;
        result = Motor_Control_IfDiagnosticStep(&diagnostic_control, &diagnostic_motor, &diagnostic_ctx, 4U);
        assert(result.vd == 0.0f && diagnostic_control.fw.id == 0.0f);
        result = Motor_Control_IfDiagnosticStep(&diagnostic_control, &diagnostic_motor, &diagnostic_ctx, 5U);
        assert(result.vd == 4.0f);
        for (mode = 6U; mode <= 8U; ++mode) {
            int previous_calls = s_current_step_calls;
            result = Motor_Control_IfDiagnosticStep(&diagnostic_control, &diagnostic_motor, &diagnostic_ctx, mode);
            assert(result.action == MOTOR_CONTROL_ACTION_OUTPUT_DQ);
            assert(result.vd == 0.1f && result.vq == 0.2f && result.theta == 0.6f);
            assert(s_current_step_calls == previous_calls + 1);
        }
        assert(s_sensorless_updates == previous_updates);
        {
            const float limits[] = {0.0f, -0.5f, 1.0f, 0.0f};
            unsigned limit_index;
            diagnostic_motor.fw_duty_start = 0.8f;
            diagnostic_motor.fw_ramp_a_s = 8.0f;
            diagnostic_control.fw.id = -0.5f;
            for (limit_index = 0U; limit_index < sizeof(limits) / sizeof(limits[0]); ++limit_index) {
                motor_fw_t expected = diagnostic_control.fw;
                float expected_id;
                diagnostic_motor.fw_current_max = limits[limit_index];
                diagnostic_control.fw.last_vd = expected.last_vd = 0.0f;
                diagnostic_control.fw.last_vq = expected.last_vq = 4.0f;
                expected_id = Motor_Fw_Step(&expected, limits[limit_index], 0.8f, 8.0f,
                                           0.0f, 4.0f, 4.0f, 1.0f / 15000.0f);
                result = Motor_Control_IfDiagnosticStep(&diagnostic_control, &diagnostic_motor,
                                                        &diagnostic_ctx, 6U);
                assert(diagnostic_control.fw.id == expected.id && s_last_id_fw == expected_id);
                assert(diagnostic_control.fw.last_vd == result.vd);
                assert(diagnostic_control.fw.last_vq == result.vq);
            }
            diagnostic_motor.fw_current_max = 0.0f;
        }
        {
            const motor_feedback_source_t sources[] = {MOTOR_FEEDBACK_SRC_OPENLOOP, MOTOR_FEEDBACK_SRC_SENSORLESS,
                                                       MOTOR_FEEDBACK_SRC_ENCODER, MOTOR_FEEDBACK_SRC_HALL};
            const float angles[] = {0.6f, -3.4f, 7.0f};
            unsigned source, enabled, angle;
            for (source = 0U; source < sizeof(sources) / sizeof(sources[0]); ++source) {
                Motor_Feedback_SetSourceDefault(sources[source]);
                for (enabled = 0U; enabled < 2U; ++enabled) {
                    diagnostic_motor.sensorless_enable = enabled != 0U;
                    for (angle = 0U; angle < sizeof(angles) / sizeof(angles[0]); ++angle) {
                        motor_feedback_sample_t expected;
                        float normalized;
                        diagnostic_motor.ol_theta = angles[angle];
                        diagnostic_control.startup.if_start.phase_boost = 0.1f;
                        normalized = diagnostic_motor.ol_theta + diagnostic_control.startup.if_start.phase_boost;
                        utils_norm_angle_rad(&normalized);
                        Motor_Feedback_SampleDefault(&diagnostic_motor, normalized, &expected);
                        result = Motor_Control_IfDiagnosticStep(&diagnostic_control, &diagnostic_motor,
                                                               &diagnostic_ctx, 1U);
                        assert(result.vd == expected.elec_angle_rad);
                    }
                }
            }
        }
    }
    test_normalize_cache();
    test_submitted_boundaries();
    printf("test_motor_control: pass\n");
    return 0;
}
