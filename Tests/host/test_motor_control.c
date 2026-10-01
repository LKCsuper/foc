#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_control.h"
#include "motor_param.h"
#include "motor_current.h"
#include "motor_feedback.h"

static float s_last_current_theta;
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

void Motor_SpeedLoop_Reset(void)
{
    s_speed_iq_out = 0.0f;
}

void Motor_Sensorless_Reset(void)
{
}

bool Motor_Sensorless_IsTakeoverComplete(void)
{
    return false;
}

void Motor_SpeedLoop_PrimeIq(float iq)
{
    s_speed_prime_iq = iq;
}

void Motor_Sensorless_Update(MOTOR_PARAM_S *motor, float dt)
{
    (void)motor;
    (void)dt;
    s_sensorless_updates++;
}

void Motor_SpeedFeedback_Set(float rpm, bool valid)
{
    s_speed_fb_rpm = rpm;
    s_speed_fb_valid = valid;
    if (!valid) {
        s_speed_iq_out = 0.0f;
    }
}

void Motor_SpeedLoop_Run(MOTOR_PARAM_S *motor, float dt)
{
    (void)dt;
    if (motor != NULL && motor->speed_enable && s_speed_fb_valid) {
        s_speed_iq_out = 1.25f; /* 模拟速度 PI 输出 Iq */
    } else {
        s_speed_iq_out = 0.0f;
    }
}

void Motor_SpeedLoop_SetRunAllowed(bool allowed)
{
    if (!allowed) {
        s_speed_iq_out = 0.0f;
    }
}

bool Motor_SpeedLoop_IsFeedbackValid(void)
{
    return s_speed_fb_valid;
}

float Motor_SpeedLoop_GetMeasuredRpm(void)
{
    return s_speed_fb_rpm;
}

float Motor_SpeedLoop_GetIqTarget(void)
{
    return s_speed_iq_out;
}

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

float Motor_Sensorless_GetControlAngle(float open_loop_angle)
{
    return open_loop_angle + 9.0f; /* 无感路径可区分 */
}

static bool s_enc_zero;
static float s_enc_elec = 1.57f;
static float s_enc_rpm = 321.0f;
static bool s_enc_speed_ok = true;
static int s_enc_angle_reads;

bool Motor_Encoder_IsZeroValid(void)
{
    return s_enc_zero;
}

float Motor_Encoder_GetElecAngle(void)
{
    s_enc_angle_reads++;
    return s_enc_elec;
}

float Motor_Encoder_GetMechAngle(void)
{
    return s_enc_elec;
}

float Motor_Encoder_GetMechRpm(void)
{
    return s_enc_rpm;
}

float Motor_Encoder_GetRawMechRpm(void)
{
    return s_enc_rpm;
}

float Motor_Encoder_GetEdgeAge(void)
{
    return 0.0f;
}


float Motor_Encoder_GetSampleAge(void)
{
    return 0.0001f;
}

bool Motor_Encoder_IsSignalFault(void)
{
    return false;
}

bool Motor_Encoder_IsSpeedValid(void)
{
    return s_enc_speed_ok;
}

bool Motor_Hall_AngleIsAngleValid(void)
{
    return false;
}

float Motor_Hall_AngleGetTheta(void)
{
    return 0.0f;
}

float Motor_Hall_AngleGetMechRpm(void)
{
    return 0.0f;
}

bool Motor_Hall_AngleIsSpeedValid(void)
{
    return false;
}

float Motor_Hall_AngleGetRawMechRpm(void)
{
    return 0.0f;
}

float Motor_Hall_AngleGetFastMechRpm(void)
{
    return 0.0f;
}

float Motor_Hall_AngleGetEdgeAge(void)
{
    return 0.0f;
}

int Motor_Hall_AngleGetState(void)
{
    return 0;
}

void Motor_Hall_AngleResetDefault(void)
{
}

bool Motor_Hall_DetectGetTargets(MOTOR_PARAM_S *m, float *theta_rad, float *id_target, float *iq_target)
{
    (void)m;
    (void)theta_rad;
    (void)id_target;
    (void)iq_target;
    return false;
}

bool Motor_Encoder_DetectGetTargets(float *theta_rad, float *id_target, float *iq_target)
{
    (void)theta_rad;
    (void)id_target;
    (void)iq_target;
    return false;
}

void Motor_Current_FeedbackUpdateDq(MOTOR_PARAM_S *motor, float theta)
{
    (void)motor;
    (void)theta;
}

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

motor_dq_voltage_t Motor_Current_Step(MOTOR_PARAM_S *motor, float theta, const motor_current_target_ctx_t *ctx,
                                      float dt, float v_limit, float *id_target_out, float *iq_target_out)
{
    motor_dq_voltage_t v = {0.1f, 0.2f};
    (void)motor;
    (void)dt;
    (void)v_limit;
    s_current_step_calls++;
    s_last_current_theta = theta;
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
    Motor_Control_NormalizeConfig(&motor);
    assert(motor.ol_pole_pairs >= 1.0f);
    assert(motor.sensorless_phase_tolerance <= 1.50f);

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

    printf("test_motor_control: pass\n");
    return 0;
}
