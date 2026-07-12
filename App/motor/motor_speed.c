#ifdef __cplusplus
extern "C" {
#endif

#include "includes.h"

/* 速度反馈与 PI 的运行时状态，不需要写入 Flash。 */
static struct {
    float measured_rpm; /* 反馈源给出的真实机械转速 */
    bool feedback_valid;
    float loop_time;    /* 速度环分频累计时间，单位 s */
    float integral;     /* PI 积分输出，单位 A */
    float error;        /* 速度误差，单位 RPM */
    float iq_target;    /* 给电流内环的 Iq 目标，单位 A */
} s_speed;

static float Motor_Speed_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

void Motor_Speed_Init(void)
{
    memset(&s_speed, 0, sizeof(s_speed));
    Motor_Protect_SetSpeedFeedback(0.0f, false);
}

void Motor_SpeedFeedback_Set(float rpm, bool valid)
{
    s_speed.measured_rpm = valid ? rpm : 0.0f;
    s_speed.feedback_valid = valid;
    Motor_Protect_SetSpeedFeedback(s_speed.measured_rpm, valid);

    if (!valid) {
        /* 反馈丢失后立即撤销速度 PI 输出，由上层决定是否使用启动 Iq。 */
        Motor_SpeedLoop_Reset();
    }
}

void Motor_SpeedLoop_Reset(void)
{
    s_speed.loop_time = 0.0f;
    s_speed.integral = 0.0f;
    s_speed.error = 0.0f;
    s_speed.iq_target = 0.0f;
}

void Motor_SpeedLoop_PrimeIq(float iq)
{
    s_speed.integral = iq;
    s_speed.iq_target = iq;
    s_speed.loop_time = 0.0f;
}

void Motor_SpeedLoop_Run(MOTOR_PARAM_S *motor, float dt)
{
    float loop_period;
    float control_dt;
    float iq_limit;
    float proportional;
    float integral_candidate;
    float unsaturated_output;
    float output;

    if (!motor->speed_enable ||
        motor->ol_mode != MOTOR_OL_MODE_IF ||
        !motor->ol_enable ||
        !s_speed.feedback_valid) {
        Motor_SpeedLoop_Reset();
        return;
    }

    if (motor->speed_loop_hz < 1.0f) {
        motor->speed_loop_hz = 1.0f;
    }
    loop_period = 1.0f / motor->speed_loop_hz;
    s_speed.loop_time += dt;
    if (s_speed.loop_time < loop_period) {
        return;
    }

    control_dt = s_speed.loop_time;
    s_speed.loop_time = 0.0f;
    iq_limit = fabsf(motor->speed_iq_limit);

    /* 使用经过加减速斜坡的目标，避免启动瞬间出现很大的误差阶跃。 */
    s_speed.error = motor->ol_current_rpm - s_speed.measured_rpm;
    proportional = motor->speed_kp * s_speed.error;
    integral_candidate = s_speed.integral +
                         motor->speed_ki * s_speed.error * control_dt;
    unsaturated_output = proportional + integral_candidate;
    output = Motor_Speed_Clamp(unsaturated_output, -iq_limit, iq_limit);

    /* 条件积分抗饱和：输出顶住且误差继续推向饱和方向时暂停积分。 */
    if (!((unsaturated_output > iq_limit && s_speed.error > 0.0f) ||
          (unsaturated_output < -iq_limit && s_speed.error < 0.0f))) {
        s_speed.integral = Motor_Speed_Clamp(integral_candidate, -iq_limit, iq_limit);
    } else {
        output = Motor_Speed_Clamp(proportional + s_speed.integral,
                                   -iq_limit, iq_limit);
    }

    /* 禁止制动时，超速只撤销转矩，不输出反向 Iq。 */
    if (!motor->speed_allow_braking) {
        if (motor->ol_current_rpm > 0.0f && output < 0.0f) {
            output = 0.0f;
        } else if (motor->ol_current_rpm < 0.0f && output > 0.0f) {
            output = 0.0f;
        }
    }

    s_speed.iq_target = output;
}

float Motor_SpeedLoop_GetIqTarget(void)
{
    return s_speed.iq_target;
}

float Motor_SpeedLoop_GetMeasuredRpm(void)
{
    return s_speed.measured_rpm;
}

float Motor_SpeedLoop_GetError(void)
{
    return s_speed.error;
}

bool Motor_SpeedLoop_IsFeedbackValid(void)
{
    return s_speed.feedback_valid;
}

#ifdef __cplusplus
}
#endif
