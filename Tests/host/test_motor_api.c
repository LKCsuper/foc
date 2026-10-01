#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "motor_param.h"
#include "motor_settings.h"
#include "motor_types.h"
#include "motor_feedback.h"
#include "motor_speed.h"

static int speed_reset_calls;
static int save_calls;
static bool bldc_selected;

bool Motor_Bldc_IsSelected(void)
{
    return bldc_selected;
}

bool FlashStorage_Load(MOTOR_PARAM_S *m)
{
    (void)m;
    return false;
}

bool FlashStorage_Save(const MOTOR_PARAM_S *m)
{
    (void)m;
    ++save_calls;
    return true;
}

void Motor_Speed_Init(void)
{
}

void Motor_Sensorless_Init(void)
{
}

void Motor_SpeedLoop_Reset(void)
{
    ++speed_reset_calls;
}

float Motor_SpeedLoop_GetMeasuredRpm(void)
{
    return 123.0f;
}

float Motor_SpeedLoop_GetIqTarget(void)
{
    return 0.45f;
}

float Motor_SpeedLoop_GetError(void)
{
    return -7.0f;
}

bool Motor_SpeedLoop_GetSnapshot(motor_speed_snapshot_t *snapshot)
{
    snapshot->target_rpm = 100.0f;
    snapshot->measured_rpm = 123.0f;
    snapshot->error = -23.0f;
    snapshot->iq_target = 0.45f;
    return true;
}

bool Motor_OpenLoop_IsAligning(void)
{
    return false;
}

float Motor_OpenLoop_GetAlignElapsed(void)
{
    return 0.0f;
}

mc_fault_code Motor_Protect_GetFault(void)
{
    return FAULT_CODE_NONE;
}

bool Motor_Protect_IsFaultLatched(void)
{
    return false;
}

static int pwm_start_calls;
static int pwm_safe_calls;
static int pwm_stop_calls;

void Hal_Motor_PwmStart(void)
{
    ++pwm_start_calls;
}

void Hal_Motor_PwmSetSafe(void)
{
    ++pwm_safe_calls;
}

void Hal_Motor_PwmStop(void)
{
    ++pwm_stop_calls;
}

void Motor_Feedback_SetSourceDefault(motor_feedback_source_t source)
{
    (void)source;
}

motor_feedback_source_t Motor_Feedback_GetSourceDefault(void)
{
    return MOTOR_FEEDBACK_SRC_SENSORLESS;
}

void Motor_Feedback_SampleDefault(const MOTOR_PARAM_S *m, float open_loop_theta, motor_feedback_sample_t *out)
{
    (void)m;
    memset(out, 0, sizeof(*out));
    out->elec_angle_rad = open_loop_theta;
    out->sensor_state = -1;
    out->used_source = MOTOR_FEEDBACK_SRC_SENSORLESS;
}

void Motor_Hall_AngleResetDefault(void)
{
}

int main(void)
{
    motor_command_t cmd;
    motor_command_t got;
    motor_status_t status;
    motor_settings_t settings;
    motor_config_t cfg;

    MOTOR_Param_Init();
    assert(Motor_Settings_Capture(&motor, &settings));

    assert(Motor_GetCommand(NULL, &cmd) == MOTOR_ERR_NULL);
    assert(Motor_SetCommand(&motor, NULL) == MOTOR_ERR_NULL);
    assert(Motor_GetStatus(&motor, NULL) == MOTOR_ERR_NULL);
    assert(Motor_SetConfig(&motor, NULL) == MOTOR_ERR_NULL);

    assert(Motor_GetCommand(&motor, &cmd) == MOTOR_OK);
    cmd.enable = true;
    cmd.target_rpm = 500.0f;
    cmd.iq_set = 0.5f;
    cmd.id_set = 0.0f;
    cmd.mode = motor.ol_mode; /* 保持默认模式，避免仅因切模式触发 SpeedLoop_Reset */
    cmd.speed_enable = false;
    speed_reset_calls = 0;
    pwm_start_calls = 0;
    pwm_safe_calls = 0;
    pwm_stop_calls = 0;
    bldc_selected = true;
    assert(Motor_SetCommand(&motor, &cmd) == MOTOR_ERR_BUSY);
    assert(!motor.ol_enable && pwm_start_calls == 0);
    bldc_selected = false;
    assert(Motor_SetCommand(&motor, &cmd) == MOTOR_OK);
    assert(motor.ol_enable && motor.ol_target_rpm == 500.0f && motor.ol_iq_set == 0.5f);
    assert(speed_reset_calls == 0);
    assert(pwm_start_calls == 1);
    assert(pwm_safe_calls == 0);

    /* 在线可改 V_boost；极对数运行中禁止。 */
    cfg = settings.config;
    cfg.ol_v_boost = 1.25f;
    assert(Motor_SetConfig(&motor, &cfg) == MOTOR_OK);
    assert(motor.ol_v_boost == 1.25f);
    cfg.ol_pole_pairs = 4.0f;
    assert(Motor_SetConfig(&motor, &cfg) == MOTOR_ERR_BUSY);
    assert(motor.ol_pole_pairs == settings.config.ol_pole_pairs);

    /* NaN / 非法模式拒绝；负 Iq 作为反向转矩命令接受。 */
    cmd.target_rpm = NAN;
    assert(Motor_SetCommand(&motor, &cmd) == MOTOR_ERR_INVALID);
    assert(motor.ol_target_rpm == 500.0f);
    cmd.target_rpm = 500.0f;
    cmd.mode = 99;
    assert(Motor_SetCommand(&motor, &cmd) == MOTOR_ERR_INVALID);
    cmd.mode = MOTOR_OL_MODE_IF;
    cmd.iq_set = -1.0f;
    assert(Motor_SetCommand(&motor, &cmd) == MOTOR_OK);
    assert(motor.ol_iq_set == -1.0f);

    /* 超限目标被限幅。 */
    cmd.iq_set = 0.5f;
    cmd.target_rpm = 9000.0f;
    assert(Motor_SetCommand(&motor, &cmd) == MOTOR_OK);
    assert(motor.ol_target_rpm == motor.ol_max_rpm);

    /* 降低 max_rpm 时同步夹紧命令目标。 */
    cfg = settings.config;
    cfg.ol_v_boost = motor.ol_v_boost;
    cfg.ol_max_rpm = 200.0f;
    if (cfg.sensorless_openloop_max_rpm > cfg.ol_max_rpm) {
        cfg.sensorless_openloop_max_rpm = cfg.ol_max_rpm;
    }
    assert(Motor_SetConfig(&motor, &cfg) == MOTOR_OK);
    assert(motor.ol_max_rpm == 200.0f);
    assert(motor.ol_target_rpm == 200.0f);

    /* 停机整体提交：清积分与斜坡，并复位速度环。 */
    motor.ol_vd_int = 9.0f;
    motor.ol_current_rpm = 200.0f;
    speed_reset_calls = 0;
    pwm_safe_calls = 0;
    pwm_stop_calls = 0;
    assert(Motor_RequestStop(&motor) == MOTOR_OK);
    assert(!motor.ol_enable && motor.ol_target_rpm == 200.0f);
    assert(motor.ol_vd_int == 0.0f && motor.ol_current_rpm == 0.0f);
    assert(motor.ol_iq_set == 0.5f);
    assert(speed_reset_calls == 1);
    assert(pwm_safe_calls == 0 && pwm_stop_calls == 1);

    assert(Motor_GetCommand(&motor, &got) == MOTOR_OK);
    assert(!got.enable && got.target_rpm == 200.0f && got.iq_set == 0.5f);

    motor.currentAdc[0] = 2037.0f;
    motor.currentAdc[1] = 2038.0f;
    motor.currentAdc[2] = 2048.0f;
    motor.currentOffset[0] = 2044.4f;
    motor.currentOffset[1] = 2043.5f;
    motor.currentOffset[2] = 2043.5f;
    assert(Motor_GetStatus(&motor, &status) == MOTOR_OK);
    assert(status.current_adc[0] == 2037.0f && status.current_adc[1] == 2038.0f &&
           status.current_adc[2] == 2048.0f);
    assert(status.current_offset[0] == 2044.4f && status.current_offset[1] == 2043.5f &&
           status.current_offset[2] == 2043.5f);
    assert(!status.enable && status.v_bus == motor.v_bus);
    assert(status.measured_rpm == 123.0f && status.speed_iq == 0.45f);
    assert(status.fault == FAULT_CODE_NONE && !status.fault_latched);

    /* ISR 发布后，任务读取的是完整已发布帧，不是随后变化的现场对象。 */
    Motor_StatusSnapshotPublishFromIsr(&motor);
    motor.currentAdc[0] = 1000.0f;
    motor.v_bus += 1.0f;
    assert(Motor_GetStatus(&motor, &status) == MOTOR_OK);
    assert(status.current_adc[0] == 2037.0f);
    assert(status.v_bus != motor.v_bus);

    /* runtime 适配：清控制态不改配置/命令目标电流。 */
    {
        motor_runtime_t rt;
        motor.ia = 3.3f;
        motor.ol_current_kp = 0.2f;
        motor.ol_iq_set = 0.5f;
        motor.ol_vd_int = 7.0f;
        motor.ol_current_rpm = 88.0f;
        assert(Motor_GetRuntime(&motor, &rt) == MOTOR_OK);
        assert(rt.ia == 3.3f && rt.ol_vd_int == 7.0f && rt.ol_current_rpm == 88.0f);
        assert(Motor_ClearRuntimeControl(&motor) == MOTOR_OK);
        assert(motor.ol_vd_int == 0.0f && motor.ol_current_rpm == 0.0f);
        assert(motor.ia == 3.3f && motor.ol_current_kp == 0.2f && motor.ol_iq_set == 0.5f);
    }

    /* 停机后可改极对数。 */
    cfg = settings.config;
    cfg.ol_pole_pairs = 4.0f;
    assert(Motor_SetConfig(&motor, &cfg) == MOTOR_OK);
    assert(motor.ol_pole_pairs == 4.0f);

    printf("test_motor_api: pass\n");
    return 0;
}
