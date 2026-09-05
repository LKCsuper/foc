/*
 * @Description: FOC 中断集成：采样、保护、分频、控制调用与 PWM/耗时
 * @Author: lkc
 *
 * 学习要点：
 *   1) 本文件是平台胶水，不承载对齐/斜坡/电流 PI 算法细节。
 *   2) 算法在 motor_control / motor_current / 启动与无感模块。
 *   3) 分频前：Hal 采样帧 → Sense 换算 + 软件保护；分频后：Motor_Control_Step → 应用 PWM。
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#include "motor_param.h"
#include "motor_it.h"
#include "motor_sense.h"
#include "motor_protect.h"
#include "motor_control.h"
#include "motor_foc_divider.h"
#include "motor_foc.h"
#include "motor_sensorless.h"
#include "motor_detect.h"
#include "motor_hall.h"
#include "hal_motor.h"
#include "motor.h"
#include "motor_encoder.h"
#include "motor_hall_angle.h"
#include "motor_hall.h"

static motor_control_t s_control = {
    .startup =
        {
            .alignment = {.stopped = true},
            .if_start = {.first_cycle = true, .iq_scale = 1.0f},
        },
};

static motor_foc_divider_t s_foc_divider = {
    .divider = FOC_CONTROL_LOOP_FREQ_DIVIDER,
};

/*
 * ADC-DMA 中断执行时间统计。
 * TIM5 在 BSP 中配置为 1MHz，所以计数差值直接等于微秒数。
 */
static volatile float s_foc_runtime_average_us = 0.0f;
static volatile float s_foc_runtime_maximum_us = 0.0f;

static void Motor_DmaInt_RecordRuntime(uint32_t start_time)
{
    uint32_t elapsed_us = Hal_Motor_TimeUs() - start_time;
    float elapsed = (float)elapsed_us;

    if (elapsed_us > 1000U) {
        return;
    }

    if (s_foc_runtime_average_us <= 0.0f) {
        s_foc_runtime_average_us = elapsed;
    } else {
        s_foc_runtime_average_us += 0.02f * (elapsed - s_foc_runtime_average_us);
    }

    if (elapsed > s_foc_runtime_maximum_us) {
        s_foc_runtime_maximum_us = elapsed;
    }
}

static float Motor_OpenLoop_GetAdcDt(MOTOR_PARAM_S *m)
{
    float foc_freq = Hal_Motor_AdcSampleRateHz();

    if (foc_freq > 1.0f) {
        m->ol_foc_freq = foc_freq;
    } else {
        foc_freq = m->ol_foc_freq;
        if (foc_freq <= 1.0f) {
            foc_freq = 1.0f;
            m->ol_foc_freq = foc_freq;
        }
    }

    return 1.0f / foc_freq;
}

static float Motor_OpenLoop_GetDt(MOTOR_PARAM_S *m)
{
    return Motor_OpenLoop_GetAdcDt(m) * (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;
}

/*
 * 输出三相中点占空比（沿用旧停机路径，不等于保证无转矩）。
 * 板级故障关断仲裁需另行验证。
 */
static void Motor_OpenLoop_OutputSafe(MOTOR_PARAM_S *m)
{
    uint32_t mid = Hal_Motor_PwmPeriodCounts() / 2U;

    m->duty1 = mid;
    m->duty2 = mid;
    m->duty3 = mid;
    Hal_Motor_PwmSetDuty(m->duty1, m->duty2, m->duty3);
    Hal_Motor_PwmCommit();
    Motor_Sensorless_SetAppliedVoltage(0.0f, 0.0f);
}

/*
 * 反 Park + SVPWM + HAL 写 PWM；并记录本拍 αβ 电压供无感下一拍使用。
 */
static void Motor_Foc_OutputVoltage(MOTOR_PARAM_S *m, float vd, float vq, float theta, float v_bus)
{
    float v_alpha;
    float v_beta;
    uint32_t sector;
    uint32_t period = Hal_Motor_PwmPeriodCounts();

    inverse_park(vd, vq, theta, &v_alpha, &v_beta);
    Motor_Sensorless_SetAppliedVoltage(v_alpha, v_beta);
    svm_hqd(v_alpha, v_beta, v_bus, period, &m->duty1, &m->duty2, &m->duty3, &sector);
    Hal_Motor_PwmSetDuty(m->duty1, m->duty2, m->duty3);
    Hal_Motor_PwmCommit();
}

/*
 * 将控制编排结果落到 PWM。
 * 调用前须已用 Motor_Control_ArbitrateOutput：故障锁存时不得传入 OUTPUT_DQ。
 */
static void Motor_It_ApplyControlResult(MOTOR_PARAM_S *m, float v_bus, const motor_control_result_t *result)
{
    if (result == NULL) {
        return;
    }

    if (result->action == MOTOR_CONTROL_ACTION_SAFE) {
        Motor_OpenLoop_OutputSafe(m);
    } else if (result->action == MOTOR_CONTROL_ACTION_OUTPUT_DQ) {
        Motor_Foc_OutputVoltage(m, result->vd, result->vq, result->theta, v_bus);
    }
}

bool Motor_OpenLoop_IsAligning(void)
{
    return Motor_Control_IsAligning(&s_control);
}

float Motor_OpenLoop_GetAlignElapsed(void)
{
    return Motor_Control_GetAlignElapsed(&s_control);
}

bool Motor_HfiStart_IsRunning(void)
{
    return Motor_Control_HfiIsRunning(&s_control);
}

bool Motor_HfiStart_Succeeded(void)
{
    return Motor_Control_HfiSucceeded(&s_control);
}

float Motor_HfiStart_GetAngle(void)
{
    return Motor_Control_HfiGetAngle(&s_control);
}

float Motor_HfiStart_GetConfidence(void)
{
    return Motor_Control_HfiGetConfidence(&s_control);
}

float Motor_DmaInt_GetAverageRuntimeUs(void)
{
    return s_foc_runtime_average_us;
}

float Motor_DmaInt_GetMaximumRuntimeUs(void)
{
    return s_foc_runtime_maximum_us;
}

float Motor_DmaInt_GetAdcFrequencyHz(void)
{
    float adc_trigger_hz = Hal_Motor_AdcSampleRateHz();

    if (adc_trigger_hz <= 0.0f) {
        adc_trigger_hz = motor.ol_foc_freq;
    }

    return adc_trigger_hz;
}

float Motor_DmaInt_GetControlFrequencyHz(void)
{
    return Motor_DmaInt_GetAdcFrequencyHz() / (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;
}

static bool Motor_DmaInt_ShouldRun(void)
{
    return Motor_FocDivider_ShouldRun(&s_foc_divider);
}

void Motor_It_ParamCal(void)
{
    /* Sense_Update 内部：SampleCapture → ApplyFrame（ADC→A/V） */
    Motor_Sense_Update(&motor);
}

static void Motor_DmaInt_Foc(void)
{
    uint32_t runtime_start = Hal_Motor_TimeUs();
    float protection_dt;
    float dt;
    motor_control_result_t result;

    Motor_It_ParamCal();
    protection_dt = Motor_OpenLoop_GetAdcDt(&motor);
    /* 编码器速度/新鲜度每拍更新；未选 ENCODER 源时也不伤，仅维护差分 */
    Motor_Encoder_Update(protection_dt);
    Motor_Hall_AngleUpdateDefault(motor.hall_table, Motor_Hall_Read(false, motor.hall_extra_samples),
                                  motor.ol_pole_pairs, protection_dt);

    if (Motor_Protect_Run(&motor, protection_dt)) {
        Motor_Control_ResetRuntime(&s_control, &motor);
        Motor_OpenLoop_OutputSafe(&motor);
        Motor_DmaInt_RecordRuntime(runtime_start);
        return;
    }

    if (!Motor_DmaInt_ShouldRun()) {
        Motor_DmaInt_RecordRuntime(runtime_start);
        return;
    }

    dt = Motor_OpenLoop_GetDt(&motor);
    result = Motor_Control_Step(&s_control, &motor, motor.v_bus, dt, Motor_Hall_DetectIsActive());
    /*
     * 无感可能在本拍 Step 内锁存失速。先仲裁：故障态只允许 SAFE，
     * 绝不先落 DQ 再改安全输出（避免故障被控制结果覆盖）。
     */
    if (Motor_Protect_IsFaultLatched()) {
        Motor_Control_ResetRuntime(&s_control, &motor);
        result = Motor_Control_ArbitrateOutput(true, &result);
    } else {
        result = Motor_Control_ArbitrateOutput(false, &result);
    }
    Motor_It_ApplyControlResult(&motor, motor.v_bus, &result);
    Motor_DmaInt_RecordRuntime(runtime_start);
}

/**
 * @description: 偏移校准占用桥时：仍采样并跑软件保护，不跑 FOC（避免抢写 PWM）。
 * @detail: 故障锁存后调用 PwmStop（关通道），不用中点 SAFE，以免与逐相 EnablePhase 打架。
 *          硬件过流/Break 仍独立于本路径。
 */
static void Motor_DmaInt_WhileDetect(void)
{
    float protection_dt;

    Motor_It_ParamCal();
    protection_dt = Motor_OpenLoop_GetAdcDt(&motor);
    if (Motor_Protect_Run(&motor, protection_dt)) {
        Hal_Motor_PwmStop();
    }
}

void Motor_DmaInt(void *p, uint32_t flags)
{
    (void)p;
    (void)flags;

    if (Motor_Detect_IsActive()) {
        Motor_DmaInt_WhileDetect();
        return;
    }

    Motor_DmaInt_Foc();
}

#ifdef __cplusplus
}
#endif
