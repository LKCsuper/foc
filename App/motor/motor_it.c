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
/* Includes ------------------------------------------------------------------*/
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
#include "motor_math.h"
#include "motor_sensorless.h"
#include "motor_detect.h"
#include "motor_hall.h"
#include "hal_motor.h"
#include "motor.h"
#include "motor_encoder.h"
#include "motor_hall_angle.h"
#include "motor_hall.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
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
/* Private function prototypes -----------------------------------------------*/
static void Motor_DmaInt_RecordRuntime(uint32_t start_time);
static float Motor_OpenLoop_GetAdcDt(MOTOR_PARAM_S *m);
static float Motor_OpenLoop_GetDt(MOTOR_PARAM_S *m);
static void Motor_OpenLoop_OutputSafe(MOTOR_PARAM_S *m);
static void Motor_Foc_OutputVoltage(MOTOR_PARAM_S *m, float vd, float vq, float theta, float v_bus);
static void Motor_It_ApplyControlResult(MOTOR_PARAM_S *m, float v_bus, const motor_control_result_t *result);
static bool Motor_DmaInt_ShouldRun(void);
static void Motor_DmaInt_Foc(void);
static void Motor_DmaInt_WhileDetect(void);
static void Motor_DmaInt_RunFastVf(MOTOR_PARAM_S *m);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 更新本拍 FOC 中断耗时统计（平均与最大值）
 * @detail: 超过 1000 us 的样本丢弃，避免计时回绕或异常拉高最大耗时
 * @param {uint32_t} start_time  本拍开始时的时间戳 (us)
 * @return {*}
 * @author: lkc
 */
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

/**
 * @description: 计算 ADC/保护用采样周期 dt
 * @detail: 优先用 Hal_Motor_AdcSampleRateHz()；无效时回退 motor->ol_foc_freq，并写回该字段
 * @param {MOTOR_PARAM_S} *m  电机对象（更新 ol_foc_freq）
 * @return {float} 采样周期 (s)，即 1/ADC 触发频率
 * @author: lkc
 */
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

/**
 * @description: 计算 FOC 控制环周期 dt（已乘分频）
 * @detail: dt_foc = dt_adc × FOC_CONTROL_LOOP_FREQ_DIVIDER，供斜坡/PI/观测器使用
 * @param {MOTOR_PARAM_S} *m  电机对象
 * @return {float} 控制周期 (s)
 * @author: lkc
 */
static float Motor_OpenLoop_GetDt(MOTOR_PARAM_S *m)
{
    return Motor_OpenLoop_GetAdcDt(m) * (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;
}

/**
 * @description: 输出三相中点占空比并提交（控制层 SAFE 路径）
 * @detail: 不等于关桥或保证无转矩；同时清零无感本拍应用电压记录
 * @param {MOTOR_PARAM_S} *m  电机对象（写 duty1/2/3）
 * @return {*}
 * @author: lkc
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

/**
 * @description: 反 Park + SVPWM，将 vd/vq 落到三相 PWM
 * @param {MOTOR_PARAM_S} *m  电机对象（写 duty）
 * @param {float} vd          D 轴电压 (V)
 * @param {float} vq          Q 轴电压 (V)
 * @param {float} theta       电角度 (rad)
 * @param {float} v_bus       母线电压 (V)
 * @return {*}
 * @author: lkc
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
    /* HD75 实机验证的相序为 A-C-B。 */
    Hal_Motor_PwmSetDuty(m->duty1, m->duty3, m->duty2);
    Hal_Motor_PwmCommit();
}

/**
 * @description: 将控制编排结果应用到 PWM
 * @detail: 调用前须已用 Motor_Control_ArbitrateOutput；故障锁存时不得传入 OUTPUT_DQ
 * @param {MOTOR_PARAM_S} *m                        电机对象
 * @param {float} v_bus                             母线电压 (V)
 * @param {const motor_control_result_t} *result    仲裁后的输出请求
 * @return {*}
 * @author: lkc
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

/**
 * @description: 查询当前是否正处于 I/F 启动前的转子预对齐阶段
 * @return {bool} true=预对齐进行中
 * @author: lkc
 */
bool Motor_OpenLoop_IsAligning(void)
{
    return Motor_Control_IsAligning(&s_control);
}

/**
 * @description: 返回本次转子预对齐已进行的时间
 * @return {float} 已对齐时长 (s)
 * @author: lkc
 */
float Motor_OpenLoop_GetAlignElapsed(void)
{
    return Motor_Control_GetAlignElapsed(&s_control);
}

/**
 * @description: 查询 HFI 凸极扫描启动流程是否正在运行
 * @return {bool} true=HFI 扫描进行中
 * @author: lkc
 */
bool Motor_HfiStart_IsRunning(void)
{
    return Motor_Control_HfiIsRunning(&s_control);
}

/**
 * @description: 查询 HFI 凸极扫描是否已成功识别出转子初始电角度
 * @return {bool} true=识别成功
 * @author: lkc
 */
bool Motor_HfiStart_Succeeded(void)
{
    return Motor_Control_HfiSucceeded(&s_control);
}

/**
 * @description: 返回 HFI 凸极扫描识别出的转子电角度
 * @return {float} 转子电角度 (rad)
 * @author: lkc
 */
float Motor_HfiStart_GetAngle(void)
{
    return Motor_Control_HfiGetAngle(&s_control);
}

/**
 * @description: 返回 HFI 凸极扫描识别的可信度
 * @return {float} 可信度 [0, 1]
 * @author: lkc
 */
float Motor_HfiStart_GetConfidence(void)
{
    return Motor_Control_HfiGetConfidence(&s_control);
}

/**
 * @description: 返回 ADC-DMA 中断的平均执行时间
 * @return {float} 平均耗时 (us)
 * @author: lkc
 */
float Motor_DmaInt_GetAverageRuntimeUs(void)
{
    return s_foc_runtime_average_us;
}

/**
 * @description: 返回 ADC-DMA 中断的历史最长执行时间
 * @return {float} 最大耗时 (us)
 * @author: lkc
 */
float Motor_DmaInt_GetMaximumRuntimeUs(void)
{
    return s_foc_runtime_maximum_us;
}

/**
 * @description: 返回实际 ADC-DMA 触发频率
 * @return {float} ADC 采样频率 (Hz)
 * @author: lkc
 */
float Motor_DmaInt_GetAdcFrequencyHz(void)
{
    float adc_trigger_hz = Hal_Motor_AdcSampleRateHz();

    if (adc_trigger_hz <= 0.0f) {
        adc_trigger_hz = motor.ol_foc_freq;
    }

    return adc_trigger_hz;
}

/**
 * @description: 返回 FOC 控制环实际运行频率
 * @return {float} 控制环频率 (Hz)
 * @author: lkc
 */
float Motor_DmaInt_GetControlFrequencyHz(void)
{
    return Motor_DmaInt_GetAdcFrequencyHz() / (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;
}

/**
 * @description: 判断本拍 ADC 回调是否应跑完整 FOC（电流环/观测器/SVPWM）
 * @return {bool} true=本拍执行完整控制
 * @author: lkc
 */
static bool Motor_DmaInt_ShouldRun(void)
{
    return Motor_FocDivider_ShouldRun(&s_foc_divider);
}

/**
 * @description: 复现 3e5d564 已上板成功的最小 V/F 电压开环路径。
 * @detail: 仅供 V/F 基线验证；固定 Vq=5V、每个 15kHz 控制拍推进 0.001rad，
 *          绕过新版启动编排，以判断回归是否位于 Motor_Control_Step。
 * @param {MOTOR_PARAM_S} *m 电机对象
 * @return {*}
 */
static void Motor_DmaInt_RunFastVf(MOTOR_PARAM_S *m)
{
    const float theta_step = 0.001f;
    float vq = m->ol_v_boost;

    if (vq < 0.0f) {
        vq = 0.0f;
    }
    if (vq > m->ol_vq_limit) {
        vq = m->ol_vq_limit;
    }

    m->ol_current_rpm = theta_step * Motor_DmaInt_GetControlFrequencyHz() /
                        (RPM2RADPS_f(1.0f) * m->ol_pole_pairs);
    m->theta += theta_step;
    utils_norm_angle_rad(&m->theta);
    m->ol_theta = m->theta;
    Motor_Foc_OutputVoltage(m, 0.0f, vq, m->theta, m->v_bus);
}

/**
 * @description: 采样换算入口：捕获 ADC 帧并换算为电流/电压物理量
 * @detail: 内部为 SampleCapture → ApplyFrame；每拍 ADC 回调均可调用
 * @return {*}
 * @author: lkc
 */
void Motor_It_ParamCal(void)
{
    Motor_Sense_Update(&motor);
}

/**
 * @description: 正常运行路径：采样、保护、分频控制与 PWM 应用
 * @detail: 故障锁存后强制 SAFE；分频未到时只统计耗时后返回
 * @return {*}
 * @author: lkc
 */
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
 * @description: 偏移校准占用桥时：只采样换算，不跑 FOC、不跑软件过流
 * @detail: 零偏标定前 ia/ib/ic 不可信，软件 ABS_OVER_CURRENT 易误锁存；硬件过流/Break 仍生效。
 *          不用中点 SAFE，以免与逐相 EnablePhase 打架。
 * @return {*}
 * @author: lkc
 */
static void Motor_DmaInt_WhileDetect(void)
{
    Motor_It_ParamCal();
}

/**
 * @description: ADC-DMA 传输完成中断回调，FOC 控制链驱动入口
 * @param {void} *p        回调用户参数（未使用）
 * @param {uint32_t} flags DMA 事件标志（未使用）
 * @return {*}
 * @author: lkc
 */
void Motor_DmaInt(void *p, uint32_t flags)
{
    uint32_t runtime_start;

    (void)p;
    (void)flags;

    if (Motor_Detect_IsActive()) {
        Motor_DmaInt_WhileDetect();
        return;
    }

    /*
     * V/F 实机回归基线：严格保持 3e5d564 的执行顺序——先分频，命中控制拍后
     * 才采样并直接更新 PWM。这样不会让新增的编码器/霍尔/软件保护计算占满
     * 每个 30kHz ADC 中断，便于确认问题是否源自实时预算被破坏。
     * 该路径仅用于短时低压验证；硬件 Break 仍须有效。
     */
    if (motor.ol_enable && motor.ol_mode == MOTOR_OL_MODE_VF) {
        runtime_start = Hal_Motor_TimeUs();
        if (!Motor_DmaInt_ShouldRun()) {
            Motor_DmaInt_RecordRuntime(runtime_start);
            return;
        }
        Motor_It_ParamCal();
        if (motor.v_bus >= 1.0f) {
            Motor_DmaInt_RunFastVf(&motor);
        }
        Motor_DmaInt_RecordRuntime(runtime_start);
        return;
    }

    Motor_DmaInt_Foc();
}

#ifdef __cplusplus
}
#endif
