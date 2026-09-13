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
#include <string.h>

#include "stm32f4xx.h"
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
#include "motor_bldc_sensorless.h"
#include "motor_bldc_six_step.h"
#include "motor_bldc_six_step_hal.h"
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
static motor_bldc_six_step_t s_bldc_commutator;
static motor_bldc_sensorless_t s_bldc_sensorless;
static volatile bool s_bldc_selected = false;
static bool s_bldc_initialized = false;
/*
 * ADC-DMA 中断执行时间统计。
 * TIM5 在 BSP 中配置为 1MHz，所以计数差值直接等于微秒数。
 */
static volatile float s_foc_runtime_average_us = 0.0f;
static volatile float s_foc_runtime_maximum_us = 0.0f;
/* Private function prototypes -----------------------------------------------*/
static void Motor_DmaInt_RecordRuntime(uint32_t start_time);
static bool Motor_Bldc_InitDefault(void);
static bool Motor_Bldc_StartInternal(motor_bldc_direction_t direction, float duty,
                                     bool fixed_open_loop, float sector_period_s);
static uint32_t Motor_Bldc_StatusLock(void);
static void Motor_Bldc_StatusUnlock(uint32_t primask);
static float Motor_OpenLoop_GetAdcDt(MOTOR_PARAM_S *m);
static float Motor_OpenLoop_GetDt(MOTOR_PARAM_S *m);
static void Motor_OpenLoop_OutputSafe(MOTOR_PARAM_S *m);
static void Motor_Foc_OutputVoltage(MOTOR_PARAM_S *m, float vd, float vq, float theta, float v_bus);
static void Motor_It_ApplyControlResult(MOTOR_PARAM_S *m, float v_bus, const motor_control_result_t *result);
static bool Motor_DmaInt_ShouldRun(void);
static void Motor_DmaInt_Foc(void);
static void Motor_DmaInt_Bldc(void);
static void Motor_DmaInt_WhileDetect(void);
static void Motor_DmaInt_RunFastVf(MOTOR_PARAM_S *m);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 初始化默认 BLDC 六步与无感控制实例。
 * @detail: 只建立状态和 HAL 输出回调，不启动功率输出。
 * @return {bool} 初始化成功返回 true
 * @author: lkc
 */
static bool Motor_Bldc_InitDefault(void)
{
    motor_bldc_six_step_config_t six_config;
    motor_bldc_sensorless_config_t sensorless_config;

    Motor_BldcSixStep_DefaultConfig(&six_config);
    six_config.apply_output = Motor_BldcSixStep_HalApply;
    six_config.output_user = NULL;
    if (!Motor_BldcSixStep_Init(&s_bldc_commutator, &six_config)) {
        return false;
    }
    Motor_BldcSensorless_DefaultConfig(&sensorless_config);
    /* 24 V 实测 8%/12ms 基本同步，14ms 明显抖动；启动终点固定在已验证工作点。 */
    sensorless_config.startup_initial_period_s = 0.080f;
    sensorless_config.startup_final_period_s = 0.012f;
    sensorless_config.startup_ramp_time_s = 3.0f;
    sensorless_config.startup_timeout_s = 5.0f;
    sensorless_config.closed_loop_timeout_s = 0.150f;
    /* 积分阈值命中不得让换相周期单步骤降，否则转子会脱离旋转磁场。 */
    sensorless_config.period_change_limit = 0.10f;
    /* 当前阶段先锁住已验证的 12ms 工作点，确认实测 BEMF 极性后再开放调速范围。 */
    sensorless_config.closed_loop_min_period_s = 0.012f;
    /* 配置阈值仅作辨识失败回退；正常启动会从末段完整扇区自动学习 BEMF 面积。 */
    sensorless_config.bemf_integral_threshold_vs = 0.0004f;
    sensorless_config.bemf_integral_noise_v = 0.05f;
    /* 受限闭环实测：偶数 step 为正、奇数 step 为负，积分统一取正面积。 */
    sensorless_config.step0_rising = true;
    if (!Motor_BldcSensorless_Init(&s_bldc_sensorless, &s_bldc_commutator, &sensorless_config)) {
        return false;
    }
    s_bldc_initialized = true;
    return true;
}

/**
 * @description: 屏蔽 IRQ，供 BLDC 多字段状态快照使用。
 * @return {uint32_t} 进入前的 PRIMASK
 * @author: lkc
 */
static uint32_t Motor_Bldc_StatusLock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @description: 按进入前状态恢复 IRQ。
 * @param {uint32_t} primask 进入临界区前的 PRIMASK
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Bldc_StatusUnlock(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @description: 选择并启动默认无感 BLDC 六步实例。
 * @param {motor_bldc_direction_t} direction 转向
 * @param {float} duty PWM 占空比，范围 (0,0.25]
 * @return {bool} 启动成功返回 true
 * @author: lkc
 */
static bool Motor_Bldc_StartInternal(motor_bldc_direction_t direction, float duty,
                                     bool fixed_open_loop, float sector_period_s)
{
    bool started = false;

    if (motor.ol_enable || Motor_Protect_IsFaultLatched() || duty <= 0.0f || duty > 0.25f ||
        (direction != MOTOR_BLDC_DIRECTION_CW && direction != MOTOR_BLDC_DIRECTION_CCW)) {
        return false;
    }

    if (!s_bldc_initialized && !Motor_Bldc_InitDefault()) {
        return false;
    }
    if (s_bldc_sensorless.fault != MOTOR_BLDC_SENSORLESS_FAULT_NONE ||
        Motor_BldcSixStep_GetFault(&s_bldc_commutator) != MOTOR_BLDC_FAULT_NONE) {
        if (!Motor_Bldc_InitDefault()) {
            return false;
        }
    }
    /* 先占用出口，避免 FOC ISR 在恢复 TIM1 模式后写入中点占空比。 */
    s_bldc_selected = true;
    Hal_Motor_PwmStart();
    Hal_Motor_BldcPrepare();
    if (fixed_open_loop) {
        started = Motor_BldcSensorless_StartOpenLoop(&s_bldc_sensorless, direction, duty,
                                                     sector_period_s);
    } else {
        started = Motor_BldcSensorless_Start(&s_bldc_sensorless, direction, duty);
    }
    if (!started) {
        Hal_Motor_PwmStop();
        s_bldc_selected = false;
    }
    return started;
}

/**
 * @description: 选择并启动默认无感 BLDC 六步实例。
 * @param {motor_bldc_direction_t} direction 转向
 * @param {float} duty PWM 占空比，范围 (0,0.25]
 * @return {bool} 启动成功返回 true
 * @author: lkc
 */
bool Motor_Bldc_Start(motor_bldc_direction_t direction, float duty)
{
    return Motor_Bldc_StartInternal(direction, duty, false, 0.0f);
}

/**
 * @description: 选择并启动固定扇区周期的开环 BLDC 六步诊断。
 * @param {motor_bldc_direction_t} direction 转向
 * @param {float} duty PWM 占空比，范围 (0,0.25]
 * @param {float} sector_period_s 每个 60° 电气扇区周期 (s)
 * @return {bool} 启动成功返回 true
 * @author: lkc
 */
bool Motor_Bldc_StartOpenLoop(motor_bldc_direction_t direction, float duty,
                              float sector_period_s)
{
    return Motor_Bldc_StartInternal(direction, duty, true, sector_period_s);
}

/**
 * @description: 停机时更新 BLDC 积分阈值与最短换相周期。
 * @param {float} integral_threshold_vs BEMF 积分阈值 (V*s)
 * @param {float} minimum_period_s 闭环最短 60° 换相周期 (s)
 * @return {bool} 设置成功返回 true
 * @author: lkc
 */
bool Motor_Bldc_SetTuning(float integral_threshold_vs, float minimum_period_s)
{
    uint32_t primask;

    if (!(integral_threshold_vs >= 0.00005f && integral_threshold_vs <= 0.010f) ||
        !(minimum_period_s >= 0.0005f && minimum_period_s <= 0.100f)) {
        return false;
    }
    /* 允许上电后首次启动前调参；默认初始化只建立实例，不启动 PWM。 */
    if (!s_bldc_initialized && !Motor_Bldc_InitDefault()) {
        return false;
    }
    if (Motor_BldcSixStep_IsRunning(&s_bldc_commutator)) {
        return false;
    }
    primask = Motor_Bldc_StatusLock();
    s_bldc_sensorless.config.bemf_integral_threshold_vs = integral_threshold_vs;
    s_bldc_sensorless.config.closed_loop_min_period_s = minimum_period_s;
    Motor_Bldc_StatusUnlock(primask);
    return true;
}

/**
 * @description: 停止并退出默认 BLDC 控制模式。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Bldc_Stop(void)
{
    /* 保持 BLDC 出口选中，避免停机后 FOC SAFE 中点覆盖三相悬空。 */
    s_bldc_selected = true;
    if (s_bldc_initialized) {
        Motor_BldcSensorless_Stop(&s_bldc_sensorless);
    }
    Hal_Motor_PwmStop();
}

/**
 * @description: 停止 BLDC 并释放 PWM 出口选择权。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Bldc_Exit(void)
{
    if (s_bldc_initialized) {
        Motor_BldcSensorless_Stop(&s_bldc_sensorless);
    }
    Hal_Motor_PwmStop();
    s_bldc_selected = false;
}

/**
 * @description: 查询 BLDC 是否占用 PWM 控制出口。
 * @return {bool} BLDC 已选择返回 true
 * @author: lkc
 */
bool Motor_Bldc_IsSelected(void)
{
    return s_bldc_selected;
}

/**
 * @description: 获取默认 BLDC 控制状态快照。
 * @param {motor_bldc_status_t *} status 输出状态，不可为空
 * @return {bool} 获取成功返回 true
 * @author: lkc
 */
bool Motor_Bldc_GetStatus(motor_bldc_status_t *status)
{
    uint32_t primask;
    if (status == NULL) {
        return false;
    }
    primask = Motor_Bldc_StatusLock();
    memset(status, 0, sizeof(*status));
    status->selected = s_bldc_selected;
    if (s_bldc_initialized) {
        status->running = Motor_BldcSixStep_IsRunning(&s_bldc_commutator);
        status->duty = s_bldc_commutator.duty;
        status->step = s_bldc_sensorless.step;
        status->direction = s_bldc_sensorless.direction;
        status->mode = s_bldc_sensorless.mode;
        status->fault = s_bldc_sensorless.fault;
        status->motor_fault = Motor_Protect_GetFault();
        status->bemf_v = s_bldc_sensorless.filtered_bemf_v;
        status->bemf_min_v = s_bldc_sensorless.startup_bemf_min_v;
        status->bemf_max_v = s_bldc_sensorless.startup_bemf_max_v;
        status->sector_elapsed_s = s_bldc_sensorless.sector_elapsed_s;
        status->commutation_period_s = s_bldc_sensorless.filtered_commutation_period_s;
        status->minimum_commutation_period_s =
            s_bldc_sensorless.minimum_commutation_period_s;
        status->maximum_commutation_period_s =
            s_bldc_sensorless.maximum_commutation_period_s;
        status->bemf_integral_vs = s_bldc_sensorless.bemf_integral_vs;
        status->bemf_integral_threshold_vs =
            s_bldc_sensorless.config.bemf_integral_threshold_vs;
        status->bemf_sample_count = Hal_Motor_BldcGetSampleCount();
        status->integral_commutations = s_bldc_sensorless.integral_commutations;
        status->forced_commutations = s_bldc_sensorless.forced_commutations;
        memcpy(status->step_integral_commutations,
               s_bldc_sensorless.step_integral_commutations,
               sizeof(status->step_integral_commutations));
        memcpy(status->step_forced_commutations,
               s_bldc_sensorless.step_forced_commutations,
               sizeof(status->step_forced_commutations));
        memcpy(status->step_last_period_s, s_bldc_sensorless.step_last_period_s,
               sizeof(status->step_last_period_s));
        memcpy(status->step_bemf_min_v, s_bldc_sensorless.step_bemf_min_v,
               sizeof(status->step_bemf_min_v));
        memcpy(status->step_bemf_max_v, s_bldc_sensorless.step_bemf_max_v,
               sizeof(status->step_bemf_max_v));
        memcpy(status->step_bemf_waveform_v, s_bldc_sensorless.step_bemf_waveform_v,
               sizeof(status->step_bemf_waveform_v));
    }
    Motor_Bldc_StatusUnlock(primask);
    return true;
}

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
    float rpm = m->ol_target_rpm;
    float rpm_abs = fabsf(rpm);
    float control_hz = Motor_DmaInt_GetControlFrequencyHz();
    float theta_step;
    float vq = m->ol_v_boost;

    /* 低于实机可稳定拖动的转速时使用启动转速地板；保留目标方向。 */
    if (rpm_abs > 0.0f && rpm_abs < m->ol_min_rpm) {
        rpm = (rpm < 0.0f) ? -m->ol_min_rpm : m->ol_min_rpm;
    }
    if (control_hz <= 1.0f || m->ol_pole_pairs <= 0.0f) {
        Motor_OpenLoop_OutputSafe(m);
        return;
    }
    theta_step = RPM2RADPS_f(rpm) * m->ol_pole_pairs / control_hz;

    if (vq < 0.0f) {
        vq = 0.0f;
    }
    if (vq > m->ol_vq_limit) {
        vq = m->ol_vq_limit;
    }

    m->ol_current_rpm = rpm;
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
    motor_feedback_source_t feedback_source;
    motor_control_result_t result;

    Motor_It_ParamCal();
    protection_dt = Motor_OpenLoop_GetAdcDt(&motor);
    feedback_source = Motor_Feedback_GetSourceDefault();
    /*
     * 纯开环不用编码器/霍尔，不应在每个 30 kHz ADC 中断做它们的浮点计算。
     * 旧路径在 72 RPM 已占用约 90% CPU，会饿死 USART IDLE 中断和命令任务。
     * 被选为反馈源时仍保持原来的每 ADC 拍更新频率。
     */
    if (feedback_source == MOTOR_FEEDBACK_SRC_ENCODER) {
        Motor_Encoder_Update(protection_dt);
    }
    if (feedback_source == MOTOR_FEEDBACK_SRC_HALL) {
        Motor_Hall_AngleUpdateDefault(motor.hall_table, Motor_Hall_Read(false, motor.hall_extra_samples),
                                      motor.ol_pole_pairs, protection_dt);
    }

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
 * @description: 无感 BLDC 六步模式的 ADC-DMA 快速入口。
 * @detail: 软件保护处理全部 ADC 帧；BEMF/换相仅使用 PWM 向上计数侧样本，dt 为完整 PWM 周期。
 * @return {void} 无
 * @author: lkc
 */
static void Motor_DmaInt_Bldc(void)
{
    uint32_t runtime_start = Hal_Motor_TimeUs();
    float dt;
    float phase_voltage[3];

    Motor_It_ParamCal();
    dt = Motor_OpenLoop_GetAdcDt(&motor);
    if (Motor_Protect_Run(&motor, dt)) {
        Motor_BldcSensorless_Stop(&s_bldc_sensorless);
        Motor_DmaInt_RecordRuntime(runtime_start);
        return;
    }
    if (!Hal_Motor_BldcBemfSampleReady()) {
        Motor_DmaInt_RecordRuntime(runtime_start);
        return;
    }

    /*
     * BLDC 六步使用原理图 INHA/B/C 与 SENS1/2/3 的直接对应关系；这里把
     * FOC 的 ACB 运行量恢复为原理图 ABC 顺序，不改平台无关控制算法。
     */
    phase_voltage[0] = motor.va;
    phase_voltage[1] = motor.vc;
    phase_voltage[2] = motor.vb;
    if (!Motor_BldcSensorless_Step(&s_bldc_sensorless, phase_voltage, motor.v_bus, 2.0f * dt)) {
        /* BLDC 模式保持选中，防止下一拍 FOC SAFE 中点覆盖三相悬空。 */
        Motor_DmaInt_RecordRuntime(runtime_start);
        return;
    }
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

    if (s_bldc_selected) {
        Motor_DmaInt_Bldc();
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
