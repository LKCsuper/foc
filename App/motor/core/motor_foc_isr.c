/*
 * @Description: 正常 FOC 中断路径：采样、保护、控制与输出
 * @Author: lkc
 *
 * 一拍完整 FOC（ADC-DMA 中断内执行，不阻塞）：
 *
 *   Motor_FocIsr_Step
 *     └─ 未到控制拍（无感/开环按 FOC 分频；有感取 PWM 向下计数侧）→ 只发布状态快照
 *   Motor_FocIsr_Run
 *     1. 采样  Motor_FocIsr_Sample        ADC 帧 → ia/ib/ic、v_bus；控制周期 dt；传感器角度/速度
 *     2. 保护  Motor_FocIsr_Protect       周期无效、软件故障已锁存、或母线 < 1V → 复位控制、三相中点、本拍结束
 *     3. 电流  PrepareCurrentFeedback     高占空比相按 LONGEST_ZERO 重构；必须在保护之后
 *     4. 控制  Motor_Control_Step         启动/选角 → Clarke/Park → d/q PI → 矢量限幅 → vd/vq
 *     5. 输出  ArbitrateAndApply          故障最终仲裁 → 反 Park → SVM → A-C-B 提交 PWM
 *
 * 诊断（分段计时、故障轨迹、采样配对、电流跟踪、无感轨迹）只在各步之间的
 * Motor_FocIsr_Diag* 钩子里，均只写 RAM，不改控制量。
 * 速度/位置外环在 foc pid 任务中运行，本中断只读其 Iq 目标。
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_foc_isr.h"
#include "common.h"

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "hal_motor.h"
#include "motor.h"
#include "motor_control.h"
#include "motor_current.h"
#include "motor_crit.h"
#include "motor_detect.h"
#include "motor_encoder.h"
#include "motor_foc_divider.h"
#include "motor_hall.h"
#include "motor_hall_diag.h"
#include "motor_output.h"
#include "motor_protect.h"
#include "motor_sensor_update.h"
#include "motor_sense.h"
#include "motor_timing.h"
#include "motor_diag_phase.h"
#include "motor_sensorless.h"
#include "motor_speed.h"
#if defined(__CC_ARM)
#pragma O2
#endif
/* Private define ------------------------------------------------------------*/
/* 默认跳过拍每150次发布；无分频时每300个回调发布，避免没有跳过拍时快照停更。 */
#define MOTOR_STATUS_PUBLISH_DIVIDER 150U
/*
 * 完整控制拍半周期门控（divider=2 时生效）。依据 3000 RPM 实测（3000RPM平台DMA重叠验证_2026-10-21）：
 * 正常完整拍在顶部更新后约 9.4～11.2 us 进入；被上一拍拖后的回调在边界后约 28～31 us 才进入。
 * 窗口取 8～22 us：既接受正常节拍，也拒绝过晚的回调和刚越过边界、DMA 可能尚未写完的旧回调。
 * 定时器相位和每微秒计数由 HAL 提供；不改变窗口或外设配置。
 */
#define MOTOR_FOC_GATE_MIN_US        8U
#define MOTOR_FOC_GATE_MAX_US        22U
#define MOTOR_FOC_GATE_REPEAT_US     40U
/* Private typedef -----------------------------------------------------------*/
/* 一拍完整 FOC 的诊断状态，放在 Run 的栈上，不占静态 RAM。 */
typedef struct {
    uint32_t features;    /* Motor_Timing_IsrFeatures() 位掩码 */
    bool profile;         /* 本拍分段计时已打开 */
    uint32_t stage_start; /* 当前分段起点 (us) */
} motor_foc_isr_diag_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static motor_control_t s_control = {
    .startup =
        {
            .alignment = {.stopped = true},
            .if_start = {.first_cycle = true, .iq_scale = 1.0f},
        },
};
static motor_foc_divider_t s_divider = {
    .divider = FOC_CONTROL_LOOP_FREQ_DIVIDER,
};
static float s_recon_diag_target_rpm;
static bool s_recon_diag_enable;
static uint16_t s_status_publish_divider;
static motor_foc_phase_gate_t s_phase_gate;
static motor_foc_phase_gate_t s_control_deadline;
static motor_control_clock_t s_control_clock;
/* Private function prototypes -----------------------------------------------*/
static bool Motor_FocIsr_ShouldExecuteControl(motor_feedback_source_t feedback_source);
static bool Motor_FocIsr_Sample(MOTOR_PARAM_S *motor, motor_feedback_source_t feedback_source, uint32_t entry_us,
                                float *control_dt);
static bool Motor_FocIsr_Protect(MOTOR_PARAM_S *motor, motor_feedback_source_t feedback_source, bool timing_ok,
                                 float control_dt, uint32_t entry_us);
static void Motor_FocIsr_PrepareCurrentFeedback(MOTOR_PARAM_S *motor);
static motor_control_result_t Motor_FocIsr_RunControl(MOTOR_PARAM_S *motor, float control_dt);
static void Motor_FocIsr_ArbitrateAndApply(MOTOR_PARAM_S *motor, motor_control_result_t result);
static void Motor_FocIsr_PublishStatus(const MOTOR_PARAM_S *motor_param);
static bool Motor_FocIsr_ProfileOpen(uint32_t entry_us, uint32_t *stage_start);
static void Motor_FocIsr_ProfileMark(motor_timing_stage_t stage, uint32_t *stage_start);
static void Motor_FocIsr_RecordPair(const MOTOR_PARAM_S *motor_param, uint32_t entry_us);
static void Motor_FocIsr_RecordTrace(const MOTOR_PARAM_S *motor_param, motor_feedback_source_t feedback_source, float vd,
                                     float vq);
static void Motor_FocIsr_RecordExit(const MOTOR_PARAM_S *motor_param, motor_feedback_source_t feedback_source,
                                    uint32_t entry_us, uint32_t diagnostics);
static void Motor_FocIsr_RecordFault(const MOTOR_PARAM_S *motor, uint32_t entry_us, float dt, bool force);
static motor_current_tracking_diag_t s_tracking;
static bool s_tracking_enabled;

/**
 * @description: 查询电流跟踪观察是否打开。
 * @detail: 只读开关，不改采集或控制。
 * @return {bool} true=已打开
 * @author: lkc
 */
bool Motor_FocIsr_TrackingIsEnabled(void) { return s_tracking_enabled; }

/**
 * @description: 停机时打开或关闭电流跟踪观察。
 * @detail: 默认关闭。运行中返回 false，不改开关。打开时清历史；关闭时清当前累计，保留已记下的事件。
 *          不改电流或保护。
 * @param {bool} enabled true=打开
 * @return {bool} 已写入返回 true；运行中返回 false
 * @author: lkc
 */
bool Motor_FocIsr_EnableTracking(bool enabled)
{
    uint32_t key = Motor_Crit_Enter();
    if (motor.ol_enable) {
        Motor_Crit_Exit(key);
        return false;
    }
    if (enabled) {
        motor_current_tracking_diag_t empty = {0};
        s_tracking = empty;
    }
    s_tracking_enabled = enabled;
    if (!enabled) {
        s_tracking.active = s_tracking.over = false;
        s_tracking.over_time_s = 0.0f;
    }
    Motor_Crit_Exit(key);
    return true;
}

/**
 * @description: 停机时读取或清零默认 ISR 的电流跟踪观察。
 * @detail: 运行中返回 false，且不写出 diag。diag 不可为空，违反约定触发 COMMON_ASSERT。
 *          clear 为真时在拷贝后清零。清零不影响故障或其他记录。不改控制。
 * @param {motor_current_tracking_diag_t *} diag 输出，不可为空
 * @param {bool} clear                           true=读出后清零
 * @return {bool} 已读出返回 true；运行中返回 false
 * @author: lkc
 */
bool Motor_FocIsr_ReadTracking(motor_current_tracking_diag_t *diag, bool clear)
{
    uint32_t key;
    COMMON_ASSERT(diag != NULL);
    key = Motor_Crit_Enter();
    if (motor.ol_enable) {
        Motor_Crit_Exit(key);
        return false;
    }
    *diag = s_tracking;
    if (clear) {
        motor_current_tracking_diag_t empty = {0};
        s_tracking = empty;
    }
    Motor_Crit_Exit(key);
    return true;
}
/**
 * @description: 组装一条完整轨迹样本。
 * @detail: 只读现有诊断字段。motor_param 与 sample 不可为空，违反约定触发 COMMON_ASSERT。
 *          间隔统计字段写 0，由 TracePush 覆盖。不提交 PWM，不改控制。
 * @param {const MOTOR_PARAM_S *} motor_param 电机对象，不可为空
 * @param {float} vd                          本拍 d 轴电压 (V)
 * @param {float} vq                          本拍 q 轴电压 (V)
 * @param {motor_trace_sample_t *} sample     输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_FocIsr_CaptureTraceSample(const MOTOR_PARAM_S *motor_param, float vd, float vq, motor_trace_sample_t *sample)
{
    motor_sensorless_diagnostics_t d;
    COMMON_ASSERT(motor_param != NULL && sample != NULL);
    Motor_Sensorless_GetDiagnostics(&d);
    sample->time_us = Hal_Motor_TimeUs();
    sample->flags =
        (unsigned)d.state | (d.flux_ok ? 4U : 0U) | (d.direction_ok ? 8U : 0U) |
        (Motor_Encoder_IsSpeedValid() && Motor_Encoder_IsZeroValid() && !Motor_Encoder_IsSignalFault() ? 16U : 0U);
    sample->theta = motor_param->ol_theta;
    sample->pll = d.pll_angle;
    sample->flux = d.flux_magnitude;
    sample->rpm = d.mechanical_rpm;
    sample->id = motor_param->Id;
    sample->iq = motor_param->Iq;
    sample->iq_target = Motor_SpeedLoop_GetIqTarget();
    sample->v_alpha = d.v_alpha;
    sample->v_beta = d.v_beta;
    sample->enc_angle = Motor_Encoder_GetElecAngle();
    sample->enc_rpm = Motor_Encoder_GetMechRpm();
    sample->enc_age = Hal_Motor_TimeElapsedSec(Motor_Encoder_GetSampleTimeUs());
    sample->blend = d.blend;
    sample->loss_time = d.loss_timer;
    sample->id_cmd = motor_param->id_target;
    sample->iq_cmd = motor_param->iq_target;
    sample->i_alpha = motor_param->Ialpha;
    sample->i_beta = motor_param->Ibeta;
    sample->vd = vd;
    sample->vq = vq;
    sample->v_bus = motor_param->v_bus;
    sample->v_limit = Motor_Control_GetVoltageLimit(motor_param, motor_param->v_bus);
    sample->gap_max_us = sample->full_max_us = sample->pending_count = sample->up_count = 0U;
}

static void Motor_FocIsr_Run(MOTOR_PARAM_S *motor_param, motor_feedback_source_t feedback_source, uint32_t entry_us);

/**
 * @description: 在无感路径写入一条紧凑轨迹。
 * @detail: 紧凑记录未采集时直接返回。是否写入由 TraceCompactDue 决定。只写 RAM，不改控制。
 *          调用者保证 motor_param 非空。
 * @param {const MOTOR_PARAM_S *} motor_param 电机对象，不可为空
 * @param {uint32_t} entry_us                 本拍入口时间 (us)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_RecordCompact(const MOTOR_PARAM_S *motor_param, uint32_t entry_us)
{
    motor_trace_compact_sample_t sample;
    unsigned state;
    if (!Motor_Timing_TraceCompactIsActive()) return;
    state = (unsigned)Motor_Sensorless_GetState();
    if (!Motor_Timing_TraceCompactDue(entry_us, state, (unsigned)Motor_Protect_GetFault(), motor_param->ol_enable))
        return;
    sample.time_us = entry_us;
    sample.flags = 0U;
    sample.theta = motor_param->ol_theta;
    sample.rpm = Motor_Sensorless_GetMechanicalRpm();
    sample.id = motor_param->Id;
    sample.iq = motor_param->Iq;
    sample.id_cmd = motor_param->id_target;
    sample.iq_cmd = motor_param->iq_target;
    Motor_Timing_TracePushCompact(&sample);
}
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 判断本次 ADC 回调是否承担一拍完整 FOC。
 * @detail: 霍尔和编码器固定在 PWM 向下计数侧执行；其他路径沿用 FOC 分频。不改控制。
 * @param {motor_feedback_source_t} feedback_source 当前反馈源
 * @return {bool} true=继续执行采样、保护和控制
 * @author: lkc
 */
static bool Motor_FocIsr_ShouldExecuteControl(motor_feedback_source_t feedback_source)
{
    hal_motor_pwm_phase_t phase;
    motor_foc_phase_gate_config_t config;

    /* divider=2：所有反馈源统一按 PWM 半周期选拍。原无感/I/F 路径用软件计数，回调被推迟或 DMA 标志合并时
     * 计数奇偶翻转，完整拍会跳到另一半周期（实测 100 us 间隔后换半周期）；Hall/编码器原本只看 DIR。 */
    if (s_divider.divider == 2) {
        Hal_Motor_PwmPhaseCapture(&phase);
        config.min_counts = MOTOR_FOC_GATE_MIN_US * phase.counts_per_us;
        config.max_counts = MOTOR_FOC_GATE_MAX_US * phase.counts_per_us;
        config.repeat_us = MOTOR_FOC_GATE_REPEAT_US;
        return Motor_FocDivider_PhaseGate(&s_phase_gate, &config, phase.counting_down, phase.count,
                                          phase.period, Hal_Motor_TimeUs());
    }
    if (feedback_source == MOTOR_FEEDBACK_SRC_HALL || feedback_source == MOTOR_FEEDBACK_SRC_ENCODER) {
        Hal_Motor_PwmPhaseCapture(&phase);
        return phase.counting_down;
    }
    return Motor_FocIsr_ShouldRun();
}

/**
 * @description: 判断本拍细分计时是否属于无感 I/F 运行窗口。
 * @detail: 细分开关关闭时直接返回 false，避免停机拍读取反馈源。
 *          只在开环使能、无感使能、I/F 模式且反馈源为无感时返回 true。
 *          停机拍仍走外层 SAMPLE，不进入这三段。
 * @param {const MOTOR_PARAM_S *} motor 电机运行时，调用者保证非空
 * @param {motor_feedback_source_t} source 本拍反馈源
 * @return {bool} true=本拍可记录采样、反馈和保护细分
 * @author: lkc
 */
static bool Motor_FocIsr_ProfileRunning(const MOTOR_PARAM_S *motor, motor_feedback_source_t source)
{
    return Motor_Timing_ProfileDetailedIsActive() && motor->ol_enable && motor->sensorless_enable &&
           motor->ol_mode == MOTOR_OL_MODE_IF && source == MOTOR_FEEDBACK_SRC_SENSORLESS;
}

/**
 * @description: 捕获本拍样本并更新传感器。
 * @detail: 这里不判故障、不改 PWM。控制周期异常时不更新传感器，交由保护入口停机。
 *          采样与传感器细分只在无感 I/F 运行窗口计入。
 * @param {MOTOR_PARAM_S *} motor               电机运行时，调用者保证非空
 * @param {motor_feedback_source_t} feedback_source 当前反馈源
 * @param {uint32_t} entry_us                   本次完整控制入口时间 (us)
 * @param {float *} control_dt                 输出本拍控制周期 (s)，调用者保证非空
 * @return {bool} true=控制周期有效
 * @author: lkc
 */
static bool Motor_FocIsr_Sample(MOTOR_PARAM_S *motor, motor_feedback_source_t feedback_source, uint32_t entry_us,
                                float *control_dt)
{
    bool profile = Motor_FocIsr_ProfileRunning(motor, feedback_source);
    uint32_t stage_start;
    Motor_Timing_RecordSampleDirection((TIM1->CR1 & TIM_CR1_DIR) == 0U);
    Motor_Hall_DiagNoteTimer((uint16_t)TIM1->CNT, (uint16_t)TIM1->ARR);
    MOTOR_PROFILE_BEGIN(profile, stage_start);
    Motor_Sense_Update(motor);
    MOTOR_PROFILE_END(profile, MOTOR_TIMING_SENSE_FRAME, stage_start);
    /* 仅无感 I/F 路径采用完整控制入口间隔，其他反馈/模式保持名义周期。 */
    if (!Motor_Timing_ControlClockStep(&s_control_clock,
                                       motor->ol_enable && motor->ol_mode == MOTOR_OL_MODE_IF &&
                                           feedback_source == MOTOR_FEEDBACK_SRC_SENSORLESS,
                                       entry_us, Motor_Timing_GetControlDt(motor), control_dt)) {
        return false;
    }
    MOTOR_PROFILE_BEGIN(profile, stage_start);
    Motor_SensorUpdate_Step(motor, feedback_source, *control_dt);
    MOTOR_PROFILE_END(profile, MOTOR_TIMING_SENSOR_FEEDBACK, stage_start);
    return true;
}

/**
 * @description: 判断本拍能不能进入电流环。
 * @detail: 下面三件事按顺序看，命中就清掉启动状态并输出三相中点，本拍到此结束。
 *          中点是 50% 占空比，桥还在导通，不是关桥。硬件过流关断不经过这里。
 *          1) 控制周期无效：立刻锁存 CONTROL_TIMING。本拍不跑过流/欠压去抖，因为 dt 不能用。
 *          2) 周期有效：跑软件保护。过流、欠压、堵转、失速谁优先、怎么去抖，见 motor_protect_core.c。
 *             故障一旦锁存，之后每拍都算命中，直到 ClearFault。第一次锁存时抄一帧电流和定时器计数。
 *          3) 没有故障、但母线低于 1 V：同样停在中点，不等欠压去抖计满。1 V 和控制允许起转的母线门槛相同。
 *          这里用的是采样换算后的原始相电流。某一相采样窗不够时的电流重构，在本函数返回 true 之后才做。
 * @param {MOTOR_PARAM_S *} motor 电机运行时，调用者保证非空
 * @param {motor_feedback_source_t} feedback_source 本拍反馈源，只用于分段计时是否打开
 * @param {bool} timing_ok            true=本拍控制周期有效
 * @param {float} control_dt          本拍控制周期 (s)；周期无效时不使用
 * @param {uint32_t} entry_us         进入本拍控制的时间 (us)，不是 ADC 采样时刻
 * @return {bool} true=继续电流环；false=已经输出三相中点
 * @author: lkc
 */
static bool Motor_FocIsr_Protect(MOTOR_PARAM_S *motor, motor_feedback_source_t feedback_source, bool timing_ok,
                                 float control_dt, uint32_t entry_us)
{
    bool profile = Motor_FocIsr_ProfileRunning(motor, feedback_source);
    bool faulted;
    uint32_t stage_start;

    if (!timing_ok) {
        /* 周期无效，不能拿这个 dt 给过流/欠压去抖。 */
        Motor_Protect_TriggerFault(FAULT_CODE_CONTROL_TIMING, motor);
        faulted = true;
    } else {
        /* 过流、欠压、堵转、失速。已经锁存的故障这里一直返回 true。 */
        MOTOR_PROFILE_BEGIN(profile, stage_start);
        faulted = Motor_Protect_Run(motor, control_dt);
        if (faulted && Motor_Protect_FaultSampleIsPending())
            Motor_Protect_CaptureFaultSample(motor, Hal_Motor_CurrentScale(), entry_us, (uint16_t)TIM1->CNT,
                                             (uint16_t)TIM1->ARR, (uint16_t)TIM1->CR1);
        MOTOR_PROFILE_END(profile, MOTOR_TIMING_PROTECTION, stage_start);
    }
    if (faulted) {
        /* 记下这一拍并冻结轨迹，后面的拍不再覆盖第一次故障。 */
        Motor_FocIsr_RecordFault(motor, entry_us, control_dt, true);
        Motor_Timing_FaultTraceFreeze((unsigned)Motor_Protect_GetFault());
    }
    /* 没有故障，并且母线至少 1 V，才进入电流环。 */
    if (!faulted && motor->v_bus >= 1.0f) return true;

    /* 有故障，或母线还不到 1 V：清启动状态，三相输出中点。 */
    Motor_Control_ResetRuntime(&s_control, motor);
    Motor_OutputSafe(motor);
    return false;
}

/**
 * @description: 在 Clarke/Park 前准备可信的三相电流。
 * @detail: 保留 VESC LONGEST_ZERO 语义，只在唯一最大占空相采样窗不足时重构该相。
 * @param {MOTOR_PARAM_S *} motor 电机运行时，调用者保证非空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_PrepareCurrentFeedback(MOTOR_PARAM_S *motor)
{
    if (motor->ol_enable && (!s_recon_diag_enable || s_recon_diag_target_rpm != motor->ol_target_rpm)) {
        Motor_Current_ResetReconstructDiag();
        s_recon_diag_target_rpm = motor->ol_target_rpm;
    }
    s_recon_diag_enable = motor->ol_enable;
    (void)Motor_Current_ReconstructLongestZero(&motor->ia, &motor->ib, &motor->ic, motor->duty1, motor->duty2,
                                               motor->duty3, Hal_Motor_PwmPeriodCounts());
}

/**
 * @description: 执行不接触 PWM 寄存器的控制核心。
 * @detail: 内部按模式完成启动/角度选择，电流闭环路径再执行 Clarke/Park、PI 和电压限幅。
 * @param {MOTOR_PARAM_S *} motor 电机运行时，调用者保证非空
 * @param {float} control_dt          本拍控制周期 (s)
 * @return {motor_control_result_t} SAFE 或 DQ 电压请求
 * @author: lkc
 */
static motor_control_result_t Motor_FocIsr_RunControl(MOTOR_PARAM_S *motor, float control_dt)
{
    motor_control_result_t result;

    result = Motor_Control_Step(&s_control, motor, motor->v_bus, control_dt,
                                Motor_Hall_DetectIsActive() || Motor_Encoder_DetectIsActive() ||
                                    Motor_Anticogging_IsLearning());
    Motor_Hall_DiagUpdate();
    return result;
}

/**
 * @description: 执行最终故障仲裁，并把结果提交到硬件。
 * @detail: 已锁存时先清运行时再仲裁。非 I/F 走 V/F；I/F 按反馈源选择无感、有感或 V/F。
 *          霍尔未启用 VESC SVM 时仍走 V/F。本函数不直接写比较值。
 * @param {MOTOR_PARAM_S *} motor          电机运行时，调用者保证非空
 * @param {motor_control_result_t} result  控制核心给出的候选输出
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_ArbitrateAndApply(MOTOR_PARAM_S *motor, motor_control_result_t result)
{
    bool fault_latched = Motor_Protect_IsFaultLatched();

    if (fault_latched) {
        Motor_Control_ResetRuntime(&s_control, motor);
    }
    result = Motor_Control_ArbitrateOutput(fault_latched, &result);
    /* 控制集成层选择一次输出路径，各输出实现不再猜测模式。
     * Hall 禁用 VESC SVM 时仍保留旧 HQD 输出，反馈来源集合与原分支一致。 */
    if (motor->ol_mode != MOTOR_OL_MODE_IF) {
        Motor_OutputApplyControlResultForPath(motor, motor->v_bus, &result, MOTOR_OUTPUT_PATH_VF);
    } else {
        motor_feedback_source_t source = Motor_Feedback_GetSourceDefault();
        motor_output_path_t path = MOTOR_OUTPUT_PATH_VF;
        if (source == MOTOR_FEEDBACK_SRC_SENSORLESS || source == MOTOR_FEEDBACK_SRC_OPENLOOP) {
            path = MOTOR_OUTPUT_PATH_SENSORLESS_IF;
        } else if (source == MOTOR_FEEDBACK_SRC_ENCODER ||
                   (source == MOTOR_FEEDBACK_SRC_HALL && MOTOR_HALL_USE_VESC_SVM)) {
            path = MOTOR_OUTPUT_PATH_SENSORED_IF;
        }
        Motor_OutputApplyControlResultForPath(motor, motor->v_bus, &result, path);
    }
}

/**
 * @description: 查询开环对齐是否仍在进行。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {bool} 对齐阶段未结束返回 true
 * @author: lkc
 */
bool Motor_OpenLoop_IsAligning(void)
{
    return Motor_Control_IsAligning(&s_control);
}

/**
 * @description: 读取当前对齐已用时间。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {float} 对齐已用时间 (s)
 * @author: lkc
 */
float Motor_OpenLoop_GetAlignElapsed(void)
{
    return Motor_Control_GetAlignElapsed(&s_control);
}

/**
 * @description: 查询 HFI 启动是否仍在运行。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {bool} HFI 启动进行中返回 true
 * @author: lkc
 */
bool Motor_HfiStart_IsRunning(void)
{
    return Motor_Control_HfiIsRunning(&s_control);
}

/**
 * @description: 查询最近一次 HFI 启动是否成功。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {bool} 成功返回 true
 * @author: lkc
 */
bool Motor_HfiStart_Succeeded(void)
{
    return Motor_Control_HfiSucceeded(&s_control);
}

/**
 * @description: 读取 HFI 启动给出的电角度。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {float} 电角度 (rad)
 * @author: lkc
 */
float Motor_HfiStart_GetAngle(void)
{
    return Motor_Control_HfiGetAngle(&s_control);
}

/**
 * @description: 读取 HFI 启动置信度。
 * @detail: 读取正常 FOC 路径持有的启动状态，可在任务中调用。
 * @return {float} 置信度，无量纲
 * @author: lkc
 */
float Motor_HfiStart_GetConfidence(void)
{
    return Motor_Control_HfiGetConfidence(&s_control);
}

/**
 * @description: 查询本拍是否到达 FOC 控制分频点。
 * @detail: 与快速 V/F 共用同一分频器，保持原有分频相位。在 ADC 中断调用。
 * @return {bool} 本拍应执行控制返回 true
 * @author: lkc
 */
bool Motor_FocIsr_ShouldRun(void)
{
    return Motor_FocDivider_ShouldRun(&s_divider);
}

/**
 * @description: 打开本拍分段耗时采样，并记下节拍方向。
 * @detail: 每 256 个控制拍采样一次。stage_start 不可为空，违反约定触发 COMMON_ASSERT。不改控制量。
 * @param {uint32_t} entry_us 本拍入口时间戳 (us)
 * @param {uint32_t *} stage_start 输出第一段起点 (us)，不可为空
 * @return {bool} true=本拍后续各段会计入统计
 * @author: lkc
 */
static bool Motor_FocIsr_ProfileOpen(uint32_t entry_us, uint32_t *stage_start)
{
    bool profile;

    COMMON_ASSERT(stage_start != NULL);
    profile = Motor_Timing_ProfileBegin();
    Motor_Timing_CadenceNote(entry_us, (TIM1->CR1 & TIM_CR1_DIR) == 0U);
    *stage_start = profile ? Hal_Motor_TimeUs() : 0U;
    return profile;
}

/**
 * @description: 记下刚结束的阶段，并作为下一段的起点。
 * @detail: 仅在本拍分段计时开启时调用。stage_start 不可为空，违反约定触发 COMMON_ASSERT。
 *          最后一段由调用者直接 ProfileRecord，避免多读一次时间。
 * @param {motor_timing_stage_t} stage 刚结束的阶段
 * @param {uint32_t *} stage_start     本段起点 (us)，返回时改为下一段起点
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_ProfileMark(motor_timing_stage_t stage, uint32_t *stage_start)
{
    COMMON_ASSERT(stage_start != NULL);
    Motor_Timing_ProfileRecord(stage, *stage_start);
    *stage_start = Hal_Motor_TimeUs();
}

/**
 * @description: 在电流阶段记录一帧控制拍配对。
 * @detail: 半周期配对由 DMA 入口采集，这里不重复。只写 RAM，不改控制量。
 * @param {const MOTOR_PARAM_S *} motor_param 电机运行时，不可为空
 * @param {uint32_t} entry_us                 本拍入口时间 (us)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_RecordPair(const MOTOR_PARAM_S *motor_param, uint32_t entry_us)
{
    motor_pair_frame_t frame;
    int reconstructed;

    COMMON_ASSERT(motor_param != NULL);
    if (!Motor_Timing_PairActive() || Motor_Timing_PairHalfMode()) return;
    frame.sample_sequence = 0U;
    frame.pwm_sequence = Hal_Motor_PwmDiagnosticSequence();
    frame.entry_us = entry_us;
    frame.callback_count = (uint16_t)TIM1->CNT;
    frame.callback_cr1 = (uint16_t)TIM1->CR1;
    frame.ccr[0] = (uint16_t)TIM1->CCR1;
    frame.ccr[1] = (uint16_t)TIM1->CCR2;
    frame.ccr[2] = (uint16_t)TIM1->CCR3;
    frame.period = (uint16_t)TIM1->ARR;
    frame.adc[0] = motor_param->currentAdc[0];
    frame.adc[1] = motor_param->currentAdc[1];
    frame.adc[2] = motor_param->currentAdc[2];
    frame.ia = motor_param->ia;
    frame.ib = motor_param->ib;
    frame.ic = motor_param->ic;
    Motor_Current_GetReconstructDiag(&reconstructed, NULL, NULL, NULL, NULL, NULL);
    frame.reconstructed = (uint16_t)reconstructed;
    if (Motor_Timing_PairFrame(&frame)) {
        motor_sensorless_light_snapshot_t observer;
        Motor_Sensorless_GetLightSnapshot(&observer);
        Motor_Timing_PairPush(observer.v_alpha, observer.v_beta);
    }
}

/**
 * @description: 在输出提交后写入一条无感轨迹。
 * @detail: 是否写入由 TraceDue 决定。轻量格式附带速度任务证据，否则写完整样本。
 *          只写 RAM，不改本拍控制结果。motor_param 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {const MOTOR_PARAM_S *} motor_param       电机运行时，不可为空
 * @param {motor_feedback_source_t} feedback_source 当前反馈源
 * @param {float} vd                                本拍 d 轴电压 (V)
 * @param {float} vq                                本拍 q 轴电压 (V)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_RecordTrace(const MOTOR_PARAM_S *motor_param, motor_feedback_source_t feedback_source,
                                     float vd, float vq)
{
    COMMON_ASSERT(motor_param != NULL);
    if (!motor_param->ol_enable || feedback_source != MOTOR_FEEDBACK_SRC_SENSORLESS || Motor_Timing_TraceIsCompact() ||
        !Motor_Timing_TraceDue(Motor_Sensorless_GetState())) {
        return;
    }
    if (Motor_Timing_TraceIsLight()) {
        motor_sensorless_light_snapshot_t d;
        motor_trace_light_sample_t t;
        Motor_Sensorless_GetLightSnapshot(&d);
        t.time_us = Hal_Motor_TimeUs();
        t.flags = (unsigned)d.state | (d.flux_ok ? 4U : 0U) | (d.direction_ok ? 8U : 0U);
        t.theta = motor_param->ol_theta;
        t.pll = d.pll;
        t.flux = d.flux;
        t.rpm = d.rpm;
        t.id = motor_param->Id;
        t.iq = motor_param->Iq;
        t.id_cmd = motor_param->id_target;
        t.iq_cmd = motor_param->iq_target;
        t.blend = d.blend;
        t.loss = d.loss;
        t.v_alpha = d.v_alpha;
        t.v_beta = d.v_beta;
        t.gap_max_us = t.full_max_us = 0U;
        Motor_Timing_SpeedTaskCaptureLight(&t);
        Motor_Timing_TracePushLight(&t);
    } else {
        motor_trace_sample_t sample;
        Motor_FocIsr_CaptureTraceSample(motor_param, vd, vq, &sample);
        Motor_Timing_TracePush(&sample);
    }
}

/**
 * @description: 结束本拍诊断：无感紧凑轨迹和入口耗时。
 * @detail: 未到控制拍的提前返回只记入口耗时，不调用本函数。
 * @param {const MOTOR_PARAM_S *} motor_param 电机运行时，不可为空
 * @param {motor_feedback_source_t} feedback_source 当前反馈源
 * @param {uint32_t} entry_us                       本拍入口时间 (us)
 * @param {uint32_t} diagnostics                    本拍诊断入口位掩码
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_RecordExit(const MOTOR_PARAM_S *motor_param, motor_feedback_source_t feedback_source,
                                    uint32_t entry_us, uint32_t diagnostics)
{
    COMMON_ASSERT(motor_param != NULL);
    if ((diagnostics & MOTOR_TIMING_ISR_COMPACT) && feedback_source == MOTOR_FEEDBACK_SRC_SENSORLESS)
        Motor_FocIsr_RecordCompact(motor_param, entry_us);
    if (diagnostics & MOTOR_TIMING_ISR_RUNTIME) Motor_Timing_RecordRuntime(entry_us);
}

/**
 * @description: 在采样入口按分频保存故障前数据，故障时强制保留本拍，不修改控制量。
 * @detail: 只在 ADC 中断调用。先判断是否到记录拍，再组装样本。ADC 与 abc 是本拍，其余控制量是此前结果。
 *          调用者保证 motor 非空。不改控制量。
 * @param {const MOTOR_PARAM_S *} motor 电机对象，不可为空
 * @param {uint32_t} entry_us           本次控制入口时间 (us)
 * @param {float} dt                    本拍控制周期 (s)，异常时可为 0
 * @param {bool} force                  true=强制记录故障拍，同一入口不重复
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_RecordFault(const MOTOR_PARAM_S *motor, uint32_t entry_us, float dt, bool force)
{
    motor_fault_trace_sample_t sample;
    if (!Motor_Timing_FaultTraceDue(entry_us, motor->ol_enable, force)) return;
    Motor_FocIsr_CaptureFaultTraceSample(motor, entry_us, dt, &sample);
    Motor_Timing_FaultTracePush(&sample);
}

/**
 * @description: 组装一条故障轨迹样本。
 * @detail: 不访问 ADC 寄存器，不提交 PWM，不改控制状态。ADC 与 abc 取当前输入，角度和观测取此前结果。
 *          motor 与 sample 不可为空，违反约定触发 COMMON_ASSERT。flags 的 bit8 表示跟踪事件已记下。
 * @param {const MOTOR_PARAM_S *} motor         电机对象，不可为空
 * @param {uint32_t} entry_us                   控制入口或基准时间 (us)
 * @param {float} dt                            控制周期 (s)
 * @param {motor_fault_trace_sample_t *} sample 输出样本，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_FocIsr_CaptureFaultTraceSample(const MOTOR_PARAM_S *motor, uint32_t entry_us, float dt,
                                        motor_fault_trace_sample_t *sample)
{
    motor_sensorless_light_snapshot_t observer;
    COMMON_ASSERT(motor != NULL && sample != NULL);
    Motor_Sensorless_GetLightSnapshot(&observer);
    sample->time_us = entry_us;
    sample->flags = (unsigned)observer.state | (s_tracking.event_latched ? 256U : 0U);
    sample->adc[0] = (uint16_t)motor->currentAdc[0];
    sample->adc[1] = (uint16_t)motor->currentAdc[1];
    sample->adc[2] = (uint16_t)motor->currentAdc[2];
    sample->reserved = 0U;
    sample->ia = motor->ia; sample->ib = motor->ib; sample->ic = motor->ic;
    sample->theta = motor->theta; sample->pll = observer.pll;
    sample->flux = observer.flux; sample->rpm = observer.rpm;
    sample->v_alpha = observer.v_alpha; sample->v_beta = observer.v_beta;
    sample->vd_int = motor->ol_vd_int; sample->vq_int = motor->ol_vq_int;
    sample->id = motor->Id; sample->iq = motor->Iq;
    sample->id_cmd = motor->id_target; sample->iq_cmd = motor->iq_target;
    sample->v_bus = motor->v_bus; sample->dt = dt;
}

/**
 * @description: 诊断钩子：本拍开始，读取诊断开关并按需打开分段计时。
 * @detail: 只写 RAM，不改控制量。调用者保证 diag 非空。
 * @param {motor_foc_isr_diag_t *} diag 本拍诊断状态
 * @param {uint32_t} entry_us           本拍入口时间 (us)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_DiagBegin(motor_foc_isr_diag_t *diag, uint32_t entry_us)
{
    diag->features = Motor_Timing_IsrFeatures();
    diag->stage_start = 0U;
    diag->profile =
        (diag->features & MOTOR_TIMING_ISR_PROFILE) ? Motor_FocIsr_ProfileOpen(entry_us, &diag->stage_start) : false;
}

/**
 * @description: 诊断钩子：采样后、保护前，按分频写故障前轨迹。
 * @detail: 记录的 ADC 与 abc 是本拍值，其余控制量是上一拍结果。只写 RAM。
 * @param {const motor_foc_isr_diag_t *} diag 本拍诊断状态
 * @param {const MOTOR_PARAM_S *} motor       电机对象
 * @param {uint32_t} entry_us                 本拍入口时间 (us)
 * @param {float} control_dt                  本拍控制周期 (s)，时序异常时为 0
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_DiagSampled(const motor_foc_isr_diag_t *diag, const MOTOR_PARAM_S *motor, uint32_t entry_us,
                                     float control_dt)
{
    if (diag->features & MOTOR_TIMING_ISR_FAULT) Motor_FocIsr_RecordFault(motor, entry_us, control_dt, false);
}

/**
 * @description: 诊断钩子：保护已停机，结束本拍诊断。
 * @detail: 电流跟踪按不适用工况清计时，关闭分段计时并记录入口耗时。只写 RAM。
 * @param {const motor_foc_isr_diag_t *} diag     本拍诊断状态
 * @param {const MOTOR_PARAM_S *} motor           电机对象
 * @param {motor_feedback_source_t} feedback_source 本拍反馈源
 * @param {uint32_t} entry_us                     本拍入口时间 (us)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_DiagStopped(const motor_foc_isr_diag_t *diag, const MOTOR_PARAM_S *motor,
                                     motor_feedback_source_t feedback_source, uint32_t entry_us)
{
    if (s_tracking_enabled)
        Motor_Current_TrackingObserve(&s_tracking, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, entry_us);
    if (diag->profile) Motor_Timing_ProfileEnd();
    Motor_FocIsr_RecordExit(motor, feedback_source, entry_us, diag->features);
}

/**
 * @description: 诊断钩子：结束一个分段计时阶段并开始下一段。
 * @detail: 本拍未打开分段计时时不读时基。只写 RAM。
 * @param {motor_foc_isr_diag_t *} diag 本拍诊断状态
 * @param {motor_timing_stage_t} stage  刚结束的阶段
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_DiagMark(motor_foc_isr_diag_t *diag, motor_timing_stage_t stage)
{
    if (diag->profile) Motor_FocIsr_ProfileMark(stage, &diag->stage_start);
}

/**
 * @description: 诊断钩子：电流准备完成后记录采样配对，并结束 CURRENT 段。
 * @detail: 只写 RAM，不改电流。
 * @param {motor_foc_isr_diag_t *} diag 本拍诊断状态
 * @param {const MOTOR_PARAM_S *} motor 电机对象，ia/ib/ic 已完成重构
 * @param {uint32_t} entry_us           本拍入口时间 (us)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_DiagCurrentReady(motor_foc_isr_diag_t *diag, const MOTOR_PARAM_S *motor, uint32_t entry_us)
{
    if (diag->features & MOTOR_TIMING_ISR_PAIR) Motor_FocIsr_RecordPair(motor, entry_us);
    Motor_FocIsr_DiagMark(diag, MOTOR_TIMING_CURRENT);
}

/**
 * @description: 诊断钩子：控制完成后观察电流跟踪，并结束 CONTROL 段。
 * @detail: 跟踪只在无感 I/F 拖转稳定、未接管、无速度环和弱磁且输出 DQ 时计入。只写 RAM。
 * @param {motor_foc_isr_diag_t *} diag            本拍诊断状态
 * @param {const MOTOR_PARAM_S *} motor            电机对象
 * @param {motor_feedback_source_t} feedback_source 本拍反馈源
 * @param {const motor_control_result_t *} result  本拍控制结果
 * @param {float} control_dt                       本拍控制周期 (s)
 * @param {uint32_t} entry_us                      本拍入口时间 (us)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_DiagControlled(motor_foc_isr_diag_t *diag, const MOTOR_PARAM_S *motor,
                                        motor_feedback_source_t feedback_source, const motor_control_result_t *result,
                                        float control_dt, uint32_t entry_us)
{
    if (s_tracking_enabled) {
        bool eligible = motor->ol_enable && motor->ol_mode == MOTOR_OL_MODE_IF &&
                        feedback_source == MOTOR_FEEDBACK_SRC_SENSORLESS && !motor->sensorless_takeover &&
                        !motor->speed_enable && motor->fw_current_max <= 0.0f &&
                        Motor_Control_IsIfDragSettled(&s_control) && result->action == MOTOR_CONTROL_ACTION_OUTPUT_DQ;
        Motor_Current_TrackingObserve(&s_tracking, eligible, motor->Id, motor->Iq, motor->id_target, motor->iq_target,
                                      control_dt, entry_us);
    }
    Motor_FocIsr_DiagMark(diag, MOTOR_TIMING_CONTROL);
}

/**
 * @description: 诊断钩子：输出提交后结束 OUTPUT 段，写无感轨迹与入口耗时。
 * @detail: 只写 RAM，不改本拍已提交的 PWM。
 * @param {const motor_foc_isr_diag_t *} diag     本拍诊断状态
 * @param {const MOTOR_PARAM_S *} motor           电机对象
 * @param {motor_feedback_source_t} feedback_source 本拍反馈源
 * @param {const motor_control_result_t *} result 本拍控制结果
 * @param {uint32_t} entry_us                     本拍入口时间 (us)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_DiagEnd(const motor_foc_isr_diag_t *diag, const MOTOR_PARAM_S *motor,
                                 motor_feedback_source_t feedback_source, const motor_control_result_t *result,
                                 uint32_t entry_us)
{
    if (diag->profile) {
        Motor_Timing_ProfileRecord(MOTOR_TIMING_OUTPUT, diag->stage_start);
        Motor_Timing_ProfileEnd();
    }
    if (diag->features & MOTOR_TIMING_ISR_TRACE) Motor_FocIsr_RecordTrace(motor, feedback_source, result->vd, result->vq);
    Motor_FocIsr_RecordExit(motor, feedback_source, entry_us, diag->features);
}

/**
 * @description: 执行一拍完整 FOC：采样 → 保护 → 电流 → 控制 → 输出。
 * @detail: Step 已判定本拍需要控制；保护不通过时已复位并中点输出，本拍不再跑电流环。
 *          各步之间的 Diag* 钩子只写 RAM 诊断。调用者保证 motor_param 非空。
 *          状态发布由 Step 在本函数之后进行。
 * @param {MOTOR_PARAM_S *} motor_param            电机对象，不可为空
 * @param {motor_feedback_source_t} feedback_source 本拍反馈源
 * @param {uint32_t} entry_us                      本拍入口时间 (us)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_Run(MOTOR_PARAM_S *motor_param, motor_feedback_source_t feedback_source, uint32_t entry_us)
{
    motor_foc_isr_diag_t diag;
    float control_dt = 0.0f;
    bool timing_ok;
    motor_control_result_t result;

#if MOTOR_CFG_DIAG
    if (g_motor_phase_active == 3U) {
        bool ready = motor_param->ol_target_rpm == 3000.0f && motor_param->ol_current_rpm >= 2970.0f &&
                     motor_param->ol_current_rpm <= 3030.0f && motor_param->sensorless_enable;
        if (ready) {
            float observed = Motor_Sensorless_GetMechanicalRpm();
            ready = observed >= 2850.0f && observed <= 3150.0f;
        }
        Motor_Timing_PhaseEdgePoll(motor_param->ol_enable, ready, entry_us);
    }
    if (g_hal_motor_pwm_window.enabled != 0U) {
        bool ready = false;
        if (g_hal_motor_pwm_window.enabled == 2U && motor_param->ol_enable) {
            float target = motor_param->ol_target_rpm;
            float error = motor_param->ol_current_rpm - target;
            ready = target * target >= 1.0f && error * error <= 0.0025f * target * target;
        }
        Hal_Motor_PwmWindowPoll(motor_param->ol_enable, ready);
    }
#endif
    Motor_FocIsr_DiagBegin(&diag, entry_us);

    /* 1. 采样 */
    timing_ok = Motor_FocIsr_Sample(motor_param, feedback_source, entry_us, &control_dt);
    Motor_FocIsr_DiagSampled(&diag, motor_param, entry_us, control_dt);

    /* 2. 保护 */
    if (!Motor_FocIsr_Protect(motor_param, feedback_source, timing_ok, control_dt, entry_us)) {
        Motor_FocIsr_DiagStopped(&diag, motor_param, feedback_source, entry_us);
        return;
    }
    Motor_FocIsr_DiagMark(&diag, MOTOR_TIMING_SAMPLE);

    /* 3. 电流 */
    Motor_FocIsr_PrepareCurrentFeedback(motor_param);
    Motor_FocIsr_DiagCurrentReady(&diag, motor_param, entry_us);

    /* 4. 控制 */
    result = Motor_FocIsr_RunControl(motor_param, control_dt);
    Motor_FocIsr_DiagControlled(&diag, motor_param, feedback_source, &result, control_dt, entry_us);

    /* 5. 输出 */
    Motor_FocIsr_ArbitrateAndApply(motor_param, result);
    Motor_FocIsr_DiagEnd(&diag, motor_param, feedback_source, &result, entry_us);
}

/**
 * @description: 按约 100 Hz 从 ADC 中断发布状态快照。
 * @detail: 默认跳过拍每150次发布；divider<=1时每300个回调发布。只写快照，不改变控制结果。
 * @param {const MOTOR_PARAM_S *} motor_param 电机采样与保护对象，不可为空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_FocIsr_PublishStatus(const MOTOR_PARAM_S *motor_param)
{
    COMMON_ASSERT(motor_param != NULL);
    s_status_publish_divider++;
    if (s_status_publish_divider >= MOTOR_STATUS_PUBLISH_DIVIDER * (s_divider.divider <= 1 ? 2U : 1U)) {
        s_status_publish_divider = 0U;
#if MOTOR_CFG_DIAG
        Motor_Timing_PhaseMarkStatus();
#endif
        if (Motor_Timing_SplitIsActive()) {
            uint32_t publish_start = Hal_Motor_TimeUs();
            Motor_StatusSnapshotPublishFromIsr(motor_param);
            Motor_Timing_CallbackPublish(Hal_Motor_TimeUs() - publish_start);
        } else {
            Motor_StatusSnapshotPublishFromIsr(motor_param);
        }
    }
}

/**
 * @description: 推进一次正常 FOC ADC 中断路径，并按约 100 Hz 发布状态快照。
 * @detail: 在 ADC-DMA 中断调用，正常路径不阻塞。每个回调检查完整控制拍的截止时间，
 *          连续拒绝选拍也会锁存时序故障并安全输出；正常采样、保护和电流环仅在完整拍执行。
 *          motor_param 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor_param 电机采样与保护对象，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_FocIsr_Step(MOTOR_PARAM_S *motor_param)
{
    motor_feedback_source_t feedback_source;
    bool execute;
    uint32_t now_us;
    float max_gap_us;
#if MOTOR_CFG_DIAG
    bool phase_active = g_motor_phase_active == 1U || g_motor_phase_active == 2U;
    uint32_t phase_us = 0U, phase_count = 0U, phase_cr1 = 0U, phase_divider = 0U;
#endif
    COMMON_ASSERT(motor_param != NULL);

    feedback_source = Motor_Feedback_GetSourceDefault();
#if MOTOR_CFG_DIAG
    if (phase_active) {
        phase_us = Hal_Motor_TimeUs();
        phase_cr1 = TIM1->CR1;
        phase_count = TIM1->CNT;
        phase_divider = (uint32_t)(uint16_t)s_divider.skip << 16;
    }
#endif
    execute = Motor_FocIsr_ShouldExecuteControl(feedback_source);
    now_us = Hal_Motor_TimeUs();
    max_gap_us = motor_param->ol_enable ? Motor_Timing_GetControlDt(motor_param) * 3000000.0f : 0.0f;
    if (Motor_FocDivider_ControlExpired(&s_control_deadline, motor_param->ol_enable, execute, now_us,
                                        max_gap_us > 0.0f && max_gap_us <= 4294967040.0f ?
                                            (uint32_t)(max_gap_us + 0.5f) : 0U)) {
        Motor_Protect_TriggerFault(FAULT_CODE_CONTROL_TIMING, motor_param);
        Motor_Control_ResetRuntime(&s_control, motor_param);
        Motor_OutputSafe(motor_param);
        execute = false;
    }
#if MOTOR_CFG_DIAG
    if (phase_active) {
        phase_divider |= (uint16_t)s_divider.skip;
        Motor_Timing_PhaseEdgeNote(phase_us, phase_count, phase_cr1, phase_divider,
                                  (execute ? 1U : 0U) | ((uint32_t)feedback_source << 8) |
                                  ((uint32_t)(uint16_t)s_divider.divider << 16), motor_param->ol_enable);
    }
#endif
    if (execute) {
        Motor_FocIsr_Run(motor_param, feedback_source, now_us);
        if (Motor_Timing_SplitIsActive()) Motor_Timing_CallbackClassify(true);
    } else if (Motor_Timing_SplitIsActive()) {
        Motor_Timing_CallbackClassify(false);
    }
    /* 默认避开完整拍发布；无分频时没有跳过拍，仍需定期更新状态。 */
    if (!execute || s_divider.divider <= 1) Motor_FocIsr_PublishStatus(motor_param);

    return;
}
