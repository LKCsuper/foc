/* FOC ISR 分频门控与状态发布：跳过拍、相位沿、时间回绕故障与发布分频。 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "motor_foc_divider.h"

#define MOTOR_CFG_DIAG 1
typedef struct { bool ol_enable; } MOTOR_PARAM_S;
volatile uint32_t g_motor_phase_active;
static struct { int skip; int divider; } s_divider;
static motor_foc_phase_gate_t s_control_deadline;
static int s_control;
static uint32_t s_now = 1234U;
static unsigned s_faults, s_resets, s_safe, s_period_reads;
static float s_nominal_dt = 1.0f / 15000.0f;
#define FAULT_CODE_CONTROL_TIMING 1
/**
 * @description: 主机测试替身：计算 FOC 控制环周期（已乘分频）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @return {float} 测试记录值 1.0f / 15000.0f
 */
static float Motor_Timing_GetControlDt(MOTOR_PARAM_S *motor) { (void)motor; s_period_reads++; return s_nominal_dt; }
/**
 * @description: 主机测试替身：默认实例外部触发故障锁存。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {int} fault 故障条件或故障编码
 * @param {MOTOR_PARAM_S *} motor 电机实例
 * @return {void} 无
 */
static void Motor_Protect_TriggerFault(int fault, MOTOR_PARAM_S *motor)
{ assert(fault == FAULT_CODE_CONTROL_TIMING && motor != NULL); s_faults++; }
/**
 * @description: 主机测试替身：复位启动链与速度/无感/电流积分显示（不清用户目标命令）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {int *} control control 参数
 * @param {MOTOR_PARAM_S *} motor 电机实例
 * @return {void} 无
 */
static void Motor_Control_ResetRuntime(int *control, MOTOR_PARAM_S *motor)
{ assert(control == &s_control && motor != NULL); s_resets++; }
/**
 * @description: 主机测试替身：输出三相中点占空比并提交 PWM。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 电机实例
 * @return {void} 无
 */
static void Motor_OutputSafe(MOTOR_PARAM_S *motor) { assert(motor != NULL); s_safe++; }
static struct { uint32_t CR1; uint32_t CNT; } phase_timer;
#define TIM1 (&phase_timer)
static unsigned phase_notes;
static uint32_t phase_flags;
/**
 * @description: 主机测试替身：写入一条相位沿事件，并检测 PWM 计数方向切换以触发后采集。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} time time 参数
 * @param {uint32_t} count 计数值
 * @param {uint32_t} cr1 cr1 参数
 * @param {uint32_t} divider divider 参数
 * @param {uint32_t} flags flags 参数
 * @param {bool} running 当前是否运行
 * @return {void} 无
 */
static void Motor_Timing_PhaseEdgeNote(uint32_t time, uint32_t count, uint32_t cr1,
                                       uint32_t divider, uint32_t flags, bool running)
{
    assert(time == 1234U && count == 200U && cr1 == 16U);
    assert(divider == 0U && running);
    phase_notes++;
    phase_flags = flags;
}
typedef unsigned motor_feedback_source_t;
static bool s_execute;
static unsigned s_runs, s_publishes, s_gates;
volatile uint8_t g_motor_timing_split_active;

/**
 * @description: 主机测试替身：查询拍型细分诊断是否开启。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 g_motor_timing_split_active != 0U
 */
static bool Motor_Timing_SplitIsActive(void)
{
    return g_motor_timing_split_active != 0U;
}
static unsigned s_class_full, s_class_skip, s_publish_notes;
/**
 * @description: 主机测试替身：标记本回调是完整控制拍还是跳过拍。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {bool} full full 参数
 * @return {void} 无
 */
void Motor_Timing_CallbackClassify(bool full)
{
    if (full) s_class_full++;
    else s_class_skip++;
}
/**
 * @description: 主机测试替身：记下本回调内一次状态发布的自身耗时。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} elapsed_us 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Timing_CallbackPublish(uint32_t elapsed_us)
{
    (void)elapsed_us;
    s_publish_notes++;
}
static motor_feedback_source_t s_source;
/**
 * @description: 主机测试替身：读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 s_now
 */
static uint32_t Hal_Motor_TimeUs(void) { return s_now; }
/**
 * @description: 主机测试替身：读取默认单实例请求的反馈来源
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {motor_feedback_source_t} 测试记录值 s_source
 */
static motor_feedback_source_t Motor_Feedback_GetSourceDefault(void) { return s_source; }
/**
 * @description: 主机测试替身：判断本次 ADC 回调是否承担一拍完整 FOC。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_feedback_source_t} source source 参数
 * @return {bool} 测试记录值 s_execute
 */
static bool Motor_FocIsr_ShouldExecuteControl(motor_feedback_source_t source)
{
    assert(source == s_source);
    s_gates++;
    return s_execute;
}
/**
 * @description: 主机测试替身：执行一拍完整 FOC：采样 → 保护 → 电流 → 控制 → 输出。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} motor 电机实例
 * @param {motor_feedback_source_t} source source 参数
 * @param {uint32_t} start start 参数
 * @param {float} nominal_dt 本次回调读取的名义控制周期 (s)
 * @return {void} 无
 */
static void Motor_FocIsr_Run(MOTOR_PARAM_S *motor, motor_feedback_source_t source, uint32_t start, float nominal_dt)
{
    assert(motor != NULL && source == s_source && start == s_now);
    assert(nominal_dt == s_nominal_dt);
    s_runs++;
}
/**
 * @description: 主机测试替身：按约 100 Hz 从 ADC 中断发布状态快照。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} motor 电机实例
 * @return {void} 无
 */
static void Motor_FocIsr_PublishStatus(const MOTOR_PARAM_S *motor) { assert(motor != NULL); s_publishes++; }
#include "foc_dispatch_under_test.inc"

static unsigned s_status_frames;
static unsigned s_status_marks;
/**
 * @description: 主机测试替身：给当前关联事件标记状态位。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void Motor_Timing_PhaseMarkStatus(void) { s_status_marks++; }
/**
 * @description: 主机测试替身：在 ISR 捕获一拍运行快照并发布到 seqlock 双缓冲。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} motor 电机实例
 * @return {void} 无
 */
static void Motor_StatusSnapshotPublishFromIsr(const MOTOR_PARAM_S *motor)
{
    assert(motor != NULL);
    s_status_frames++;
}
#define Motor_FocIsr_PublishStatus Motor_FocIsr_PublishStatusUnderTest
#include "status_publish_under_test.inc"
#undef Motor_FocIsr_PublishStatus

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    MOTOR_PARAM_S motor = {0};
    s_divider.divider = 2;
    unsigned source;
    for (source = 0U; source < 4U; source++) {
        s_source = source;
        s_execute = false;
        Motor_FocIsr_Step(&motor);
        assert(s_runs == source);
        s_execute = true;
        Motor_FocIsr_Step(&motor);
        assert(s_runs == source + 1U);
        /* 状态发布只在跳过拍调用 */
        assert(s_gates == 2U * (source + 1U) && s_publishes == source + 1U);
    }
    for (source = 1U; source <= 450U; source++) {
        Motor_FocIsr_PublishStatusUnderTest(&motor);
        assert(s_status_frames == source / 150U && s_status_marks == s_status_frames);
    }
    g_motor_timing_split_active = 1U;
    s_execute = false;
    Motor_FocIsr_Step(&motor);
    s_execute = true;
    Motor_FocIsr_Step(&motor);
    assert(s_class_skip == 1U && s_class_full == 1U);
    for (source = 0U; source < 150U; source++) Motor_FocIsr_PublishStatusUnderTest(&motor);
    assert(s_status_frames == 4U && s_publish_notes == 1U);
    assert(s_status_marks == 4U);
    g_motor_phase_active = 1U;
    phase_timer.CR1 = 16U;
    phase_timer.CNT = 200U;
    s_divider.divider = 2;
    motor.ol_enable = true;
    s_execute = false;
    Motor_FocIsr_Step(&motor);
    assert(phase_notes == 1U && phase_flags == ((s_source << 8) | (2U << 16)));
    s_execute = true;
    Motor_FocIsr_Step(&motor);
    assert(phase_notes == 2U && phase_flags == ((s_source << 8) | (2U << 16) | 1U));
    g_motor_phase_active = 0U;
    {
        unsigned runs = s_runs;
        s_execute = false;
        s_now += 201U;
        Motor_FocIsr_Step(&motor);
        assert(s_faults == 1U && s_resets == 1U && s_safe == 1U && s_runs == runs);
        s_execute = true;
        Motor_FocIsr_Step(&motor);
        assert(s_faults == 2U && s_runs == runs);
        motor.ol_enable = false;
        Motor_FocIsr_Step(&motor);
        motor.ol_enable = true;
        Motor_FocIsr_Step(&motor);
        assert(s_faults == 2U && s_runs == runs + 2U);
    }
    {
        unsigned publishes = s_publishes;
        s_divider.divider = 1;
        Motor_FocIsr_Step(&motor);
        assert(s_publishes == publishes + 1U);
        s_status_publish_divider = 0U;
        publishes = s_status_frames;
        for (source = 0U; source < 299U; ++source) Motor_FocIsr_PublishStatusUnderTest(&motor);
        assert(s_status_frames == publishes);
        Motor_FocIsr_PublishStatusUnderTest(&motor);
        assert(s_status_frames == publishes + 1U);
    }
    /* 停机跳过拍不查频率；完整拍与运行拍只查一次，下一回调立即采用新周期。 */
    motor.ol_enable = false;
    s_execute = false;
    s_period_reads = 0U;
    Motor_FocIsr_Step(&motor);
    assert(s_period_reads == 0U);
    s_execute = true;
    Motor_FocIsr_Step(&motor);
    assert(s_period_reads == 1U);
    motor.ol_enable = true;
    s_execute = false;
    Motor_FocIsr_Step(&motor);
    assert(s_period_reads == 2U);
    s_execute = true;
    s_nominal_dt = 1.0f / 20000.0f;
    Motor_FocIsr_Step(&motor);
    assert(s_period_reads == 3U);
    puts("test_foc_dispatch: pass");
    return 0;
}
