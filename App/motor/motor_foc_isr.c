/*
 * @Description: 正常 FOC 中断路径：采样、保护、控制与输出
 * @Author: lkc
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_foc_isr.h"

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "hal_motor.h"
#include "motor.h"
#include "motor_control.h"
#include "motor_current.h"
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
/* Private define ------------------------------------------------------------*/
#define MOTOR_STATUS_PUBLISH_DIVIDER 300U /* 30 kHz ADC callback -> 100 Hz status */
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
static motor_foc_divider_t s_divider = {
    .divider = FOC_CONTROL_LOOP_FREQ_DIVIDER,
};
static float s_recon_diag_target_rpm;
static bool s_recon_diag_enable;
static uint8_t s_status_publish_divider;
/* Private function prototypes -----------------------------------------------*/
static bool Motor_FocIsr_ShouldExecuteControl(motor_feedback_source_t feedback_source);
static bool Motor_FocIsr_SampleAndProtect(MOTOR_PARAM_S *motor, motor_feedback_source_t feedback_source,
                                          float *control_dt);
static void Motor_FocIsr_PrepareCurrentFeedback(MOTOR_PARAM_S *motor);
static motor_control_result_t Motor_FocIsr_RunControl(MOTOR_PARAM_S *motor, float control_dt);
static void Motor_FocIsr_ArbitrateAndApply(MOTOR_PARAM_S *motor, motor_control_result_t result);
static void Motor_FocIsr_Run(MOTOR_PARAM_S *motor_param);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 判断本次 ADC 回调是否承担一拍完整 FOC。
 * @detail: 有感路径固定使用 PWM 向下计数侧；其他路径保持原 FOC 分频语义。
 * @param {motor_feedback_source_t} feedback_source 当前反馈源
 * @return {bool} true=继续执行采样、保护和控制
 */
static bool Motor_FocIsr_ShouldExecuteControl(motor_feedback_source_t feedback_source)
{
    if (feedback_source == MOTOR_FEEDBACK_SRC_HALL || feedback_source == MOTOR_FEEDBACK_SRC_ENCODER) {
        return (TIM1->CR1 & TIM_CR1_DIR) != 0U;
    }
    return Motor_FocIsr_ShouldRun();
}

/**
 * @description: 捕获本拍样本、更新传感器并执行快速保护。
 * @detail: 保护使用原始换算电流；高占空比电流重构必须在保护之后单独执行。
 * @param {MOTOR_PARAM_S *} motor 电机运行时
 * @param {motor_feedback_source_t} feedback_source 当前反馈源
 * @param {float *} control_dt 输出本拍控制周期 (s)
 * @return {bool} true=允许继续控制；false=已应用安全输出
 */
static bool Motor_FocIsr_SampleAndProtect(MOTOR_PARAM_S *motor, motor_feedback_source_t feedback_source,
                                          float *control_dt)
{
    Motor_Timing_RecordSampleDirection((TIM1->CR1 & TIM_CR1_DIR) == 0U);
    Motor_Hall_DiagNoteTimer((uint16_t)TIM1->CNT, (uint16_t)TIM1->ARR);
    Motor_Sense_Update(motor);
    *control_dt = Motor_Timing_GetControlDt(motor);
    Motor_SensorUpdate_Step(motor, feedback_source, *control_dt);

    if (!Motor_Protect_Run(motor, *control_dt)) {
        return true;
    }

    Motor_Control_ResetRuntime(&s_control, motor);
    Motor_OutputSafe(motor);
    return false;
}

/**
 * @description: 在 Clarke/Park 前准备可信的三相电流。
 * @detail: 保留 VESC LONGEST_ZERO 语义，只在唯一最大占空相采样窗不足时重构该相。
 * @param {MOTOR_PARAM_S *} motor 电机运行时
 * @return {void} 无
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
 * @param {MOTOR_PARAM_S *} motor 电机运行时
 * @param {float} control_dt 本拍控制周期 (s)
 * @return {motor_control_result_t} SAFE 或 DQ 电压请求
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
 * @description: 执行最终故障仲裁，并把 DQ 请求经反 Park/SVPWM 提交到硬件。
 * @param {MOTOR_PARAM_S *} motor 电机运行时
 * @param {motor_control_result_t} result 控制核心给出的候选输出
 * @return {void} 无
 */
static void Motor_FocIsr_ArbitrateAndApply(MOTOR_PARAM_S *motor, motor_control_result_t result)
{
    bool fault_latched = Motor_Protect_IsFaultLatched();

    if (fault_latched) {
        Motor_Control_ResetRuntime(&s_control, motor);
    }
    result = Motor_Control_ArbitrateOutput(fault_latched, &result);
    Motor_OutputApplyControlResult(motor, motor->v_bus, &result);
}

/**
 * @description: 查询开环对齐是否仍在进行。
 * @return {bool} 对齐阶段未结束返回 true
 */
bool Motor_OpenLoop_IsAligning(void)
{
    return Motor_Control_IsAligning(&s_control);
}

/**
 * @description: 读取当前对齐已用时间。
 * @return {float} 对齐已用时间 (s)
 */
float Motor_OpenLoop_GetAlignElapsed(void)
{
    return Motor_Control_GetAlignElapsed(&s_control);
}

/**
 * @description: 查询 HFI 启动是否仍在运行。
 * @return {bool} HFI 启动进行中返回 true
 */
bool Motor_HfiStart_IsRunning(void)
{
    return Motor_Control_HfiIsRunning(&s_control);
}

/**
 * @description: 查询最近一次 HFI 启动是否成功。
 * @return {bool} 成功返回 true
 */
bool Motor_HfiStart_Succeeded(void)
{
    return Motor_Control_HfiSucceeded(&s_control);
}

/**
 * @description: 读取 HFI 启动给出的电角度。
 * @return {float} 电角度 (rad)
 */
float Motor_HfiStart_GetAngle(void)
{
    return Motor_Control_HfiGetAngle(&s_control);
}

/**
 * @description: 读取 HFI 启动置信度。
 * @return {float} 置信度，无量纲
 */
float Motor_HfiStart_GetConfidence(void)
{
    return Motor_Control_HfiGetConfidence(&s_control);
}

/**
 * @description: 查询本拍是否到达 FOC 控制分频点。
 * @return {bool} 本拍应执行控制返回 true
 */
bool Motor_FocIsr_ShouldRun(void)
{
    return Motor_FocDivider_ShouldRun(&s_divider);
}

/**
 * @description: 执行一次正常 FOC 控制拍：采样、保护、控制并提交输出。
 * @detail: 有感反馈在 PWM 向上计数侧直接返回；其余路径按分频器决定是否执行。
 *          仅由 Motor_FocIsr_Step 在 ADC 中断调用。
 * @param {MOTOR_PARAM_S *} motor_param 电机采样与保护对象，调用前已确认非空
 * @return {void} 无
 */
static void Motor_FocIsr_Run(MOTOR_PARAM_S *motor_param)
{
    uint32_t runtime_start = Hal_Motor_TimeUs();
    float control_dt;
    motor_feedback_source_t feedback_source;
    motor_control_result_t result;

    feedback_source = Motor_Feedback_GetSourceDefault();
    if (!Motor_FocIsr_ShouldExecuteControl(feedback_source)) {
        Motor_Timing_RecordRuntime(runtime_start);
        return;
    }

    if (!Motor_FocIsr_SampleAndProtect(motor_param, feedback_source, &control_dt)) {
        Motor_Timing_RecordRuntime(runtime_start);
        return;
    }

    Motor_FocIsr_PrepareCurrentFeedback(motor_param);
    result = Motor_FocIsr_RunControl(motor_param, control_dt);
    Motor_FocIsr_ArbitrateAndApply(motor_param, result);
    Motor_Timing_RecordRuntime(runtime_start);
}

/**
 * @description: 推进一次正常 FOC ADC 中断路径，并按约 100 Hz 发布状态快照。
 * @param {MOTOR_PARAM_S *} motor_param 电机采样与保护对象，不可为空
 * @return {void} 无
 */
void Motor_FocIsr_Step(MOTOR_PARAM_S *motor_param)
{
    if (motor_param == NULL) {
        return;
    }

    Motor_FocIsr_Run(motor_param);
    s_status_publish_divider++;
    if (s_status_publish_divider >= MOTOR_STATUS_PUBLISH_DIVIDER) {
        s_status_publish_divider = 0U;
        Motor_StatusSnapshotPublishFromIsr(motor_param);
    }
}
