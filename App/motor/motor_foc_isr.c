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
static void Motor_FocIsr_Run(MOTOR_PARAM_S *motor_param);
/* Private functions ---------------------------------------------------------*/

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
    float protection_dt;
    motor_feedback_source_t feedback_source;
    motor_control_result_t result;

    feedback_source = Motor_Feedback_GetSourceDefault();
    if (feedback_source == MOTOR_FEEDBACK_SRC_HALL || feedback_source == MOTOR_FEEDBACK_SRC_ENCODER) {
        if ((TIM1->CR1 & TIM_CR1_DIR) == 0U) {
            Motor_Timing_RecordRuntime(runtime_start);
            return;
        }
    } else if (!Motor_FocIsr_ShouldRun()) {
        Motor_Timing_RecordRuntime(runtime_start);
        return;
    }

    Motor_Timing_RecordSampleDirection((TIM1->CR1 & TIM_CR1_DIR) == 0U);
    Motor_Hall_DiagNoteTimer((uint16_t)TIM1->CNT, (uint16_t)TIM1->ARR);
    Motor_Sense_Update(motor_param);
    protection_dt = Motor_Timing_GetControlDt(motor_param);
    Motor_SensorUpdate_Step(motor_param, feedback_source, protection_dt);

    if (Motor_Protect_Run(motor_param, protection_dt)) {
        Motor_Control_ResetRuntime(&s_control, motor_param);
        Motor_OutputSafe(motor_param);
        Motor_Timing_RecordRuntime(runtime_start);
        return;
    }

    if (motor_param->ol_enable &&
        (!s_recon_diag_enable || s_recon_diag_target_rpm != motor_param->ol_target_rpm)) {
        Motor_Current_ResetReconstructDiag();
        s_recon_diag_target_rpm = motor_param->ol_target_rpm;
    }
    s_recon_diag_enable = motor_param->ol_enable;
    (void)Motor_Current_ReconstructLongestZero(
        &motor_param->ia, &motor_param->ib, &motor_param->ic, motor_param->duty1,
        motor_param->duty2, motor_param->duty3, Hal_Motor_PwmPeriodCounts());

    result = Motor_Control_Step(&s_control, motor_param, motor_param->v_bus, protection_dt,
                                Motor_Hall_DetectIsActive() || Motor_Encoder_DetectIsActive() ||
                                    Motor_Anticogging_IsLearning());
    Motor_Hall_DiagUpdate();
    if (Motor_Protect_IsFaultLatched()) {
        Motor_Control_ResetRuntime(&s_control, motor_param);
        result = Motor_Control_ArbitrateOutput(true, &result);
    } else {
        result = Motor_Control_ArbitrateOutput(false, &result);
    }
    Motor_OutputApplyControlResult(motor_param, motor_param->v_bus, &result);
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
