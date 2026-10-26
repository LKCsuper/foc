/*
 * Host stubs for motor_sensorless default wrappers (speed/protect side effects).
 */
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "motor_param.h"
#include "motor_types.h"
#include "motor_timing.h"

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：返回本拍 Begin 写下的采样标志。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Timing_ProfileIsActive(void) { return false; }
bool host_observer_step_active;
motor_observer_step_sample_t host_observer_step;
unsigned host_observer_step_count;
/**
 * @description: 主机测试替身：查询观测步进记录是否仍在采集。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 host_observer_step_active
 */
bool Motor_Timing_ObserverStepActive(void) { return host_observer_step_active; }
/**
 * @description: 主机测试替身：判断本拍是否写入观测步进样本。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 host_observer_step_active
 */
bool Motor_Timing_ObserverStepDue(void) { return host_observer_step_active; }
/**
 * @description: 主机测试替身：写入一条观测步进样本。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const motor_observer_step_sample_t *} sample 本拍样本
 * @return {void} 无
 */
void Motor_Timing_ObserverStepPush(const motor_observer_step_sample_t *sample)
{
    host_observer_step = *sample;
    host_observer_step_count++;
}
/**
 * @description: 主机测试替身：查询相位记录是否为当前格式。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Timing_PhaseMode(void) { return false; }
/**
 * @description: 主机测试替身：判断本拍是否写入相位样本。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {bool} completed 兼容参数，本实现不使用
 * @return {bool} 本实现固定返回 false
 */
bool Motor_Timing_PhaseDue(bool completed) { (void)completed; return false; }
/**
 * @description: 主机测试替身：写入一条相位样本。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const motor_phase_sample_t *} sample 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Timing_PhasePush(const motor_phase_sample_t *sample) { (void)sample; }
/**
 * @description: 主机测试替身：读取分段计时用的当前时间戳。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 本实现固定返回 0U
 */
uint32_t Motor_Timing_ProfileTimeUs(void) { return 0U; }
/**
 * @description: 主机测试替身：把一个阶段从 start_us 到现在的耗时累进当前窗口。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_timing_stage_t} stage 兼容参数，本实现不使用
 * @param {uint32_t} start 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Timing_ProfileRecord(motor_timing_stage_t stage, uint32_t start)
{
    (void)stage;
    (void)start;
}

float host_speed_feedback_rpm;
bool host_speed_feedback_valid;
unsigned host_speed_feedback_calls;
mc_fault_code host_protect_fault;
unsigned host_protect_fault_calls;

/**
 * @description: 主机测试替身：更新默认实例的真实机械转速反馈
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} rpm 机械转速 (RPM)
 * @param {bool} valid valid 参数
 * @return {void} 无
 */
void Motor_SpeedFeedback_Set(float rpm, bool valid)
{
    host_speed_feedback_rpm = rpm;
    host_speed_feedback_valid = valid;
    host_speed_feedback_calls++;
}

/**
 * @description: 主机测试替身：用当前等效 Iq 预置默认实例速度 PI
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} iq 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_SpeedLoop_PrimeIq(float iq)
{
    (void)iq;
}

/**
 * @description: 主机测试替身：默认实例外部触发故障锁存。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {mc_fault_code} fault 故障条件或故障编码
 * @param {const MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Protect_TriggerFault(mc_fault_code fault, const MOTOR_PARAM_S *motor)
{
    (void)motor;
    host_protect_fault = fault;
    host_protect_fault_calls++;
}
