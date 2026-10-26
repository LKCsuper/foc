/* 电机状态分离快照主机测试：ISR 发布与临界区一致性 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "motor_speed.h"

#define MOTOR_CRIT_H
static unsigned critical_depth;
static bool in_isr;
static unsigned feedback_reads;
static unsigned speed_reads;
static bool speed_available = true;
static motor_speed_snapshot_t speed_frame;
static mc_fault_code fault;

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：进入短临界区（屏蔽 IRQ），返回进入前的 PRIMASK。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 critical_depth++
 */
static uint32_t Motor_Crit_Enter(void) { return critical_depth++; }
/**
 * @description: 主机测试替身：退出短临界区；仅当进入前 IRQ 开启时恢复。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} previous 上一次计划截止时刻 (tick)
 * @return {void} 无
 */
static void Motor_Crit_Exit(uint32_t previous)
{
    assert(critical_depth == previous + 1U);
    critical_depth = previous;
}
/**
 * @description: 主机测试替身：读取最近一次速度 PI 使用的同一帧目标、反馈、误差和 Iq。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_speed_snapshot_t *} out 输出缓冲或结果
 * @return {bool} 测试记录值 speed_available
 */
bool Motor_SpeedLoop_GetSnapshot(motor_speed_snapshot_t *out)
{
    assert(!in_isr && critical_depth != 0U);
    speed_reads++;
    *out = speed_frame;
    return speed_available;
}
/**
 * @description: 主机测试替身：默认单实例采样
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @param {float} theta 强制电角度 (rad)
 * @param {motor_feedback_sample_t *} out 输出缓冲或结果
 * @return {void} 无
 */
void Motor_Feedback_SampleDefault(const MOTOR_PARAM_S *motor, float theta, motor_feedback_sample_t *out)
{
    (void)motor;
    assert(in_isr || critical_depth != 0U);
    feedback_reads++;
    memset(out, 0, sizeof(*out));
    out->elec_angle_rad = theta;
    out->used_source = MOTOR_FEEDBACK_SRC_SENSORLESS;
}
/**
 * @description: 主机测试替身：查询开环对齐是否仍在进行。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_OpenLoop_IsAligning(void) { return false; }
/**
 * @description: 主机测试替身：读取当前对齐已用时间。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0.0f
 */
float Motor_OpenLoop_GetAlignElapsed(void) { return 0.0f; }
/**
 * @description: 主机测试替身：读取默认实例当前故障码。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {mc_fault_code} 测试记录值 fault
 */
mc_fault_code Motor_Protect_GetFault(void) { return fault; }
/**
 * @description: 主机测试替身：查询默认实例是否已锁存故障。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 fault != FAULT_CODE_NONE
 */
bool Motor_Protect_IsFaultLatched(void) { return fault != FAULT_CODE_NONE; }

#include "../../App/motor/motor_snapshot.c"

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    MOTOR_PARAM_S motor = {0};
    motor_status_t status;
    motor.ol_current_rpm = 72.0f;
    motor.v_bus = 24.0f;
    motor.currentAdc[0] = 2037.0f;
    motor.ol_theta = 0.7f;
    speed_frame.target_rpm = 100.0f;
    speed_frame.measured_rpm = 123.0f;
    speed_frame.error = -23.0f;
    speed_frame.iq_target = 0.45f;
    assert(Motor_GetStatus(NULL, &status) == MOTOR_ERR_NULL);
    assert(Motor_GetStatus(&motor, NULL) == MOTOR_ERR_NULL);
    assert(Motor_GetStatus(&motor, &status) == MOTOR_OK);
    assert(status.current_rpm == 72.0f && status.measured_rpm == 123.0f);
    assert(feedback_reads == 1U && speed_reads == 1U && critical_depth == 0U);

    fault = FAULT_CODE_ABS_OVER_CURRENT;
    in_isr = true;
    Motor_StatusSnapshotPublishFromIsr(&motor);
    in_isr = false;
    assert(speed_reads == 1U && feedback_reads == 2U);
    motor.v_bus = 20.0f;
    motor.currentAdc[0] = 1000.0f;
    motor.ol_current_rpm = 300.0f;
    fault = FAULT_CODE_NONE;
    speed_frame.measured_rpm = 456.0f;
    speed_frame.error = -356.0f;
    speed_frame.iq_target = 0.8f;
    assert(Motor_GetStatus(&motor, &status) == MOTOR_OK);
    assert(status.v_bus == 24.0f && status.current_adc[0] == 2037.0f && status.current_rpm == 72.0f);
    assert(status.feedback.elec_angle_rad == 0.7f && status.fault_latched);
    assert(status.fault == FAULT_CODE_ABS_OVER_CURRENT);
    assert(status.measured_rpm == 456.0f && status.speed_error == -356.0f && status.speed_iq == 0.8f);
    assert(speed_reads == 2U && feedback_reads == 2U && critical_depth == 0U);

    speed_available = false;
    assert(Motor_GetStatus(&motor, &status) == MOTOR_OK);
    assert(status.measured_rpm == 0.0f && status.speed_error == 0.0f && status.speed_iq == 0.0f);
    in_isr = true;
    Motor_StatusSnapshotPublishFromIsr(&motor);
    in_isr = false;
    assert(Motor_GetStatus(&motor, &status) == MOTOR_OK);
    assert(status.v_bus == 20.0f && status.current_rpm == 300.0f && !status.fault_latched);
    speed_available = true;
    motor.speed_enable = true;
    in_isr = true;
    Motor_StatusSnapshotPublishFromIsr(&motor);
    in_isr = false;
    assert(Motor_GetStatus(&motor, &status) == MOTOR_OK);
    assert(status.current_rpm == 100.0f && status.speed_enable);
    assert(critical_depth == 0U);
    puts("Motor status split snapshot tests passed");
    return 0;
}
