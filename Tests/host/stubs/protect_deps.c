/*
 * Host stubs for motor_protect default wrappers (sensorless/speed deps).
 * State-API tests do not call these paths; symbols exist for link completeness.
 */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include "motor_sensorless.h"
#include "motor_speed.h"
/* Private variables ---------------------------------------------------------*/
unsigned protect_diagnostic_reads;

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：拷贝默认实例观测器的接管诊断数据。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_sensorless_diagnostics_t *} diagnostics diagnostics 参数
 * @return {void} 无
 */
void Motor_Sensorless_GetDiagnostics(motor_sensorless_diagnostics_t *diagnostics)
{
    if (diagnostics != NULL) {
        protect_diagnostic_reads++;
        memset(diagnostics, 0, sizeof(*diagnostics));
        diagnostics->applied_voltage = 7.0f;
        diagnostics->blend = 0.75f;
        diagnostics->flux_ok = true;
    }
}

/**
 * @description: 主机测试替身：获取默认实例低通后的机械转速。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0.0f
 */
float Motor_Sensorless_GetMechanicalRpm(void)
{
    return 0.0f;
}

/**
 * @description: 主机测试替身：获取默认实例估算转子磁链模长。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0.0f
 */
float Motor_Sensorless_GetFluxMagnitude(void)
{
    return 0.0f;
}

/**
 * @description: 主机测试替身：获取默认实例 PLL 观测角与 I/F 强制角的最短相位差。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0.0f
 */
float Motor_Sensorless_GetPhaseError(void)
{
    return 0.0f;
}

/**
 * @description: 主机测试替身：获取默认实例观测器状态机当前状态。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {motor_sensorless_state_t} 测试记录值 MOTOR_SENSORLESS_OPEN_LOOP
 */
motor_sensorless_state_t Motor_Sensorless_GetState(void)
{
    return MOTOR_SENSORLESS_OPEN_LOOP;
}

/**
 * @description: 主机测试替身：读取默认实例当前 Iq 目标
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0.0f
 */
float Motor_SpeedLoop_GetIqTarget(void)
{
    return 0.0f;
}

/**
 * @description: 主机测试替身：查询默认实例速度反馈是否可信
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
bool Motor_SpeedLoop_IsFeedbackValid(void)
{
    return false;
}
