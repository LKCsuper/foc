/* Cmd_Autotune 参数校验与拒绝：非法输入不改写电机配置、无副作用。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "motor_detect.h"
#include "hal_motor.h"

MOTOR_PARAM_S motor;
static motor_detect_result_t detected;
static bool active, running, faulted;
static float sample_hz = 30000.0f;
int Cmd_Autotune(int argc, char *argv[]);

/**
 * @description: 主机测试替身：从电机对象捕获当前命令字段。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} m 兼容参数，本实现不使用
 * @param {motor_command_t *} cmd 待提交命令
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_GetCommand(const MOTOR_PARAM_S *m, motor_command_t *cmd)
{
    (void)m; memset(cmd, 0, sizeof(*cmd)); cmd->enable = running; return MOTOR_OK;
}
/**
 * @description: 主机测试替身：获取一致的只读运行快照。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} m 兼容参数，本实现不使用
 * @param {motor_status_t *} status status 参数
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_GetStatus(const MOTOR_PARAM_S *m, motor_status_t *status)
{
    (void)m; memset(status, 0, sizeof(*status)); status->fault_latched = faulted; return MOTOR_OK;
}
/**
 * @description: 主机测试替身：偏移校准时 FOC 不得改占空比编排（ISR 仍采样+软件保护）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 active
 */
bool Motor_Detect_IsActive(void) { return active; }
/**
 * @description: 主机测试替身：在短临界区内拷贝最近一次检测结果。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_detect_result_t *} result 输出结果
 * @return {void} 无
 */
void DETECT_GetResult(motor_detect_result_t *result) { *result = detected; }
/**
 * @description: 主机测试替身：估算 ADC/控制采样触发频率
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 测试记录值 sample_hz
 */
float Hal_Motor_AdcSampleRateHz(void) { return sample_hz; }

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    MOTOR_PARAM_S before;
    char *args[] = {"pitune", "150", "100", "60"};
    memset(&motor, 0, sizeof(motor));
    motor.ol_current_kp = 0.12f; motor.ol_current_ki = 18.0f;
    before = motor;
    assert(Cmd_Autotune(3, args) == -1);
    detected.version = MOTOR_DETECT_RESULT_VERSION;
    detected.resistance_valid = detected.inductance_valid = true;
    detected.resistance_ohm = 0.5f;
    detected.inductance_d_h = 0.001f; detected.inductance_q_h = 0.002f;
    assert(Cmd_Autotune(3, args) == 0);
    assert(Cmd_Autotune(4, args) == 0);
    active = true; assert(Cmd_Autotune(3, args) == -1); active = false;
    running = true; assert(Cmd_Autotune(3, args) == -1); running = false;
    faulted = true; assert(Cmd_Autotune(3, args) == -1); faulted = false;
    sample_hz = NAN; assert(Cmd_Autotune(3, args) == -1); sample_hz = 30000.0f;
    detected.error = MOTOR_DETECT_ERROR_USER_ABORT;
    assert(Cmd_Autotune(3, args) == -1); detected.error = MOTOR_DETECT_ERROR_NONE;
    args[1] = "150junk"; assert(Cmd_Autotune(3, args) == -1);
    args[1] = "nan"; assert(Cmd_Autotune(3, args) == -1);
    args[1] = "1000"; assert(Cmd_Autotune(3, args) == -1);
    args[1] = "150"; args[2] = "1000"; assert(Cmd_Autotune(3, args) == -1);
    args[2] = "1"; assert(Cmd_Autotune(3, args) == -1);
    assert(Cmd_Autotune(0, NULL) == -1);
    assert(memcmp(&motor, &before, sizeof(motor)) == 0);
    puts("autotune command: validation, rejection and read-only behavior passed");
    return 0;
}
