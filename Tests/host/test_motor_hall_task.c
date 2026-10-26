/* 霍尔标定兼容任务包装主机测试：完成、停机、故障、超时、取消与旧表保留。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor.h"
#include "motor_hall.h"
#include "motor_protect.h"
#include "motor_hall_angle.h"

MOTOR_PARAM_S motor;
static uint32_t time_us;              /* 单调递增虚拟时间戳 (us) */
static unsigned polls, starts, stops, stop_requests;
static bool fault;                    /* 注入的保护故障 */
static int mode;                      /* 0=完成，1=取消，2=故障，3=调度迟到超时，4=缺扇区 */
static bool pwm_on;                   /* PWM 是否已启动 */

/**
 * @description: ---- 依赖 stub：隔离保护/电机 API/HAL/RTOS，仅记录调用与注入故障 ----
 * @return {bool} 测试记录值 fault
 */
bool Motor_Protect_IsFaultLatched(void) { return fault; }
/**
 * @description: 主机测试替身：读取默认实例当前故障码。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {mc_fault_code} 测试记录值 fault ? FAULT_CODE_ABS_OVER_CURRENT : FAULT_CODE_NONE
 */
mc_fault_code Motor_Protect_GetFault(void) { return fault ? FAULT_CODE_ABS_OVER_CURRENT : FAULT_CODE_NONE; }
/**
 * @description: 主机测试替身：故障码转可读短名，供 Shell 与日志使用。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {mc_fault_code} code 兼容参数，本实现不使用
 * @return {const char *} 测试记录值 "test"
 */
const char *Motor_Protect_GetFaultName(mc_fault_code code) { (void)code; return "test"; }
/**
 * @description: 主机测试替身：清除默认实例故障锁存、计时和首发采样证据。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Protect_ClearFault(void) { assert(!fault); }
/**
 * @description: 主机测试替身：清除控制用运行时状态（斜坡转速与 d/q 积分），不改命令与配置。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} m 兼容参数，本实现不使用
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_ClearRuntimeControl(MOTOR_PARAM_S *m) { (void)m; return MOTOR_OK; }
/**
 * @description: 停机请求 stub：验证释放标定权前已原子发布禁用，并记录请求次数。
 * @param {MOTOR_PARAM_S *} m 电机实例
 * @return {motor_result_t} 测试记录值 MOTOR_OK
 */
motor_result_t Motor_RequestStop(MOTOR_PARAM_S *m)
{
    ++stop_requests;
    assert(!m->ol_enable); /* 释放标定权前已经原子地发布禁用 */
    m->ol_enable = false;
    assert(!Motor_Hall_DetectIsActive());
    return MOTOR_OK;
}
/**
 * @description: 主机测试替身：启动三相 PWM 输出（模式已配置，不改占空比）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Hal_Motor_PwmStart(void) { ++starts; pwm_on = true; }
/**
 * @description: 主机测试替身：停止三相 PWM 输出（关主通道/互补通道，具体由板级 BSP）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Hal_Motor_PwmStop(void) { ++stops; pwm_on = false; }
/**
 * @description: 主机测试替身：读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 time_us
 */
uint32_t Hal_Motor_TimeUs(void) { return time_us; }
/**
 * @description: 主机测试替身：模拟任务延时，按本测试场景推进时间或注入事件。
 * @param {uint32_t} ticks ticks 参数
 * @return {int32_t} 本实现固定返回 0
 */
int32_t osDelay(uint32_t ticks)
{
    assert(pwm_on && ticks == 1U);
    time_us += mode == 3 ? 1000000U : ticks * 1000U;
    ++polls;
    if (polls == 1500U) {
        if (mode == 1) Motor_Hall_DetectAbort();
        if (mode == 2) fault = true;
    }
    return 0;
}
/**
 * @description: 霍尔读数 stub：由当前强制电角度反推扇区号；mode==4 返回缺扇区。
 * @param {bool} second 兼容参数，本实现不使用
 * @param {int} samples 兼容参数，本实现不使用
 * @return {int} 霍尔状态 0~7（bit0=A, bit1=B, bit2=C）
 */
int Hal_Hall_ReadRaw(bool second, int samples)
{
    float theta, id, iq;
    (void)second; (void)samples;
    if (mode == 4) return 1;
    if (!Motor_Hall_DetectGetTargets(&motor, &theta, &id, &iq)) return 1;
    return ((int)lroundf(theta * 180.0f / 3.14159265358979323846f) % 360) / 60 + 1;
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    uint8_t table[8];
    int8_t old_table[8];
    bool result;
    for (mode = 0; mode <= 4; ++mode) {
        memset(&motor, 0, sizeof(motor));
        motor.hall_table[0] = motor.hall_table[7] = -1;
        for (unsigned i = 1; i <= 6; ++i) motor.hall_table[i] = (int8_t)((i - 1U) * 33U);
        assert(Motor_Hall_TableReady(motor.hall_table));
        memcpy(old_table, motor.hall_table, sizeof(old_table));
        time_us = UINT32_MAX - 12345U;
        polls = starts = stops = stop_requests = 0;
        pwm_on = fault = false;
        result = true;
        int code = Motor_Hall_Detect(0.6f, table, &result);
        assert(starts == 1 && stops == 1 && stop_requests == 1);
        assert(!pwm_on && !motor.ol_enable && !Motor_Hall_DetectIsActive());
        if (mode == 0) {
            assert(result && Motor_Hall_TableReady(motor.hall_table));
            assert(polls == 37500U); /* 与旧流程等待总时长一致 */
        } else {
            assert(!result && memcmp(motor.hall_table, old_table, sizeof(old_table)) == 0);
        }
        assert(code == (mode == 2 ? FAULT_CODE_ABS_OVER_CURRENT : FAULT_CODE_NONE));
    }
    fault = true;
    starts = 0;
    result = true;
    assert(Motor_Hall_Detect(0.6f, table, &result) == FAULT_CODE_ABS_OVER_CURRENT);
    assert(!result && starts == 0);
    fault = false;
    result = true;
    Motor_Hall_Detect(NAN, table, &result);
    assert(!result && starts == 0);
    puts("Hall task wrapper: completion, stop, fault, timeout and old table preservation passed");
    return 0;
}
