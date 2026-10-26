/* RPM 简报输出主机测试：命令格式与字段一致性 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
typedef unsigned char UCHAR;
typedef unsigned long ULONG;
typedef unsigned mc_fault_code;
typedef struct { unsigned state; float rpm; } motor_sensorless_light_snapshot_t;
static struct { float ol_target_rpm, Iq; bool sensorless_takeover; } motor;
static motor_sensorless_light_snapshot_t light;
static char output[192];
static unsigned crit, writes, flushes;
static mc_fault_code fault;
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：进入短临界区（屏蔽 IRQ），返回进入前的 PRIMASK。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 本实现固定返回 0U
 */
static uint32_t Motor_Crit_Enter(void) { assert(!crit); crit = 1U; return 0U; }
/**
 * @description: 主机测试替身：退出短临界区；仅当进入前 IRQ 开启时恢复。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} key 进入临界区时保存的中断状态
 * @return {void} 无
 */
static void Motor_Crit_Exit(uint32_t key) { assert(key == 0U && crit); crit = 0U; }
/**
 * @description: 主机测试替身：拷贝默认实例已计算字段的轻量快照。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_sensorless_light_snapshot_t *} out 输出缓冲或结果
 * @return {void} 无
 */
static void Motor_Sensorless_GetLightSnapshot(motor_sensorless_light_snapshot_t *out) { assert(crit); *out = light; }
/**
 * @description: 主机测试替身：读取默认实例实测机械转速
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {float} 本实现固定返回 0.0f
 */
static float Motor_SpeedLoop_GetMeasuredRpm(void) { assert(crit); return 0.0f; }
/**
 * @description: 主机测试替身：查询默认实例速度反馈是否可信
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
static bool Motor_SpeedLoop_IsFeedbackValid(void) { assert(crit); return false; }
/**
 * @description: 主机测试替身：读取默认实例当前故障码。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {mc_fault_code} 测试记录值 fault
 */
static mc_fault_code Motor_Protect_GetFault(void) { assert(crit); return fault; }
/**
 * @description: 提供主机测试使用的无感状态名称。
 * @param {unsigned} state 实例状态
 * @return {const char *} 测试记录值 "OPEN_LOOP"
 */
static const char *ol_sensorless_state_name(unsigned state) { assert(!crit && state == 0U); return "OPEN_LOOP"; }
/**
 * @description: 主机测试替身：串口输出一段数据（DMA 未就绪时退回轮询）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const UCHAR *} data 数据缓冲
 * @param {ULONG} size 数据长度或元素数量，以接口编码约定为准
 * @return {ULONG} 测试记录值 size
 */
static ULONG Bsp_Usart_Write(const UCHAR *data, ULONG size)
{
    assert(!crit && size < sizeof(output));
    memcpy(output, data, size);
    output[size] = '\0';
    writes++;
    return size;
}
/**
 * @description: 主机测试替身：等待发送队列清空并等待最后一个字符发出
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void Bsp_Usart_Flush(void) { assert(!crit); flushes++; }
#include "rpm_brief_under_test.inc"
/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    motor.ol_target_rpm = 300.0f;
    motor.Iq = 1.025f;
    light.rpm = 299.5f;
    assert(ol_print_rpm_brief() == 0);
    assert(strstr(output, "rpm obs=299.5 meas=0.0 tgt=300 iq=1.025 OPEN_LOOP tk=0 fb=invalid fault=0\r\n"));
    assert(strchr(output, '\n') == output + strlen(output) - 1U);
    assert(!strstr(output, "prof") && !strstr(output, "enc"));
    fault = 4U;
    motor.sensorless_takeover = true;
    assert(ol_print_rpm_brief() == 0);
    assert(strstr(output, "tk=1 fb=invalid fault=4"));
    assert(writes == 2U && flushes == 2U && crit == 0U);
    puts("test_rpm_brief: pass");
    return 0;
}
