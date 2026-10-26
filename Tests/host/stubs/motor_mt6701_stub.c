/* MT6701 依赖替身：为不测 MT6701 的用例提供可链接的空实现。 */
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "motor_mt6701.h"
/* Private variables ---------------------------------------------------------*/
static motor_mt6701_status_t s_status;
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：设置后续 MT6701 测试接口返回的角度、时间、序号和有效性。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} sequence sequence 参数
 * @param {uint32_t} timestamp timestamp 参数
 * @return {void} 无
 */
void Test_MT6701_SetSample(uint32_t sequence, uint32_t timestamp)
{
    s_status.valid = true;
    s_status.sample_sequence = sequence;
    s_status.sample_time_us = timestamp;
}

/**
 * @description: 主机测试替身：清零 MT6701 状态并初始化当前接口（SSI 或 I2C）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_MT6701_Init(void)
{
}

/**
 * @description: 主机测试替身：读一帧绝对角度并更新有效标志。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 true
 */
bool Motor_MT6701_Poll(void)
{
    return true;
}

/**
 * @description: 主机测试替身：复制最近一次 MT6701 状态。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_mt6701_status_t *} status status 参数
 * @return {void} 无
 */
void Motor_MT6701_GetStatus(motor_mt6701_status_t *status)
{
    *status = s_status;
}

/**
 * @description: 主机测试替身：查询最近成功帧是否仍有效。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 true
 */
bool Motor_MT6701_IsValid(void)
{
    return true;
}

/**
 * @description: 主机测试替身：读取成功采样序号。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 ++sequence
 */
uint32_t Motor_MT6701_GetSampleSequence(void)
{
    static uint32_t sequence;
    return ++sequence;
}

/**
 * @description: 主机测试替身：返回当前编译选定的接口名。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {const char *} 测试记录值 "HOST"
 */
const char *Motor_MT6701_InterfaceName(void)
{
    return "HOST";
}
