/* MT6701 I2C 接口：寄存器读取顺序、角度发布与连续失败失效。 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "motor_mt6701.h"

static uint8_t s_registers[256];
static uint8_t s_read_order[4];
static uint32_t s_read_count;
static uint16_t s_published_angle;
static bool s_fail_reads;
static uint32_t s_time_us;

/**
 * @description: 主机测试替身：读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 s_time_us
 */
uint32_t Hal_Motor_TimeUs(void)
{
    s_time_us += 100U;
    return s_time_us;
}

/**
 * @description: 主机测试替身：初始化当前编码器硬件接口（MT6701 使用 I2C2）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Hal_Encoder_Init(void)
{
}

/**
 * @description: 主机测试替身：读取位置传感器 I2C 寄存器。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint8_t} address_7bit address_7bit 参数
 * @param {uint8_t} reg reg 参数
 * @param {uint8_t *} value 待处理值
 * @return {bool} true=读取成功
 */
bool Hal_Encoder_I2cReadRegister(uint8_t address_7bit, uint8_t reg, uint8_t *value)
{
    assert(address_7bit == 0x06U);
    if (s_fail_reads) {
        return false;
    }
    s_read_order[s_read_count++] = reg;
    *value = s_registers[reg];
    return true;
}

/**
 * @description: 主机测试替身：初始化位置传感器 SSI 读取接口（MT6701）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Hal_Encoder_SsiInit(void)
{
}

/**
 * @description: 主机测试替身：读取一帧 24 位 SSI 角度数据
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t *} frame 兼容参数，本实现不使用
 * @return {bool} 本实现固定返回 false
 */
bool Hal_Encoder_SsiReadFrame(uint32_t *frame)
{
    (void)frame;
    return false;
}

/**
 * @description: 主机测试替身：发布 MT6701 14 位角度快照
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint16_t} count 计数值
 * @return {void} 无
 */
void Hal_Encoder_SetCount(uint16_t count)
{
    s_published_angle = count;
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    motor_mt6701_status_t status;

    Motor_MT6701_Init();
    assert(!Motor_MT6701_IsValid());

    s_registers[0x03U] = 0xABU;
    s_registers[0x04U] = 0xC0U;
    assert(Motor_MT6701_Poll());
    assert(s_read_count == 2U);
    assert(s_read_order[0] == 0x03U);
    assert(s_read_order[1] == 0x04U);
    assert(s_published_angle == (uint16_t)((0xABU << 6U) | (0xC0U >> 2U)));
    assert(Motor_MT6701_IsValid());
    assert(Motor_MT6701_GetSampleSequence() == 1U);

    s_fail_reads = true;
    assert(!Motor_MT6701_Poll());
    assert(Motor_MT6701_IsValid());
    assert(!Motor_MT6701_Poll());
    assert(Motor_MT6701_IsValid());
    assert(!Motor_MT6701_Poll());
    assert(!Motor_MT6701_IsValid());

    Motor_MT6701_GetStatus(&status);
    assert(status.success_count == 1U);
    assert(status.error_count == 3U);
    assert(status.consecutive_errors == 3U);
    assert(status.sample_sequence == 1U);
    assert(status.sample_time_us == 100U);
    puts("test_motor_mt6701: pass");
    return 0;
}
