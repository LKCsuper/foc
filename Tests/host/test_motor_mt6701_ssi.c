/* MT6701 SSI 接口：CRC6 校验、状态/传输错误计数与重试失效。 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "motor_mt6701.h"

static uint32_t s_frame;
static uint16_t s_published_angle;
static bool s_transfer_ok = true;
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
 * @description: 计算测试帧的 CRC6。
 * @param {uint32_t} data 数据缓冲
 * @param {uint8_t} bit_count bit_count 参数
 * @return {uint8_t} 测试记录值 crc
 */
static uint8_t crc6(uint32_t data, uint8_t bit_count)
{
    uint8_t crc = 0U;
    int bit;

    for (bit = (int)bit_count - 1; bit >= 0; bit--) {
        uint8_t feedback = (uint8_t)(((crc >> 5U) & 1U) ^ ((data >> (uint32_t)bit) & 1U));

        crc = (uint8_t)((crc << 1U) & 0x3FU);
        if (feedback != 0U) {
            crc ^= 0x03U;
        }
    }
    return crc;
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
 * @param {uint32_t *} frame frame 参数
 * @return {bool} true=读取成功
 */
bool Hal_Encoder_SsiReadFrame(uint32_t *frame)
{
    if (!s_transfer_ok) {
        return false;
    }
    *frame = s_frame;
    return true;
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
    uint16_t angle = 0x2345U;
    uint8_t magnetic_status = 0x04U;
    uint32_t payload = ((uint32_t)angle << 4U) | magnetic_status;
    uint32_t retry;

    Motor_MT6701_Init();
    assert(!Motor_MT6701_IsValid());
    assert(Motor_MT6701_InterfaceName()[0] == 'S');

    s_frame = (payload << 6U) | crc6(payload, 18U);
    assert(Motor_MT6701_Poll());
    assert(s_published_angle == angle);
    Motor_MT6701_GetStatus(&status);
    assert(status.valid);
    assert(status.magnetic_status == magnetic_status);
    assert(status.crc_error_count == 0U);
    assert(status.transfer_error_count == 0U);
    assert(status.status_error_count == 0U);
    assert(status.sample_sequence == 1U);
    assert(status.sample_time_us == 100U);

    s_frame ^= 1U;
    assert(!Motor_MT6701_Poll());
    Motor_MT6701_GetStatus(&status);
    assert(status.crc_error_count == 1U);
    assert(status.transfer_error_count == 0U);

    magnetic_status = 0x08U;
    payload = ((uint32_t)angle << 4U) | magnetic_status;
    s_frame = (payload << 6U) | crc6(payload, 18U);
    assert(Motor_MT6701_Poll());
    Motor_MT6701_GetStatus(&status);
    assert(status.status_error_count == 1U);
    assert(status.last_bad_magnetic_status == magnetic_status);
    assert(status.valid);
    assert(status.consecutive_errors == 0U);

    s_transfer_ok = false;
    assert(!Motor_MT6701_Poll());
    Motor_MT6701_GetStatus(&status);
    assert(status.transfer_error_count == 1U);
    assert(Motor_MT6701_IsValid());
    for (retry = 0U; retry < 32U && Motor_MT6701_IsValid(); retry++) {
        assert(!Motor_MT6701_Poll());
    }
    assert(!Motor_MT6701_IsValid());
    puts("test_motor_mt6701_ssi: pass");
    return 0;
}
