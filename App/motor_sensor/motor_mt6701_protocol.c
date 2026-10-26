/* MT6701 帧解码：SSI CRC6 校验与 I2C 字节拼装，不依赖 HAL 或全局状态。 */
/* Includes ------------------------------------------------------------------*/
#include "motor_mt6701_protocol.h"

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 计算 MT6701 SSI 帧 18 位数据的 CRC6（生成多项式 x^6+x+1）。
 * @param {uint32_t} data 数据缓冲
 * @return {uint8_t} 当前值 crc
 */
static uint8_t Motor_MT6701_Crc6(uint32_t data)
{
    uint8_t crc = 0U;
    int bit;
    for (bit = 17; bit >= 0; bit--) {
        uint8_t feedback = (uint8_t)(((crc >> 5U) & 1U) ^ ((data >> (uint32_t)bit) & 1U));
        crc = (uint8_t)((crc << 1U) & 0x3FU);
        if (feedback != 0U) crc ^= 0x03U; /* x^6+x+1，保持原SSI驱动算法 */
    }
    return crc;
}

/**
 * @description: 校验并解码 24 位 SSI 帧，输出角度与磁场状态。
 * @detail: CRC 错误时不改写 decoded；CRC 正确时磁场异常作为单独结果返回。
 * @param {uint32_t} frame  24 位 SSI 原始帧
 * @param {motor_mt6701_frame_t *} decoded  输出解码结果，不可为空
 * @return {motor_mt6701_frame_result_t} OK / CRC_ERROR / MAGNETIC_ERROR
 */
motor_mt6701_frame_result_t Motor_MT6701_DecodeSsi(uint32_t frame, motor_mt6701_frame_t *decoded)
{
    if (Motor_MT6701_Crc6((frame >> 6U) & 0x3FFFFU) != (uint8_t)(frame & 0x3FU)) return MOTOR_MT6701_FRAME_CRC_ERROR;
    decoded->angle = (uint16_t)((frame >> 10U) & 0x3FFFU);
    decoded->magnetic_status = (uint8_t)((frame >> 6U) & 0x0FU);
    decoded->overspeed_warning = (decoded->magnetic_status & 0x08U) != 0U;
    return (decoded->magnetic_status & 0x03U) != 0U ?
        MOTOR_MT6701_FRAME_MAGNETIC_ERROR : MOTOR_MT6701_FRAME_OK;
}

/**
 * @description: 由 I2C 依次读到的 MSB/LSB 拼出 14 位角度。
 * @param {uint8_t} msb  寄存器 0x03 读出的高位字节
 * @param {uint8_t} lsb  寄存器 0x04 读出的低位字节
 * @return {uint16_t} 14 位角度计数
 */
uint16_t Motor_MT6701_DecodeI2c(uint8_t msb, uint8_t lsb)
{
    return (uint16_t)((((uint16_t)msb << 6U) | ((uint16_t)lsb >> 2U)) & 0x3FFFU);
}
