#include "motor_mt6701_protocol.h"

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

motor_mt6701_frame_result_t Motor_MT6701_DecodeSsi(uint32_t frame, motor_mt6701_frame_t *decoded)
{
    if (Motor_MT6701_Crc6((frame >> 6U) & 0x3FFFFU) != (uint8_t)(frame & 0x3FU)) {
        return MOTOR_MT6701_FRAME_CRC_ERROR;
    }
    decoded->angle = (uint16_t)((frame >> 10U) & 0x3FFFU);
    decoded->magnetic_status = (uint8_t)((frame >> 6U) & 0x0FU);
    decoded->overspeed_warning = (decoded->magnetic_status & 0x08U) != 0U;
    return (decoded->magnetic_status & 0x03U) != 0U ?
        MOTOR_MT6701_FRAME_MAGNETIC_ERROR : MOTOR_MT6701_FRAME_OK;
}

uint16_t Motor_MT6701_DecodeI2c(uint8_t msb, uint8_t lsb)
{
    return (uint16_t)((((uint16_t)msb << 6U) | ((uint16_t)lsb >> 2U)) & 0x3FFFU);
}
