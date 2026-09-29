/*
 * @Description: MT6701 I2C 绝对角度传感器
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "motor_mt6701.h"
#include "hal_sensor.h"
#include "hal_motor.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_MT6701_I2C_ADDRESS_7BIT (0x06U)
#define MOTOR_MT6701_REG_ANGLE_MSB     (0x03U)
#define MOTOR_MT6701_REG_ANGLE_LSB     (0x04U)
#define MOTOR_MT6701_ERROR_LIMIT       (3U)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile motor_mt6701_status_t s_mt6701;
/* Private function prototypes -----------------------------------------------*/
#if MOTOR_MT6701_INTERFACE == MOTOR_MT6701_INTERFACE_SSI
static uint8_t Motor_MT6701_Crc6(uint32_t data, uint8_t bit_count);
#endif
/* Private functions ---------------------------------------------------------*/

#if MOTOR_MT6701_INTERFACE == MOTOR_MT6701_INTERFACE_SSI
static uint8_t Motor_MT6701_Crc6(uint32_t data, uint8_t bit_count)
{
    uint8_t crc = 0U;
    int bit;

    for (bit = (int)bit_count - 1; bit >= 0; bit--) {
        uint8_t feedback = (uint8_t)(((crc >> 5U) & 1U) ^ ((data >> (uint32_t)bit) & 1U));

        crc = (uint8_t)((crc << 1U) & 0x3FU);
        if (feedback != 0U) {
            crc ^= 0x03U; /* x^6 + x + 1 */
        }
    }
    return crc;
}
#endif

/**
 * @description: 清零 MT6701 状态并初始化当前接口（SSI 或 I2C）。
 * @return {*}
 * @author: lkc
 */
void Motor_MT6701_Init(void)
{
    s_mt6701.raw_angle = 0U;
    s_mt6701.magnetic_status = 0U;
    s_mt6701.success_count = 0U;
    s_mt6701.error_count = 0U;
    s_mt6701.crc_error_count = 0U;
    s_mt6701.consecutive_errors = 0U;
    s_mt6701.sample_sequence = 0U;
    s_mt6701.sample_time_us = 0U;
    s_mt6701.valid = false;
#if MOTOR_MT6701_INTERFACE == MOTOR_MT6701_INTERFACE_I2C
    Hal_Encoder_Init();
#elif MOTOR_MT6701_INTERFACE == MOTOR_MT6701_INTERFACE_SSI
    Hal_Encoder_SsiInit();
#else
#error "Unsupported MOTOR_MT6701_INTERFACE"
#endif
}

/**
 * @description: 读一帧绝对角度并更新有效标志。
 * @detail: SSI 校验 CRC6 和磁场状态；连续失败达到上限后 valid 变 false。
 * @return {bool} true=本帧角度可用
 * @author: lkc
 */
bool Motor_MT6701_Poll(void)
{
#if MOTOR_MT6701_INTERFACE == MOTOR_MT6701_INTERFACE_I2C
    uint8_t msb;
    uint8_t lsb;
    uint16_t angle;

    /* 手册要求先读 0x03，再读 0x04。 */
    if (!Hal_Encoder_I2cReadRegister(MOTOR_MT6701_I2C_ADDRESS_7BIT, MOTOR_MT6701_REG_ANGLE_MSB, &msb) ||
        !Hal_Encoder_I2cReadRegister(MOTOR_MT6701_I2C_ADDRESS_7BIT, MOTOR_MT6701_REG_ANGLE_LSB, &lsb)) {
        s_mt6701.error_count++;
        s_mt6701.consecutive_errors++;
        if (s_mt6701.consecutive_errors >= MOTOR_MT6701_ERROR_LIMIT) {
            s_mt6701.valid = false;
        }
        return false;
    }

    angle = (uint16_t)(((uint16_t)msb << 6U) | ((uint16_t)lsb >> 2U));
    angle &= (MOTOR_MT6701_COUNTS_PER_REV - 1U);
    s_mt6701.magnetic_status = 0U;
#else
    uint32_t frame;
    uint32_t payload;
    uint16_t angle;
    uint8_t received_crc;

    if (!Hal_Encoder_SsiReadFrame(&frame)) {
        s_mt6701.error_count++;
        s_mt6701.consecutive_errors++;
        if (s_mt6701.consecutive_errors >= MOTOR_MT6701_ERROR_LIMIT) {
            s_mt6701.valid = false;
        }
        return false;
    }

    payload = (frame >> 6U) & 0x3FFFFU; /* D[13:0] + Mg[3:0] */
    received_crc = (uint8_t)(frame & 0x3FU);
    if (Motor_MT6701_Crc6(payload, 18U) != received_crc) {
        s_mt6701.error_count++;
        s_mt6701.crc_error_count++;
        s_mt6701.consecutive_errors++;
        if (s_mt6701.consecutive_errors >= MOTOR_MT6701_ERROR_LIMIT) {
            s_mt6701.valid = false;
        }
        return false;
    }

    angle = (uint16_t)((frame >> 10U) & 0x3FFFU);
    s_mt6701.magnetic_status = (uint8_t)((frame >> 6U) & 0x0FU);
    /* Mg[1:0]=磁场强弱异常，Mg[3]=超速；Mg[2] 仅为 PUSH 状态。 */
    if ((s_mt6701.magnetic_status & 0x0BU) != 0U) {
        s_mt6701.error_count++;
        s_mt6701.consecutive_errors++;
        if (s_mt6701.consecutive_errors >= MOTOR_MT6701_ERROR_LIMIT) {
            s_mt6701.valid = false;
        }
        return false;
    }
#endif
    Hal_Encoder_SetCount(angle);
    s_mt6701.raw_angle = angle;
    s_mt6701.success_count++;
    s_mt6701.sample_time_us = Hal_Motor_TimeUs();
    s_mt6701.sample_sequence++;
    s_mt6701.consecutive_errors = 0U;
    s_mt6701.valid = true;
    return true;
}

/**
 * @description: 复制最近一次 MT6701 状态。
 * @param {motor_mt6701_status_t *} status  输出，不可为空
 * @return {*}
 * @author: lkc
 */
void Motor_MT6701_GetStatus(motor_mt6701_status_t *status)
{
    if (status == NULL) {
        return;
    }
    status->raw_angle = s_mt6701.raw_angle;
    status->magnetic_status = s_mt6701.magnetic_status;
    status->success_count = s_mt6701.success_count;
    status->error_count = s_mt6701.error_count;
    status->crc_error_count = s_mt6701.crc_error_count;
    status->consecutive_errors = s_mt6701.consecutive_errors;
    status->sample_sequence = s_mt6701.sample_sequence;
    status->sample_time_us = s_mt6701.sample_time_us;
    status->valid = s_mt6701.valid;
}

/**
 * @description: 查询最近成功帧是否仍有效。
 * @return {bool} true=角度可用
 * @author: lkc
 */
bool Motor_MT6701_IsValid(void)
{
    return s_mt6701.valid;
}

/**
 * @description: 读取成功采样序号。
 * @return {uint32_t} 每成功一帧加 1
 * @author: lkc
 */
uint32_t Motor_MT6701_GetSampleSequence(void)
{
    return s_mt6701.sample_sequence;
}

/**
 * @description: 返回当前编译选定的接口名。
 * @return {const char *} "SSI" 或 "I2C"
 * @author: lkc
 */
const char *Motor_MT6701_InterfaceName(void)
{
#if MOTOR_MT6701_INTERFACE == MOTOR_MT6701_INTERFACE_I2C
    return "I2C";
#else
    return "SSI";
#endif
}

#ifdef __cplusplus
}
#endif
