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
#include "motor_mt6701_protocol.h"
#include "hal_sensor.h"
#include "hal_motor.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_MT6701_I2C_ADDRESS_7BIT (0x06U)
#define MOTOR_MT6701_REG_ANGLE_MSB     (0x03U)
#define MOTOR_MT6701_REG_ANGLE_LSB     (0x04U)
#define MOTOR_MT6701_ERROR_TIMEOUT_MS  (2U)
#define MOTOR_MT6701_ERROR_LIMIT_CALC  ((MOTOR_MT6701_POLL_HZ * MOTOR_MT6701_ERROR_TIMEOUT_MS + 999U) / 1000U)
#define MOTOR_MT6701_ERROR_LIMIT       ((MOTOR_MT6701_ERROR_LIMIT_CALC < 3U) ? 3U : MOTOR_MT6701_ERROR_LIMIT_CALC)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile motor_mt6701_status_t s_mt6701;
/* Private function prototypes -----------------------------------------------*/
typedef enum {
    MOTOR_MT6701_FAILURE_TRANSFER = 0,
    MOTOR_MT6701_FAILURE_CRC,
    MOTOR_MT6701_FAILURE_STATUS
} motor_mt6701_failure_t;
static void Motor_MT6701_RecordFailure(motor_mt6701_failure_t reason);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 记录一次坏帧；连续失败达到约2ms后作废角度。
 * @detail: 成功帧在 Poll 末尾清零 consecutive_errors 并置 valid。此处按传输、CRC、状态位分类累计失败。
 * @param {motor_mt6701_failure_t} reason  本帧失败原因
 * @return {void} 无
 * @author: lkc
 */
static void Motor_MT6701_RecordFailure(motor_mt6701_failure_t reason)
{
    s_mt6701.error_count++;
    if (reason == MOTOR_MT6701_FAILURE_CRC) {
        s_mt6701.crc_error_count++;
    } else if (reason == MOTOR_MT6701_FAILURE_TRANSFER) {
        s_mt6701.transfer_error_count++;
    } else if (reason == MOTOR_MT6701_FAILURE_STATUS) {
        s_mt6701.status_error_count++;
        s_mt6701.last_bad_magnetic_status = s_mt6701.magnetic_status;
    }
    s_mt6701.consecutive_errors++;
    if (s_mt6701.consecutive_errors >= MOTOR_MT6701_ERROR_LIMIT) {
        s_mt6701.valid = false;
    }
}

/**
 * @description: 记录不影响当前角度有效性的MT6701状态告警。
 * @detail: Mg[3]超速位可能瞬态出现；实际电机超速由软件测速保护判断，不让该位单独中断FOC。
 * @return {void}
 */
#if MOTOR_MT6701_INTERFACE == MOTOR_MT6701_INTERFACE_SSI
static void Motor_MT6701_RecordStatusWarning(void)
{
    s_mt6701.error_count++;
    s_mt6701.status_error_count++;
    s_mt6701.last_bad_magnetic_status = s_mt6701.magnetic_status;
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
    s_mt6701.transfer_error_count = 0U;
    s_mt6701.status_error_count = 0U;
    s_mt6701.consecutive_errors = 0U;
    s_mt6701.last_bad_magnetic_status = 0U;
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
        Motor_MT6701_RecordFailure(MOTOR_MT6701_FAILURE_TRANSFER);
        return false;
    }

    angle = Motor_MT6701_DecodeI2c(msb, lsb);
    s_mt6701.magnetic_status = 0U;
#else
    uint32_t frame;
    uint16_t angle;
    motor_mt6701_frame_t decoded;
    motor_mt6701_frame_result_t result;

    if (!Hal_Encoder_SsiReadFrame(&frame)) {
        Motor_MT6701_RecordFailure(MOTOR_MT6701_FAILURE_TRANSFER);
        return false;
    }

    result = Motor_MT6701_DecodeSsi(frame, &decoded);
    if (result == MOTOR_MT6701_FRAME_CRC_ERROR) {
        Motor_MT6701_RecordFailure(MOTOR_MT6701_FAILURE_CRC);
        return false;
    }
    angle = decoded.angle;
    s_mt6701.magnetic_status = decoded.magnetic_status;
    if (result == MOTOR_MT6701_FRAME_MAGNETIC_ERROR) {
        Motor_MT6701_RecordFailure(MOTOR_MT6701_FAILURE_STATUS);
        return false;
    }
    if (decoded.overspeed_warning) {
        Motor_MT6701_RecordStatusWarning();
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
    status->transfer_error_count = s_mt6701.transfer_error_count;
    status->status_error_count = s_mt6701.status_error_count;
    status->consecutive_errors = s_mt6701.consecutive_errors;
    status->last_bad_magnetic_status = s_mt6701.last_bad_magnetic_status;
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
