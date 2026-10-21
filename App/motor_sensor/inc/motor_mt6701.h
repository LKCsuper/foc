/*
 * @Description: MT6701 I2C 绝对角度传感器
 * @Author: lkc
 */
#ifndef MOTOR_MT6701_H
#define MOTOR_MT6701_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "motor_config.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_MT6701_COUNTS_PER_REV (16384U)
#define MOTOR_MT6701_INTERFACE_I2C  (1U)
#define MOTOR_MT6701_INTERFACE_SSI  (2U)
#ifndef MOTOR_MT6701_INTERFACE
#define MOTOR_MT6701_INTERFACE MOTOR_MT6701_INTERFACE_SSI
#endif
#if MOTOR_MT6701_INTERFACE == MOTOR_MT6701_INTERFACE_SSI
#define MOTOR_MT6701_POLL_HZ (10000U)
#else
#define MOTOR_MT6701_POLL_HZ (1000U)
#endif
/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint16_t raw_angle;
    uint8_t magnetic_status;
    uint32_t success_count;
    uint32_t error_count;
    uint32_t crc_error_count;
    uint32_t transfer_error_count;
    uint32_t status_error_count;
    uint32_t consecutive_errors;
    uint8_t last_bad_magnetic_status;
    uint32_t sample_sequence; /* 每次成功采样递增，供控制侧识别新帧 */
    uint32_t sample_time_us;  /* 最近一次成功采样的系统时间 */
    bool valid;
} motor_mt6701_status_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 清零 MT6701 状态并初始化当前接口（SSI 或 I2C）。
 * @return {*}
 * @author: lkc
 */
#if MOTOR_CFG_ENCODER
void Motor_MT6701_Init(void);

/**
 * @description: 读一帧绝对角度并更新有效标志。
 * @detail: SSI 校验CRC6和磁场强弱状态；Mg[3]超速位仅记录告警。连续失败约2ms后valid变false。
 * @return {bool} true=本帧角度可用
 * @author: lkc
 */
bool Motor_MT6701_Poll(void);

/**
 * @description: 复制最近一次 MT6701 状态。
 * @param {motor_mt6701_status_t *} status  输出，不可为空
 * @return {*}
 * @author: lkc
 */
void Motor_MT6701_GetStatus(motor_mt6701_status_t *status);

/**
 * @description: 查询最近成功帧是否仍有效。
 * @return {bool} true=角度可用
 * @author: lkc
 */
bool Motor_MT6701_IsValid(void);

/**
 * @description: 读取成功采样序号。
 * @return {uint32_t} 每成功一帧加 1
 * @author: lkc
 */
uint32_t Motor_MT6701_GetSampleSequence(void);

/**
 * @description: 返回当前编译选定的接口名。
 * @return {const char *} "SSI" 或 "I2C"
 * @author: lkc
 */
const char *Motor_MT6701_InterfaceName(void);
#else
static inline void Motor_MT6701_Init(void) {}
static inline bool Motor_MT6701_Poll(void) { return false; }
static inline void Motor_MT6701_GetStatus(motor_mt6701_status_t *status)
{
    if (status != NULL) {
        motor_mt6701_status_t zero = {0};
        *status = zero;
    }
}
static inline bool Motor_MT6701_IsValid(void) { return false; }
static inline uint32_t Motor_MT6701_GetSampleSequence(void) { return 0U; }
static inline const char *Motor_MT6701_InterfaceName(void) { return "off"; }
#endif
#ifdef __cplusplus
}
#endif
#endif
