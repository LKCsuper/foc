/*
 * @Description: 框架公共类型（故障码、霍尔表常量等）
 * @Author: lkc
 */
#ifndef _MOTOR_TYPES_H
#define _MOTOR_TYPES_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_HALL_TABLE_SIZE     (8U)
#define MOTOR_HALL_TABLE_INVALID  (-1)
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 电机故障码（与 VESC mc_fault_code 数值兼容）
 */
typedef enum {
    FAULT_CODE_NONE = 0,
    FAULT_CODE_OVER_VOLTAGE,
    FAULT_CODE_UNDER_VOLTAGE,
    FAULT_CODE_DRV,
    FAULT_CODE_ABS_OVER_CURRENT,
    FAULT_CODE_OVER_TEMP_FET,
    FAULT_CODE_OVER_TEMP_MOTOR,
    FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,
    FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,
    FAULT_CODE_MCU_UNDER_VOLTAGE,
    FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,
    FAULT_CODE_ENCODER_SPI,
    FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,
    FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,
    FAULT_CODE_FLASH_CORRUPTION,
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,
    FAULT_CODE_UNBALANCED_CURRENTS,
    FAULT_CODE_BRK,
    FAULT_CODE_RESOLVER_LOT,
    FAULT_CODE_RESOLVER_DOS,
    FAULT_CODE_RESOLVER_LOS,
    FAULT_CODE_FLASH_CORRUPTION_APP_CFG,
    FAULT_CODE_FLASH_CORRUPTION_MC_CFG,
    FAULT_CODE_ENCODER_NO_MAGNET,
    FAULT_CODE_ENCODER_MAGNET_TOO_STRONG,
    FAULT_CODE_PHASE_FILTER,
    FAULT_CODE_ENCODER_FAULT,
    FAULT_CODE_LV_OUTPUT_FAULT,
    FAULT_CODE_MOTOR_STALL,
    FAULT_CODE_MOTOR_SPEED_LOSS,
    FAULT_CODE_CONTROL_TIMING,
} mc_fault_code;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif
