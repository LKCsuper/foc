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
 * @description: 电机故障码
 * @detail: 0~29 与 VESC mc_fault_code 的数值和含义一致；30 起为本框架扩展，
 *          VESC 同位置是 ENCODER_SLIP/OVERSPEED 等，不能按 VESC 解释。
 *          目前本框架实际产生：ABS_OVER_CURRENT、UNDER_VOLTAGE、MOTOR_STALL、
 *          MOTOR_SPEED_LOSS、CONTROL_TIMING、ENCODER_SPI；其余为预留码。
 */
typedef enum {
    FAULT_CODE_NONE = 0,                                   /* 无故障；错误路径也表示未识别 */
    FAULT_CODE_OVER_VOLTAGE,                               /* 母线过压 */
    FAULT_CODE_UNDER_VOLTAGE,                              /* 母线欠压 */
    FAULT_CODE_DRV,                                        /* 门极驱动器故障（DRV nFAULT） */
    FAULT_CODE_ABS_OVER_CURRENT,                           /* 相电流瞬时绝对值过流 */
    FAULT_CODE_OVER_TEMP_FET,                              /* 功率管过温 */
    FAULT_CODE_OVER_TEMP_MOTOR,                            /* 电机绕组过温 */
    FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,                   /* 门极驱动供电过压 */
    FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,                  /* 门极驱动供电欠压 */
    FAULT_CODE_MCU_UNDER_VOLTAGE,                          /* MCU 供电欠压 */
    FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,                /* 看门狗复位后启动 */
    FAULT_CODE_ENCODER_SPI,                                /* 编码器 SPI 通信错误率过高 */
    FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,         /* 正余弦编码器幅值低于下限 */
    FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,         /* 正余弦编码器幅值高于上限 */
    FAULT_CODE_FLASH_CORRUPTION,                           /* Flash 数据损坏（通用） */
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,               /* 电流传感器 1 零偏过大 */
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,               /* 电流传感器 2 零偏过大 */
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,               /* 电流传感器 3 零偏过大 */
    FAULT_CODE_UNBALANCED_CURRENTS,                        /* 三相电流和不为零（不平衡） */
    FAULT_CODE_BRK,                                        /* 硬件 Break/刹车输入触发 */
    FAULT_CODE_RESOLVER_LOT,                               /* 旋变跟踪丢失 (Loss Of Tracking) */
    FAULT_CODE_RESOLVER_DOS,                               /* 旋变信号劣化 (Degradation Of Signal) */
    FAULT_CODE_RESOLVER_LOS,                               /* 旋变信号丢失 (Loss Of Signal) */
    FAULT_CODE_FLASH_CORRUPTION_APP_CFG,                   /* 应用配置区 Flash 损坏 */
    FAULT_CODE_FLASH_CORRUPTION_MC_CFG,                    /* 电机配置区 Flash 损坏 */
    FAULT_CODE_ENCODER_NO_MAGNET,                          /* 磁编码器无磁铁/磁场过弱 */
    FAULT_CODE_ENCODER_MAGNET_TOO_STRONG,                  /* 磁编码器磁场过强 */
    FAULT_CODE_PHASE_FILTER,                               /* 相电压观测值与调制幅值不符 */
    FAULT_CODE_ENCODER_FAULT,                              /* 编码器通用故障（通信/状态位） */
    FAULT_CODE_LV_OUTPUT_FAULT,                            /* 低压输出供电故障 */
    FAULT_CODE_MOTOR_STALL,                                /* 堵转：电流达限而转速过低（本框架） */
    FAULT_CODE_MOTOR_SPEED_LOSS,                           /* 失速：转速远低于已达目标或观测失步 */
    FAULT_CODE_CONTROL_TIMING,                             /* 控制周期无效/时序异常（本框架） */
} mc_fault_code;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif
