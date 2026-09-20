/*
 * @Description: 电流/电压偏移校准
 * @Author: lkc
 */
#ifndef _MOTOR_DETECT_H
#define _MOTOR_DETECT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "motor_types.h"
/* Private define ------------------------------------------------------------*/
#define MOTOR_DETECT_RESULT_VERSION (1U)
/* Private typedef -----------------------------------------------------------*/
/*
 * 旧 v1 Flash 曾保存该结构；v2 不再存储，仅保留调用兼容。R/L/磁链检测仍停用。
 */
typedef enum {
    MOTOR_DETECT_ERROR_NONE = 0,
    MOTOR_DETECT_ERROR_BUSY,
    MOTOR_DETECT_ERROR_MOTOR_RUNNING,
    MOTOR_DETECT_ERROR_FAULT,
    MOTOR_DETECT_ERROR_TIMEOUT,
    MOTOR_DETECT_ERROR_CURRENT,
    MOTOR_DETECT_ERROR_RESISTANCE,
    MOTOR_DETECT_ERROR_INDUCTANCE,
    MOTOR_DETECT_ERROR_FLUX,
    MOTOR_DETECT_ERROR_MECHANICAL_REFERENCE,
    MOTOR_DETECT_ERROR_USER_ABORT,
} motor_detect_error_t;

typedef struct {
    uint32_t version;
    bool resistance_valid;
    bool inductance_valid;
    bool flux_valid;
    bool pole_pairs_valid;
    float resistance_ohm;
    float inductance_h;
    float flux_linkage_wb;
    float pole_pairs;
    float test_current_a;
    float test_electrical_rpm;
    motor_detect_error_t error;
    mc_fault_code fault;
} motor_detect_result_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 校准三相电流零点和驱动时相电压偏移（应用包装：含 RTOS 延时与 PWM）。
 * @detail: 累加/结算见 Motor_Detect_Offset*（可裸机逐步喂样本）；本接口供 Shell/上电调用。
 *          成功后自动把配置/校准写入 Flash；失败不写 Flash。
 * @param {bool} cal_undriven  true 时额外校准未驱动相电压偏移
 * @return {uint32_t} 1 成功，0 被中止
 * @author: lkc
 */
uint32_t DETECT_Offset(bool cal_undriven);

/**
 * @description: 上电偏移校准：Flash 已有有效电流零点则跳过，否则检测并写入 Flash
 * @return {*}
 * @author: lkc
 */
void Detect_Init(void);

/**
 * @description: 开环转一圈，标定霍尔状态与电角度对应表
 * @return {bool} true 表示 6 个有效扇区标定成功
 * @author: lkc
 */
bool DETECT_Hall(void);

/**
 * @description: 偏移校准时 FOC 不得改占空比编排（ISR 仍采样+软件保护）。
 * @return {bool} true 表示校准占用功率桥
 * @author: lkc
 */
bool Motor_Detect_IsActive(void);

/**
 * @description: DMA 完成中断喂入一拍电流/电压，仅零偏 ISR 采集窗口内有效。
 * @param {bool} pwm_down true=TIM1 向下计数（Hall FOC 使用的顶部零矢量窗口）
 * @return {*}
 * @author: lkc
 */
void Motor_Detect_IsrFeed(bool pwm_down);

#ifdef __cplusplus
}
#endif
#endif
