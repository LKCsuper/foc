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
/* Private define ------------------------------------------------------------*/
#define MOTOR_DETECT_RESULT_VERSION (6U)
#define MOTOR_DETECT_INDUCTANCE_DIRECTIONS (6U)
/* Private typedef -----------------------------------------------------------*/
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
    MOTOR_DETECT_ERROR_USER_ABORT,
    MOTOR_DETECT_ERROR_INVALID_INPUT,
} motor_detect_error_t;

typedef struct {
    uint32_t version;
    bool flux_scan_valid;
    float flux_scan_fit_wb, flux_scan_offset_v, flux_scan_r2;
    float flux_scan_erpm[3], flux_scan_voltage[3], flux_scan_encoder_erpm[3];
    float flux_scan_coverage[3], flux_scan_encoder_voltage[3];
    bool resistance_valid;
    bool inductance_valid;
    bool flux_valid;
    float resistance_ohm;
    float inductance_h;
    float inductance_d_h;
    float inductance_q_h;
    float inductance_direction_h[MOTOR_DETECT_INDUCTANCE_DIRECTIONS];
    float flux_linkage_wb;
    float test_electrical_rpm;
    float test_flux_current_a;
    float inverter_voltage_offset_v;
    float resistance_fit_r2;
    float test_current_a;
    motor_detect_error_t error;
    int fault;
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
 * @description: 校准三相电流零点，只更新 RAM，不写 Flash。
 * @detail: 采样、关桥和中止条件与 DETECT_Offset 相同。供停机诊断调用，阻塞在应用任务中。
 * @param {bool} cal_undriven true 时额外校准未驱动相电压偏移
 * @return {uint32_t} 1 成功，0 被中止
 * @author: lkc
 */
uint32_t DETECT_OffsetRam(bool cal_undriven);

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
 * @description: 检测相电阻和 dq 相电感；只更新 RAM 结果，不应用、不保存。
 * @detail: 调用方应在后台任务中调用。过程会固定转子并注入短脉冲。
 * @param {float} current_a 对齐电流 A，限制在 0.3..2.0 A
 * @param {motor_detect_result_t *} result 结果输出，可为 NULL
 * @return {bool} R/L 均有效返回 true
 */
bool DETECT_MotorRl(float current_a, motor_detect_result_t *result);

/** 完整检测 R/L/磁链；磁链阶段会开环旋转电机。 */
bool DETECT_MotorParameters(float current_a, float electrical_rpm, float flux_current_a,
                            motor_detect_result_t *result);

/** 三速度滑行磁链交叉验证；最高900..3000eRPM，需有效编码器比较，RAM-only。 */
bool DETECT_MotorFluxScan(float current_a, float max_erpm, float flux_current_a, motor_detect_result_t *result);

/** 读取最近一次 R/L 检测结果。 */
void DETECT_GetResult(motor_detect_result_t *result);

/** 请求中止当前 R/L 检测。 */
void DETECT_Abort(void);

/** 清除空闲状态下的中止请求。 */
void DETECT_ClearAbortRequest(void);

/** 检测错误码转可读名称。 */
const char *DETECT_GetErrorName(motor_detect_error_t error);

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
