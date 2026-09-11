/*
 * @Description: 传统 BLDC 反电动势过零无感六步控制器
 * @Author: lkc
 */
#ifndef MOTOR_BLDC_SENSORLESS_H
#define MOTOR_BLDC_SENSORLESS_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "motor_bldc_six_step.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef enum {
    MOTOR_BLDC_SENSORLESS_IDLE = 0,
    MOTOR_BLDC_SENSORLESS_ALIGN,
    MOTOR_BLDC_SENSORLESS_OPEN_LOOP,
    MOTOR_BLDC_SENSORLESS_CLOSED_LOOP,
    MOTOR_BLDC_SENSORLESS_FAULT,
} motor_bldc_sensorless_mode_t;

typedef enum {
    MOTOR_BLDC_SENSORLESS_FAULT_NONE = 0,
    MOTOR_BLDC_SENSORLESS_FAULT_INVALID_CONFIG,
    MOTOR_BLDC_SENSORLESS_FAULT_COMMUTATOR,
    MOTOR_BLDC_SENSORLESS_FAULT_STARTUP_TIMEOUT,
    MOTOR_BLDC_SENSORLESS_FAULT_ZERO_CROSS_LOST,
    MOTOR_BLDC_SENSORLESS_FAULT_INVALID_SAMPLE,
} motor_bldc_sensorless_fault_t;

typedef struct {
    float align_time_s;              /* 固定一步定向时间 */
    float startup_initial_period_s;  /* 开环首段 60° 换相周期 */
    float startup_final_period_s;    /* 开环斜坡末段换相周期 */
    float startup_ramp_time_s;       /* 从 initial 线性降到 final 的时间 */
    float startup_timeout_s;         /* 未接管前最长运行时间 */
    float commutation_blanking_s;    /* 每次换相后的 BEMF 消隐时间 */
    float zero_cross_threshold_v;    /* 过零迟滞阈值，按电机端实际电压 */
    float bemf_filter_alpha;         /* 一阶滤波系数 (0,1]；1=不滤波 */
    uint8_t takeover_crossings;      /* 连续有效过零次数达到该值后接管 */
    float closed_loop_timeout_s;     /* 闭环单个扇区允许的最长时间 */
} motor_bldc_sensorless_config_t;

typedef struct {
    motor_bldc_sensorless_config_t config;
    motor_bldc_six_step_t *commutator;
    motor_bldc_sensorless_mode_t mode;
    motor_bldc_sensorless_fault_t fault;
    motor_bldc_direction_t direction;
    float state_elapsed_s;
    float startup_elapsed_s;
    float sector_elapsed_s;
    float scheduled_delay_s;
    float filtered_bemf_v;
    uint8_t step;
    uint8_t valid_crossings;
    bool crossing_armed;
    bool crossing_seen;
    bool commutation_scheduled;
    bool initialized;
} motor_bldc_sensorless_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 填充无感六步控制默认配置。
 * @detail: 参数仅用于低压限流条件下的起步调试，不代表已经通过 BM75 上板验收。
 * @param {motor_bldc_sensorless_config_t *} config 输出配置，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSensorless_DefaultConfig(motor_bldc_sensorless_config_t *config);

/**
 * @description: 初始化一个独立的无感六步控制实例。
 * @detail: 只保存状态与配置，不读取 ADC、不操作 PWM、不启动电机。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {motor_bldc_six_step_t *} commutator 六步输出实例，不可为空
 * @param {const motor_bldc_sensorless_config_t *} config 无感配置，不可为空
 * @return {bool} 初始化成功返回 true
 * @author: lkc
 */
bool Motor_BldcSensorless_Init(motor_bldc_sensorless_t *control,
                               motor_bldc_six_step_t *commutator,
                               const motor_bldc_sensorless_config_t *config);

/**
 * @description: 启动无感六步状态机。
 * @detail: 先输出 step 0 定向，再进入开环强制换相；不会绕过六步输出实例的故障检查。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {motor_bldc_direction_t} direction 转向，只允许 CW 或 CCW
 * @param {float} duty PWM 占空比，自动限制到 [0,1]
 * @return {bool} 成功进入定向状态返回 true
 * @author: lkc
 */
bool Motor_BldcSensorless_Start(motor_bldc_sensorless_t *control,
                                motor_bldc_direction_t direction, float duty);

/**
 * @description: 停止无感六步状态机。
 * @detail: 同时停止关联的六步输出实例并请求三相悬空；保留故障码。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例；NULL 时忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSensorless_Stop(motor_bldc_sensorless_t *control);

/**
 * @description: 推进一个无感六步控制周期。
 * @detail: 根据悬空相相对软件中性点的 BEMF 过零推进开环接管或闭环 30° 延迟换相。
 * @param {motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @param {const float [3]} phase_voltage 控制器逻辑 A/B/C 相端电压 (V)，不可为空
 * @param {float} bus_voltage 直流母线电压 (V)，须为有限正值
 * @param {float} dt_s 本控制周期时间 (s)，须为有限正值
 * @return {bool} 本周期保持有效运行返回 true
 * @author: lkc
 */
bool Motor_BldcSensorless_Step(motor_bldc_sensorless_t *control,
                               const float phase_voltage[3], float bus_voltage, float dt_s);

/**
 * @description: 获取无感六步运行状态。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {motor_bldc_sensorless_mode_t} 当前状态；control 为空时返回 FAULT
 * @author: lkc
 */
motor_bldc_sensorless_mode_t Motor_BldcSensorless_GetMode(const motor_bldc_sensorless_t *control);

/**
 * @description: 获取无感六步故障码。
 * @param {const motor_bldc_sensorless_t *} control 无感控制实例，不可为空
 * @return {motor_bldc_sensorless_fault_t} 当前故障；control 为空时返回 INVALID_CONFIG
 * @author: lkc
 */
motor_bldc_sensorless_fault_t Motor_BldcSensorless_GetFault(const motor_bldc_sensorless_t *control);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif /* MOTOR_BLDC_SENSORLESS_H */
