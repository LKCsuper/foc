/*
 * @Description: 脉振高频注入（HFI）低速凸极跟踪
 * @Author: lkc
 *
 * 学习要点：
 *   1) 反电势跟转速成正比，零速/极低速观测器看不见转子；凸极电机可在估计 d 轴注入高频电压，用 q 轴电流响应估角。
 *   2) 本模块是持续跟踪，不是 motor_hfi_start 的 12 轴扫角。扫角只给一次初值并分辨 N/S；跟踪有 180° 模糊。
 *   3) 注入为方波 ±Vh，频率 = 电流环频率 / 2（15 kHz 电流环 → 7.5 kHz）。不改 PWM/ADC 时序，不使用 V0/V7 双边调制。
 *   4) 只输出要叠加的 Vd_h 和跟踪电角；不写 PWM、不改 PI。表贴机 Ld≈Lq 时无凸极，本算法无效。
 *   5) 符号按 IPMSM（Lq>Ld）推导：正 gain 把估计角推向真角。默认不接入主控制，由调用者决定是否叠加。
 */
#ifndef MOTOR_HFI_H
#define MOTOR_HFI_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_HFI_DEFAULT_BUS_RATIO (0.20f) /* 注入电压不超过母线的默认比例 */
#define MOTOR_HFI_MAX_VOLTAGE (12.0f)       /* 注入电压上限 (V) */
#define MOTOR_HFI_MAX_GAIN (100.0f)         /* 角度校正增益上限 (rad/A) */
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 高频注入配置（不持久化；由调用者每拍传入）
 */
typedef struct {
    bool enabled;                /* true=本拍允许注入 */
    float voltage;               /* 方波幅值 |Vd_h| (V) */
    float gain;                  /* 解调误差 → 电角校正 (rad/A)，须 ≥0 */
    float max_bus_voltage_ratio; /* 注入电压 / 母线上限，(0, 1] */
} motor_hfi_config_t;

/**
 * @description: 高频注入运行状态（不持久化）
 */
typedef struct {
    bool running;        /* 已进入注入序列 */
    bool sample_valid;   /* 已有上一拍 q 轴电流，可解调 */
    int8_t polarity;     /* 上一拍已施加的注入极性，+1 / −1 */
    float angle;         /* 跟踪电角 (rad)，[0, 2π) */
    float speed_e;       /* 估计电角速度 (rad/s) */
    float prev_iq;       /* 上一拍估计坐标系 Iq (A) */
    float last_error;    /* 上一拍解调误差 (A)，供诊断 */
    float last_vd_inj;   /* 上一拍已输出的注入 Vd (V) */
} motor_hfi_t;

/**
 * @description: 本拍注入输出
 */
typedef struct {
    bool apply;   /* true=应将 vd_inj 叠加到电流环 Vd */
    float vd_inj; /* d 轴注入电压 (V) */
    float vq_inj; /* q 轴注入电压 (V)，脉振 HFI 为 0 */
    float angle;  /* 本拍跟踪电角 (rad) */
} motor_hfi_output_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 复位高频注入状态（角度与速度清零，停止注入）
 * @param {motor_hfi_t} *hfi  实例，可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Hfi_Reset(motor_hfi_t *hfi);

/**
 * @description: 规整高频注入配置
 * @detail: 电压/增益钳到非负上限；母线比例非法时回退为 0.20。
 * @param {motor_hfi_config_t} *config  配置，被原地修改；可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Hfi_NormalizeConfig(motor_hfi_config_t *config);

/**
 * @description: 写入跟踪初值电角（通常来自 HFI 扫角或预对齐）
 * @detail: 不改变极性与解调历史。角度归一化到 [0, 2π)。
 * @param {motor_hfi_t} *hfi  实例，可为 NULL
 * @param {float} angle       初始电角 (rad)
 * @return {*}
 * @author: lkc
 */
void Motor_Hfi_SeedAngle(motor_hfi_t *hfi, float angle);

/**
 * @description: 推进一拍脉振高频注入
 * @detail: 用上一拍 ±Vh 在估计 d 轴产生的 ΔIq 解调凸极误差，校正电角，再翻转极性输出本拍 Vd_h。
 *          首拍只注入不解调。未使能或电压为 0 时不注入、不改角。
 * @param {motor_hfi_t} *hfi                    实例
 * @param {const motor_hfi_config_t} *config    配置
 * @param {float} i_alpha                       定子 α 轴电流 (A)
 * @param {float} i_beta                        定子 β 轴电流 (A)
 * @param {float} v_bus                         母线电压 (V)
 * @param {float} dt                            电流环周期 (s)，须为有限正值才更新速度
 * @param {motor_hfi_output_t} *output          本拍输出；不可为空
 * @return {bool} true=本拍应叠加 vd_inj
 * @author: lkc
 */
bool Motor_Hfi_Step(motor_hfi_t *hfi, const motor_hfi_config_t *config, float i_alpha, float i_beta, float v_bus,
                    float dt, motor_hfi_output_t *output);

/**
 * @description: 读取跟踪电角
 * @param {const motor_hfi_t} *hfi  实例；NULL 返回 0
 * @return {float} 电角 (rad)
 * @author: lkc
 */
float Motor_Hfi_GetAngle(const motor_hfi_t *hfi);

/**
 * @description: 读取估计电角速度
 * @param {const motor_hfi_t} *hfi  实例；NULL 返回 0
 * @return {float} 电角速度 (rad/s)
 * @author: lkc
 */
float Motor_Hfi_GetSpeedE(const motor_hfi_t *hfi);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
