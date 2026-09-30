/*
 * @Description: C2208 编码器机械角抗齿槽学习与补偿
 * @Author: lkc
 *
 * 84 桶对应 12 槽 14 极电机每机械圈的齿槽周期。
 */
#ifndef MOTOR_ANTICOGGING_H
#define MOTOR_ANTICOGGING_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
/* Private define ------------------------------------------------------------*/
/* 12 槽与 14 极的最小公倍数，一圈一个齿槽周期一桶。 */
#define MOTOR_ANTICOGGING_BINS (84U)
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 齿槽学习与补偿表的只读状态
 */
typedef struct {
    bool learning;           /* true=正在按桶采样 */
    bool calibrated;         /* true=全表已收成零均值补偿 */
    bool enabled;            /* true=GetVoltage 输出补偿 */
    uint16_t target_samples; /* 每桶目标样本数 */
    uint16_t min_samples;    /* 各桶已有样本的最小值 */
    uint32_t total_samples;  /* 已计入的样本总数 */
    float min_voltage;       /* 表内最小补偿电压 (V) */
    float max_voltage;       /* 表内最大补偿电压 (V) */
} motor_anticogging_status_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 清零齿槽表和学习状态。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_Init(void);

/**
 * @description: 清空旧表并开始按桶学习齿槽电压。
 * @detail: 学习期间补偿关闭。每个桶要被转子重新进入达到 samples_per_bin 次才算完成。
 * @param {uint16_t} samples_per_bin  每桶目标样本数，允许 1..50
 * @return {bool} true=已开始；样本数超出范围返回 false，状态不变
 * @author: lkc
 */
bool Motor_Anticogging_Start(uint16_t samples_per_bin);

/**
 * @description: 停止学习，不清除已有表，也不把未完成的表标成已标定。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_Stop(void);

/**
 * @description: 清零齿槽表、样本计数和学习/标定/使能标志。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_Clear(void);

/**
 * @description: 打开或关闭齿槽电压补偿。
 * @detail: 未标定完成时不能打开，原使能状态保持不变。关闭不要求已标定。
 * @param {bool} enabled  true=使用补偿表
 * @return {bool} true=已按请求更新；未标定却要求打开时返回 false
 * @author: lkc
 */
bool Motor_Anticogging_SetEnabled(bool enabled);

/**
 * @description: 在转子进入新桶时记入一拍 q 轴 PI 电压。
 * @detail: 停在同一桶不重复计数。全部桶达到目标样本后收成零均值补偿并自动使能。
 * @param {float} mech_angle_rad  机械角 (rad)
 * @param {float} pi_voltage      本拍速度环给出的 q 轴电压 (V)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_LearnStep(float mech_angle_rad, float pi_voltage);

/**
 * @description: 按机械角插值读取齿槽补偿电压。
 * @detail: 未使能、未标定或角度非有限值时返回 0。
 * @param {float} mech_angle_rad  机械角 (rad)
 * @return {float} q 轴补偿电压 (V)
 * @author: lkc
 */
float Motor_Anticogging_GetVoltage(float mech_angle_rad);

/**
 * @description: 复制学习进度和补偿表电压范围。
 * @param {motor_anticogging_status_t *} status  输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Anticogging_GetStatus(motor_anticogging_status_t *status);

#ifdef __cplusplus
}
#endif
#endif
