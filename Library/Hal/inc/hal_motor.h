/*
 * @Description: 电机功率级硬件抽象（PWM / 采样时基）
 * @Author: lkc
 */
#ifndef _HAL_MOTOR_H
#define _HAL_MOTOR_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 启动三相 PWM 输出（模式已配置，不改占空比）
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmStart(void);

/**
 * @description: 停止三相 PWM 输出
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmStop(void);

/**
 * @description: 设置三相 PWM 比较值
 * @param {uint32_t} duty_a  A 相比较值，范围 [0, period]
 * @param {uint32_t} duty_b  B 相比较值，范围 [0, period]
 * @param {uint32_t} duty_c  C 相比较值，范围 [0, period]
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmSetDuty(uint32_t duty_a, uint32_t duty_b, uint32_t duty_c);

/**
 * @description: 提交 PWM 更新（如 STM32 的 COM 事件）；无此需求的平台可为空操作
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmCommit(void);

/**
 * @description: 输出安全中点占空比并提交（三相各 period/2）
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmSetSafe(void);

/**
 * @description: 仅使能指定相的互补 PWM（偏移校准时逐相采样用）
 * @param {uint8_t} phase  相序号：1=A，2=B，3=C
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_EnablePhase(uint8_t phase);

/**
 * @description: 读取 PWM 周期计数（通常等于 ARR）
 * @return {uint32_t} 周期计数值
 * @author: lkc
 */
uint32_t Hal_Motor_PwmPeriodCounts(void);

/**
 * @description: 估算 ADC/控制采样触发频率
 * @detail: 中心对齐 PWM 顶底各触发一次时约为 PWM 频率的 2 倍
 * @return {float} 采样频率 (Hz)；未初始化时返回 0
 * @author: lkc
 */
float Hal_Motor_AdcSampleRateHz(void);

/**
 * @description: 读取微秒时基计数
 * @return {uint32_t} 当前时间 (us)
 * @author: lkc
 */
uint32_t Hal_Motor_TimeUs(void);

/**
 * @description: 计算自 start_us 以来流逝的时间
 * @param {uint32_t} start_us  起始时刻 (us)，通常来自 Hal_Motor_TimeUs
 * @return {float} 流逝时间 (s)，支持无符号溢出回绕
 * @author: lkc
 */
float Hal_Motor_TimeElapsedSec(uint32_t start_us);

/**
 * @description: 读取单个 ADC 通道原始值
 * @param {uint8_t} channel  ADC 缓冲下标
 * @return {uint16_t} 原始计数；通道越界返回 0
 * @author: lkc
 */
uint16_t Hal_Motor_AdcRaw(uint8_t channel);

/**
 * @description: 拷贝 ADC DMA 缓冲到调用方数组
 * @param {uint16_t} *dst  目标缓冲
 * @param {uint32_t} len   最多拷贝通道数
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_AdcGetRaw(uint16_t *dst, uint32_t len);

/**
 * @description: 读取相电流 ADC 原始值（含分流极性处理）
 * @param {uint8_t} phase  0=A，1=B，2=C
 * @return {uint16_t} ADC 计数
 * @author: lkc
 */
uint16_t Hal_Motor_CurrentAdc(uint8_t phase);

/**
 * @description: 读取相电压采样点电压
 * @param {uint8_t} phase  0=SENS1，1=SENS2，2=SENS3
 * @return {float} 采样点电压 (V)
 * @author: lkc
 */
float Hal_Motor_SenseVolts(uint8_t phase);

/**
 * @description: 读取直流母线电压
 * @return {float} 母线电压 (V)
 * @author: lkc
 */
float Hal_Motor_BusVoltage(void);

/**
 * @description: 电流 ADC 计数到安培的换算系数
 * @return {float} 比例 (A/count)
 * @author: lkc
 */
float Hal_Motor_CurrentScale(void);

/**
 * @description: 相电压采样点到相电压的增益
 * @detail: 含分压网络与 ADC_VOLTS_PH_FACTOR
 * @return {float} 无量纲增益
 * @author: lkc
 */
float Hal_Motor_PhaseVoltageGain(void);

/**
 * @description: 是否为三电阻采样硬件
 * @return {bool} true 表示三相分流齐全，可用完整 Clarke
 * @author: lkc
 */
bool Hal_Motor_HasThreeShunts(void);

#ifdef __cplusplus
}
#endif
#endif
