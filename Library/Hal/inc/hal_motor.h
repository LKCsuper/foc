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

#ifdef __cplusplus
}
#endif
#endif
