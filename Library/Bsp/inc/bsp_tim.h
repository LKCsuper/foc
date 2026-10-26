/*
 * @Description: 定时器板级配置（PWM 时基/触发/编码器）
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 10:03:36
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-19 00:05:36
 */
#ifndef _BSP_TIM_H
#define _BSP_TIM_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEM_CORE_CLOCK							(168000000)
/* TIM5 自由运行时基：APB1 定时器 84 MHz / 84 = 1 MHz，1 tick = 1 us */
#define BSP_TIM_DT_HZ								(1000000.0f)
#define MCPWM_FOC_CURRENT_SAMP_OFFSET				(2) // Offset from timer top for ADC samples

#define HW_PWM1_TIM						(TIM1)
#define HW_PWM1_RCC						(RCC_APB2Periph_TIM1)

#define HW_TRIG_TIM						(TIM2)
#define HE_TRIG_RCC						(RCC_APB1Periph_TIM2)

#define HW_COUNT_TIM					(TIM3)
#define HE_COUNT_RCC					(RCC_APB1Periph_TIM3)

/* TIM4 AB 编码器：PB6/PB7；MT6701 配置 1024 PPR，正交 4 倍频为 4096 count/rev */
#define BSP_ENCODER_TIM                 (TIM4)
#define BSP_ENCODER_COUNTS              (1024U * 4U)

/* 原pwm死区为360  死区时间大约350ns左右 */
#define HW_DEAD_TIME_NSEC				(360.0f)	// Dead time

/*
 * 寄存器写宏仅供 BSP / HAL port 使用。
 * 在对应 .c 中于 #include "includes.h" 之前定义 HAL_INTERNAL。
 */
#ifdef HAL_INTERNAL
#ifdef HW_HAS_3_SHUNTS
#define TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;

#define TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3) \
		TIM8->CR1 |= TIM_CR1_UDIS; \
		TIM8->CCR1 = duty1; \
		TIM8->CCR2 = duty2; \
		TIM8->CCR3 = duty3; \
		TIM8->CR1 &= ~TIM_CR1_UDIS;
#else
#define TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty3; \
		TIM1->CCR3 = duty2; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;
#define TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3) \
		TIM8->CR1 |= TIM_CR1_UDIS; \
		TIM8->CCR1 = duty1; \
		TIM8->CCR2 = duty3; \
		TIM8->CCR3 = duty2; \
		TIM8->CR1 &= ~TIM_CR1_UDIS;
#endif

#define TIMER_UPDATE_SAMP(samp) \
		TIM2->CCR2 = (samp / 2);

#define TIMER_UPDATE_SAMP_TOP_M1(samp, top) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM2->CR1 |= TIM_CR1_UDIS; \
		TIM1->ARR = top; \
		TIM2->CCR2 = samp / 2; \
		TIM1->CR1 &= ~TIM_CR1_UDIS; \
		TIM2->CR1 &= ~TIM_CR1_UDIS;
#define TIMER_UPDATE_SAMP_TOP_M2(samp, top) \
		TIM8->CR1 |= TIM_CR1_UDIS; \
		TIM2->CR1 |= TIM_CR1_UDIS; \
		TIM8->ARR = top; \
		TIM2->CCR2 = samp / 2; \
		TIM8->CR1 &= ~TIM_CR1_UDIS; \
		TIM2->CR1 &= ~TIM_CR1_UDIS;
#endif /* HAL_INTERNAL */

typedef enum{
	MOTOR1 = 0,
	MOTOR2,
}IS_MOTOR;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 启动指定电机三相 PWM 输出
 * @param {BOOL} is_second MOTOR1(0) 或 MOTOR2(1)
 * @return {void} 无
 */
VOID Bsp_Tim_StartPwm(BOOL is_second);

/**
 * @description: 停止指定电机三相 PWM 输出（强制无效电平）
 * @param {BOOL} is_second MOTOR1(0) 或 MOTOR2(1)
 * @return {void} 无
 */
VOID Bsp_Tim_StopPwm(BOOL is_second);

/**
 * @description: 配置定时器1 触发方式（触发 TIM2 采样时基）
 * @return {void} 无
 */
VOID Bsp_Tim1_Trig(VOID);

/**
 * @description: 配置定时器触发 ADC
 * @return {void} 无
 */
VOID Bsp_Tim1_TrigAdc(VOID);

/**
 * @description: 定时器1 PWM 初始化（电机1 主 PWM）
 * @param {ULONG} f_zv PWM 中心频率参数，实际 PWM 频率 = f_zv / 2
 * @return {void} 无
 */
VOID Bsp_Tim_Pwm1(ULONG f_zv);

/**
 * @description: 定时器8 PWM 初始化（电机2 主 PWM）
 * @param {ULONG} f_zv PWM 中心频率参数，实际 PWM 频率 = f_zv / 2
 * @return {void} 无
 */
VOID Bsp_Tim_Pwm2(ULONG f_zv);

/**
 * @description: 定时器初始化入口（PWM + ADC 触发 + 时基）
 * @return {void} 无
 */
VOID Bsp_Tim_Init(VOID);

/**
 * @description: 定时器统计 CPU 运行时间
 * @return {void} 无
 */
VOID Bsp_Tim_RunCount(VOID);

/**
 * @description: 定时器3 中断处理
 * @return {void} 无
 */
VOID Bsp_Tim3_IRQ(VOID);

/**
 * @description: 定时器4 AB 编码器接口初始化
 * @return {void} 无
 */
VOID Bsp_Tim4_EncInit(VOID);

/**
 * @description: 读取 AB 编码器计数值
 * @return {uint16_t} TIM4 CNT
 */
uint16_t Bsp_Tim4_EncGetCount(void);

/**
 * @description: 设置 AB 编码器计数值
 * @param {uint16_t} count 目标计数
 * @return {void} 无
 */
void Bsp_Tim4_EncSetCount(uint16_t count);

/**
 * @description: 定时器1 时基初始化
 * @param {ULONG} f_zv PWM 中心频率参数
 * @return {void} 无
 */
VOID Bsp_Tim1_Base(ULONG f_zv);

/**
 * @description: 定时器1 输出比较通道初始化（PWM 输出配置）
 * @return {void} 无
 */
VOID Bsp_Tim1_OC(VOID);

/**
 * @description: 定时器1 刹车/死区/锁定寄存器配置
 * @return {void} 无
 */
VOID Bsp_Tim1_BDTR(VOID);

/*
定时器详解

一基础

时基单元,分频

对齐模式

 
二输出捕获

三高级功能
*/

/**
 * @description: 读取 TIM5 自由运行时基当前值
 * @return {ULONG} 当前时间 (us)
 */
ULONG Bsp_Tim_DtNow(void);

/**
 * @description: 计算自 time 以来流逝的秒数（支持 CNT 回绕）
 * @param {ULONG} time 起始时刻 (us)
 * @return {float} 流逝时间 (s)
 */
float Bsp_Tim_DtSecondElapsedSince(ULONG time);
#ifdef __cplusplus
}
#endif

#endif
