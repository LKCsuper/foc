/*
 * @Description: 
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
#define MCPWM_FOC_CURRENT_SAMP_OFFSET				(2) // Offset from timer top for ADC samples

#define HW_PWM1_TIM						(TIM1)
#define HW_PWM1_RCC						(RCC_APB2Periph_TIM1)

#define HW_TRIG_TIM						(TIM2)
#define HE_TRIG_RCC						(RCC_APB1Periph_TIM2)

#define HW_COUNT_TIM					(TIM3)
#define HE_COUNT_RCC					(RCC_APB1Periph_TIM3)

/* TIM4 AB 编码器：PB6/PB7，1024 线 × 4 倍频 */
#define BSP_ENCODER_TIM                 (TIM4)
#define BSP_ENCODER_COUNTS              (1024U * 4U)

/* 原pwm死区为360  死区时间大约350ns左右 */
#define HW_DEAD_TIME_NSEC				(360.0f)	// Dead time

/*
 * 寄存器写宏仅供 BSP / HAL port 使用。
 * 在对应 .c 中于 #include "includes.h" 之前定义 HAL_INTERNAL。
 */
#ifdef HAL_INTERNAL
#ifndef HW_PWM_PHASE_BC_SWAP
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
VOID Bsp_Tim_StartPwm(BOOL is_second);
VOID Bsp_Tim_StopPwm(BOOL is_second);
VOID Bsp_Tim1_Trig(VOID);
VOID Bsp_Tim1_TrigAdc(VOID);
VOID Bsp_Tim_Pwm1(ULONG f_zv);
VOID Bsp_Tim_Pwm2(ULONG f_zv);
VOID Bsp_Tim_Init(VOID);
VOID Bsp_Tim_RunCount(VOID);
VOID Bsp_Tim3_IRQ(VOID);
VOID Bsp_Tim4_EncInit(VOID);
uint16_t Bsp_Tim4_EncGetCount(void);
void Bsp_Tim4_EncSetCount(uint16_t count);

VOID Bsp_Tim1_Base(ULONG f_zv);
VOID Bsp_Tim1_OC(VOID);
VOID Bsp_Tim1_BDTR(VOID);

/*
定时器详解

一基础

时基单元,分频

对齐模式

 
二输出捕获

三高级功能
*/

ULONG Bsp_Tim_DtNow(void);
float Bsp_Tim_DtSecondElapsedSince(ULONG time);
#ifdef __cplusplus
}
#endif

#endif
