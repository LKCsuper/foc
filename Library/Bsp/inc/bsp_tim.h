
/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 10:03:36
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-12 21:14:55
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

typedef enum{
	MOTOR1 = 0,
	MOTOR2,
}IS_MOTOR;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Bsp_Tim_StartPwm(bool is_second);
void Bsp_Tim_StopPwm(bool is_second);
void Bsp_Tim_Trig(void);
void Bsp_Tim_TrigAdc(void);
void Bsp_Tim_Pwm1(int f_zv);
void Bsp_Tim_Pwm2(int f_zv);
void Bsp_Tim_Init(void);
void Bsp_Tim3_RunCount(void);
VOID Bsp_Tim3_IRQ(VOID);
void Bsp_Tim3_RunCount(void);

#ifdef __cplusplus
}
#endif

#endif
