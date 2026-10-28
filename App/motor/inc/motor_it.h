/*
 * @Description: 
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-03-26 20:08:32
 * @FilePath: \foc\App\motor\inc\utils.h
 */
#ifndef _MOTOR_IT_H
#define _MOTOR_IT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*
 *	Run the FOC loop once every N ADC ISR requests. This way the pwm frequency is
 *	detached from the FOC calculation, which because it takes ~25usec it can't work
 *	at >40khz. To set a 100kHz pwm FOC_CONTROL_LOOP_FREQ_DIVIDER can be set at 3
 *	so it skips 2 ISR calls and execute the control loop in the 3rd call.
 *
 *	That ~25 us figure is VESC's measured *execution* time, not the PWM period.
 *	See foc_isr_cycles (DWT) in motor_it.c.
 */
#ifndef FOC_CONTROL_LOOP_FREQ_DIVIDER
#define FOC_CONTROL_LOOP_FREQ_DIVIDER	1
#endif

/* Last DMA-ISR durations, in CPU cycles (168 MHz: cycles/168 = microseconds).
 * foc_isr_cycles is the V0 sample that actually runs the FOC body.
 * foc_isr_cycles_v7 is the V7 sample, which returns immediately on low-side shunts.
 * foc_isr_cycles_max is the high-water mark of the V0 body. */
extern volatile uint32_t foc_isr_cycles;
extern volatile uint32_t foc_isr_cycles_v7;
extern volatile uint32_t foc_isr_cycles_max;

// Current ADC to amperes factor V_REG输入3.3v电  采样电阻CURRENT_SHUNT_RES
//#define FAC_CURRENT					((V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Motor_DmaInt(void *p, uint32_t flags);
#ifdef __cplusplus
}
#endif

#endif
