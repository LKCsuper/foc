/*
 * @Description: 
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-02-19 22:23:08
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
 */
#ifndef FOC_CONTROL_LOOP_FREQ_DIVIDER
#define FOC_CONTROL_LOOP_FREQ_DIVIDER	1
#endif

// Current ADC to amperes factor V_REG输入3.3v电  采样电阻CURRENT_SHUNT_RES
#define FAC_CURRENT					((V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Motor_DmaInt(void *p, uint32_t flags);
#ifdef __cplusplus
}
#endif

#endif
