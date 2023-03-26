/*
 * @Description: 
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-03-16 22:56:31
 * @FilePath: \foc\Library\Bsp\inc\bsp_adc.h
 */
#ifndef _BSP_ADC_H
#define _BSP_ADC_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern volatile uint16_t ADC_Value[HW_ADC_CHANNELS + HW_ADC_CHANNELS_EXTRA];
extern volatile float ADC_curr_norm_value[6];

// Current ADC to amperes factor
#define FAC_CURRENT					((V_REG / 4095.0f) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
VOID Bsp_Adc_Sample(VOID);
VOID Bsp_Adc_SetChannels(VOID);
VOID Bsp_Adc_Init(VOID);

#ifdef __cplusplus
}
#endif

#endif
