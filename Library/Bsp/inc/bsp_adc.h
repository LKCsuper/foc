/*
 * @Description: 
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-02-22 22:03:15
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
/* 通道个数 */
#define HW_ADC_CHANNELS			15

/* adc额外通道 */
#define HW_ADC_CHANNELS_EXTRA   0

/* 注入通道个数 注入通道是抢占的 */
#define HW_ADC_INJ_CHANNELS		3
/* adc 一次性转换通道个数  */
#define HW_ADC_NBR_CONV			5
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern volatile uint16_t ADC_Value[HW_ADC_CHANNELS + HW_ADC_CHANNELS_EXTRA];
extern volatile float ADC_curr_norm_value[6];

// Current ADC to amperes factor
#define FAC_CURRENT					((V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Bsp_Adc_Sample(void);
void Bsp_Adc_SetChannels(void);
void Bsp_Adc_Init(void);

#ifdef __cplusplus
}
#endif

#endif
