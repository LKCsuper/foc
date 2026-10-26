/*
 * @Description: ADC 采样板级接口（采样结果数组与换算系数）
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
/**
 * @description: ADC 采样入口（公共配置 + 三 ADC 初始化 + 通道映射）
 * @return {void} 无
 */
VOID Bsp_Adc_Sample(VOID);

/**
 * @description: 配置三 ADC 的规则组与注入组通道
 * @return {void} 无
 */
VOID Bsp_Adc_SetChannels(VOID);

/**
 * @description: ADC 初始化入口（采样配置并设置通道）
 * @return {void} 无
 */
VOID Bsp_Adc_Init(VOID);

#ifdef __cplusplus
}
#endif

#endif
