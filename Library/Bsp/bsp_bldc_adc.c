/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-04 16:26:48
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-04 20:11:13
 * @detail: 
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: adc1 初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Adc1_Bldc_Init(VOID)
{
    /* adc 结构体 */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* 初始化 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// ADC Common Init
	// Note that the ADC is running at 42MHz, which is higher than the
	// specified 36MHz in the data sheet, but it works.
	ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;                    /* 三重同步模式 */
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;                     /* 二分频 */
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;                /* dma模式1 用于三重同步模式 */
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;    /* 几个时钟采样 */
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Channel-specific settings
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                          /* 分辨率12位 */
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                    /* 扫描转换模式开启 */
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                             /* 关闭连续转换 */
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;  /* 外部触发模式，下降沿 */
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                          /* 数据对齐，右对齐 */
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;                        /* 一次性转换几个 */

	ADC_Init(ADC1, &ADC_InitStructure);

    return;
}

/**
 * @description: adc2初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Adc2_Bldc_Init(VOID)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* 初始化时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	// ADC Common Init
	// Note that the ADC is running at 42MHz, which is higher than the
	// specified 36MHz in the data sheet, but it works.
	ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;                    /* 三重同步模式 */
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;                     /* 二分频 */
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;                /* dma模式1 用于三重同步模式 */
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;    /* 几个时钟采样 */
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Channel-specific settings
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                          /* 分辨率12位 */
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                    /* 扫描转换模式开启 */
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                             /* 关闭连续转换 */
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;  /* 外部触发模式，下降沿 */
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                          /* 数据对齐，右对齐 */
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;                        /* 一次性转换几个 */

	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;     /* 触发模式关闭 */
	ADC_InitStructure.ADC_ExternalTrigConv = 0;                                     /* 转换为0 */
	ADC_Init(ADC2, &ADC_InitStructure);

    return;
}

/**
 * @description: adc3初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Adc3_Bldc_Init(VOID)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* 初始化时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	// ADC Common Init
	// Note that the ADC is running at 42MHz, which is higher than the
	// specified 36MHz in the data sheet, but it works.
	ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;                    /* 三重同步模式 */
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;                     /* 二分频 */
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;                /* dma模式1 用于三重同步模式 */
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;    /* 几个时钟采样 */
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Channel-specific settings
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                          /* 分辨率12位 */
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                    /* 扫描转换模式开启 */
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                             /* 关闭连续转换 */
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;  /* 外部触发模式，下降沿 */
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                          /* 数据对齐，右对齐 */
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;                        /* 一次性转换几个 */

	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;     /* 触发模式关闭 */
	ADC_InitStructure.ADC_ExternalTrigConv = 0;                                     /* 转换为0 */
	ADC_Init(ADC3, &ADC_InitStructure);
    
    return;
}

/**
 * @description: adc设置通道
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Adc_Bldc_SetChannels(VOID)
{
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 		1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 		2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 		3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 		4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1,  1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6,  3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0,  5, ADC_SampleTime_15Cycles);

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2,  1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3,  3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1,  5, ADC_SampleTime_15Cycles);

	// Injected channels 注入通道一般是直接抢占
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);

	return;
}

/**
 * @description: 
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Adc_Bldc_Init(VOID)
{
	// Enable Vrefint channel
	ADC_TempSensorVrefintCmd(ENABLE);

	// Enable DMA request after last transfer (Multi-ADC mode)
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	// Injected channels for current measurement at end of cycle
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);
	ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T8_CC2);
#ifdef HW_HAS_3_SHUNTS
	ADC_ExternalTrigInjectedConvConfig(ADC3, ADC_ExternalTrigInjecConv_T8_CC3);
#endif
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Falling);
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_Falling);
#ifdef HW_HAS_3_SHUNTS
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC3, ADC_ExternalTrigInjecConvEdge_Falling);
#endif
	ADC_InjectedSequencerLengthConfig(ADC1, HW_ADC_INJ_CHANNELS);
	ADC_InjectedSequencerLengthConfig(ADC2, HW_ADC_INJ_CHANNELS);
#ifdef HW_HAS_3_SHUNTS
	ADC_InjectedSequencerLengthConfig(ADC3, HW_ADC_INJ_CHANNELS);
#endif

	Bsp_Adc_Bldc_SetChannels();

	// Interrupt
	ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
	nvicEnableVector(ADC_IRQn, 6);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// Enable ADC2
	ADC_Cmd(ADC2, ENABLE);

	// Enable ADC3
	ADC_Cmd(ADC3, ENABLE);
    return;
}


#ifdef __cplusplus
}
#endif


