/*
 * @Description: adc初始化
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-08-27 23:58:10
 * @FilePath: \foc\Library\Bsp\bsp_adc.c
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 默认初始化为0 */
volatile uint16_t ADC_Value[HW_ADC_CHANNELS + HW_ADC_CHANNELS_EXTRA] = {0};
volatile float ADC_curr_norm_value[6] = {0};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 通用状态寄存器
 * @return {STATIC VOID} 本次计算或查询的结果
 * @author: lkc
 */
STATIC VOID Bsp_Adc123_Common(VOID)
{
	/* 初始化adc时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);
	
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	/* 目前adc运行42mhz 但是数据手册是36,但是可行 */
	/* 三重同步模式 3个连续的DMA请求 */
	ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
	/* 2分频 */ 
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	/* 用于三重规则同时模式 */
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	/* 采样周期 5 */
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	return;
}

/**
 * @description: ADC1/2/3 规则转换初始化（分辨率/扫描/外部触发）
 * @return {STATIC VOID} 本次计算或查询的结果
 * @author: lkc
 */
STATIC VOID Bsp_Adc123_Init(VOID)
{
	ADC_InitTypeDef ADC_InitStructure;

	/* 分辨率 12 位 */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	/* 扫描模式 用于多通道 */
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	/* 单次进行还是连续进行 */
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	/* 外部触发 下降沿 */
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
	/* 触发源 定时器2 通道2 */
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	/* 右对齐方式 从低位数据开始处理 */
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	/* adc 一次性转换个数 */
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;

	ADC_Init(ADC1, &ADC_InitStructure);
	/* 这样选择只有adc1才会是外部触发 */
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);

	return;
}

/**
 * @description: adc设置通道
 * @return {void} 无
 * @author: lkc
 */
VOID Bsp_Adc_SetChannels(VOID)
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
 * @description: ADC 采样入口（公共配置 + 三 ADC 初始化 + 通道映射）
 * @Description: adc采样
 * @author: lkc
 * @Date: 2023-02-19 09:22:41
 * @return {void} 无
 */
VOID Bsp_Adc_Sample(VOID)
{
	Bsp_Adc123_Common();
	Bsp_Adc123_Init();
                                                                                             
	// 使能内部温度传感器
	ADC_TempSensorVrefintCmd(ENABLE);
	/* 主从模式adc1 带动adc2 */
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	Bsp_Adc_SetChannels();

	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);

    return;
}

/**
 * @description: ADC 采样初始化入口（采样配置并设置通道）
 * @return {void} 无
 * @author: lkc
 */
VOID Bsp_Adc_Init(VOID)
{
	Bsp_Adc_Sample();
	Bsp_Adc_SetChannels();
	
	return;
}

#ifdef __cplusplus
}
#endif
