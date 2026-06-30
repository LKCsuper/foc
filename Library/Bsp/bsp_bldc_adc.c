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
 * @description: ADC 公共配置
 * @detail: 配置 ADC1/ADC2/ADC3 共享的多 ADC 模式、时钟分频和 DMA 访问方式
 * @return {*}
 * @author: lkc
 */
STATIC VOID Bsp_Adc_Bldc_CommonInit(VOID)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	/* 初始化 ADC 公共时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

	// ADC Common Init
	// Note that the ADC is running at 42MHz, which is higher than the
	// specified 36MHz in the data sheet, but it works.
	ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;                    // 三重同步模式，ADC1/ADC2/ADC3 同步采样
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;                     // ADC 时钟二分频
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;                // 多 ADC DMA 模式1，从公共数据寄存器搬运结果
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;    // 多 ADC 采样间隔参数，规则同步模式保持最小延迟
	ADC_CommonInit(&ADC_CommonInitStructure);

	return;
}

/**
 * @description: adc1 初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 * 
 * TIM8_CC1 触发一次
    ADC1 扫描一串通道
    ADC2 扫描一串通道
    ADC3 扫描一串通道
 * 
 */
VOID Bsp_Adc1_Bldc_Init(VOID)
{
    /* adc 结构体 */
	ADC_InitTypeDef ADC_InitStructure;

	/* 初始化 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// Channel-specific settings
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                          // 分辨率12位
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                    // 扫描转换模式开启，一次性转换多个通道
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                             // 关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;  // 外部触发模式，ADC_ExternalTrigConv_T8_CC1下降沿
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1;			// 触发源
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                          // 数据对齐，右对齐
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;                        // 一次性转换HW_ADC_NBR_CONV个通道

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
	ADC_InitTypeDef ADC_InitStructure;

	/* 初始化时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	// Channel-specific settings
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                          // 分辨率12位
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                    // 扫描转换模式开启，一次性转换多个通道
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                             // 关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;     // 从 ADC 不单独接外部规则触发，只跟随 ADC1 多 ADC 触发
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                          // 数据对齐，右对齐
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;                        // 一次性转换HW_ADC_NBR_CONV个通道

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
	ADC_InitTypeDef ADC_InitStructure;

	/* 初始化时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	// Channel-specific settings
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                          // 分辨率12位
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                    // 扫描转换模式开启
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                             // 关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;     // 从 ADC 不单独接外部规则触发，只跟随 ADC1 多 ADC 触发
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                          // 数据对齐，右对齐
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;                        // 一次性转换几个

	ADC_Init(ADC3, &ADC_InitStructure);
    
    return;
}

/**
 * @description: adc设置通道
 * @detail: 
 * @return {*}
 * @author: lkc
 * 
0:  IN0   SENS1
1:  IN1   SENS2
2:  IN2   SENS3
3:  IN10  CURR1
4:  IN11  CURR2
5:  IN12  CURR3
6:  IN5   ADC_EXT1
7:  IN6   ADC_EXT2
8:  IN3   TEMP_PCB
9:  IN14  TEMP_MOTOR
10: IN15  Shutdown
11: IN13  AN_IN
12: Vrefint
13: IN0   SENS1
14: IN1   SENS2

三重同步，采样五次
rank1: ADC1 + ADC2 + ADC3
rank2: ADC1 + ADC2 + ADC3
rank3: ADC1 + ADC2 + ADC3
rank4: ADC1 + ADC2 + ADC3
rank5: ADC1 + ADC2 + ADC3

注入组：优先级更高，可以在规则组采样过程中插队
使用注入组进行采样电流，规则组采样电流只是遵循规则，实际应该还是使用注入组的电流
 * 
 */
VOID Bsp_Adc_Bldc_SetChannels(VOID)
{
	// 规则组通道：三重同步模式下，ADC1/ADC2/ADC3 的同一个 rank 会在同一次触发中成组采样。
	// rank1: SENS1/SENS2/SENS3，三相相电压采样。
	// rank2: CURR1/CURR2/CURR3，三相电流采样。
	// rank3: ADC_EXT1/ADC_EXT2/TEMP_PCB，外部 ADC 和 PCB 温度。
	// rank4: TEMP_MOTOR/Shutdown/AN_IN，电机温度、关机检测和模拟输入。
	// rank5: Vrefint/SENS1/SENS2，内部参考电压和两路相电压重复采样。
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 		1, ADC_SampleTime_15Cycles);	// ADC1 rank1: SENS1，A 相相电压
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 		2, ADC_SampleTime_15Cycles);	// ADC1 rank2: CURR1，A 相电流
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 		3, ADC_SampleTime_15Cycles);	// ADC1 rank3: ADC_EXT1，外部模拟输入1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 		4, ADC_SampleTime_15Cycles);	// ADC1 rank4: TEMP_MOTOR，电机温度
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);	// ADC1 rank5: Vrefint，内部参考电压

	ADC_RegularChannelConfig(ADC2, ADC_Channel_1,  1, ADC_SampleTime_15Cycles);	// ADC2 rank1: SENS2，B 相相电压
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);	// ADC2 rank2: CURR2，B 相电流
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6,  3, ADC_SampleTime_15Cycles);	// ADC2 rank3: ADC_EXT2，外部模拟输入2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);	// ADC2 rank4: Shutdown，关机/按键采样
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0,  5, ADC_SampleTime_15Cycles);	// ADC2 rank5: SENS1，相电压重复采样

	ADC_RegularChannelConfig(ADC3, ADC_Channel_2,  1, ADC_SampleTime_15Cycles);	// ADC3 rank1: SENS3，C 相相电压
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);	// ADC3 rank2: CURR3，C 相电流
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3,  3, ADC_SampleTime_15Cycles);	// ADC3 rank3: TEMP_PCB，PCB 温度
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);	// ADC3 rank4: AN_IN，普通模拟输入
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1,  5, ADC_SampleTime_15Cycles);	// ADC3 rank5: SENS2，相电压重复采样

	// 注入组通道：用于电流快速采样。注入转换优先级高，可被定时器在 PWM 周期指定位置触发。
	// 这里 3 个 rank 都采同一组三相电流，便于在一个周期内多点采样或后续取平均/择优使用。
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);	// ADC1 injected rank1: CURR1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);	// ADC2 injected rank1: CURR2
#ifdef HW_HAS_3_SHUNTS
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);	// ADC3 injected rank1: CURR3
#endif

	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);	// ADC1 injected rank2: CURR1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);	// ADC2 injected rank2: CURR2
#ifdef HW_HAS_3_SHUNTS
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);	// ADC3 injected rank2: CURR3
#endif

	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);	// ADC1 injected rank3: CURR1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);	// ADC2 injected rank3: CURR2
#ifdef HW_HAS_3_SHUNTS
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);	// ADC3 injected rank3: CURR3
#endif

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
	Bsp_Adc_Bldc_CommonInit();
	Bsp_Adc1_Bldc_Init();
	Bsp_Adc2_Bldc_Init();
	Bsp_Adc3_Bldc_Init();

	// Enable Vrefint channel 开启内部参考电压通道
	ADC_TempSensorVrefintCmd(ENABLE);

	// Enable DMA request after last transfer (Multi-ADC mode) 让三重 ADC + DMA 在每次转换序列结束后还能持续发 DMA 请求，保证周期性采样不断流
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	// Injected channels for current measurement at end of cycle
	// ADC1的注入触发。由ADC_ExternalTrigInjecConv_T1_CC4触发
	// ADC2的注入触发。由ADC_ExternalTrigInjecConv_T8_CC2触发
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);
	ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T8_CC2);

	// 是否开启ADC3的注入触发，但是一般ADC1 和 ADC2可以计算出ADC3，目前推测有的话计算offset更准确
	// 三电阻采样硬件存在时开启 ADC3 注入触发，直接采 CURR3。
	// 两电阻采样时可不启用 ADC3，第三相电流可由 Ic = -Ia - Ib 计算。
#ifdef HW_HAS_3_SHUNTS
	ADC_ExternalTrigInjectedConvConfig(ADC3, ADC_ExternalTrigInjecConv_T8_CC3);
#endif

	// 外部下降沿触发
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Falling);
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_Falling);
#ifdef HW_HAS_3_SHUNTS
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC3, ADC_ExternalTrigInjecConvEdge_Falling);
#endif

	// 注入通道个数
	ADC_InjectedSequencerLengthConfig(ADC1, HW_ADC_INJ_CHANNELS);
	ADC_InjectedSequencerLengthConfig(ADC2, HW_ADC_INJ_CHANNELS);
#ifdef HW_HAS_3_SHUNTS
	ADC_InjectedSequencerLengthConfig(ADC3, HW_ADC_INJ_CHANNELS);
#endif

	// 设置规则组和注入组通道
	Bsp_Adc_Bldc_SetChannels();

	// 打开 ADC1 的注入组转换完成中断
	ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
	nvicEnableVector(ADC_IRQn, 6);

	ADC_Cmd(ADC1, ENABLE); // Enable ADC1
	ADC_Cmd(ADC2, ENABLE); // Enable ADC2
	ADC_Cmd(ADC3, ENABLE); // Enable ADC3
	
    return;
}


#ifdef __cplusplus
}
#endif


