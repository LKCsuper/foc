/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-16 22:54:14
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef HW_DEAD_TIME_NSEC
//#define HW_DEAD_TIME_NSEC				(360.0f)	// Dead time
#define HW_DEAD_TIME_NSEC				(360.0f)	// Dead time
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @Description: 停止pwm
 * @author: lkc
 * @Date: 2023-02-19 10:21:06
 * @param {bool} is_second
 * @return {*}
 */
VOID Bsp_Tim_StopPwm(BOOL is_second)
{
    /* dq 为0 */
    if (!is_second)
    {
		/* 强制通道1为低电平 */
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
		/* 使能通道1 */
		TIM_CCxCmd(TIM1, 	 TIM_Channel_1, TIM_CCx_Enable);
		/* 互补失能 */
		TIM_CCxNCmd(TIM1, 	 TIM_Channel_1, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM1, 	 TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, 	 TIM_Channel_2, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM1, 	 TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, 	 TIM_Channel_3, TIM_CCxN_Disable);

		/* 这里应该是再进入一次dma中断,处理状态 */
		TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DUAL_PARALLEL
		TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Disable);

		TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
#endif

#ifdef HW_HAS_DRV8313
		DISABLE_BR();
#endif
		PHASE_FILTER_OFF();
    }
    else
    {
		TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, 	 TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, 	 TIM_Channel_1, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, 	 TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, 	 TIM_Channel_2, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, 	 TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, 	 TIM_Channel_3, TIM_CCxN_Disable);

		TIM_GenerateEvent(TIM8, TIM_EventSource_COM);

#ifdef HW_HAS_DRV8313_2
		DISABLE_BR_2();
#endif

		PHASE_FILTER_OFF_M2();
    }

    return;
}

/**
 * @Description: 开始pwm
 * @author: lkc
 * @Date: 2023-02-19 10:21:06
 * @param {bool} is_second
 * @return {*}
 */
VOID Bsp_Tim_StartPwm(BOOL is_second)
{
	if (!is_second) {
		/* 选择输出pwm1 */
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
		/* 使能通道1 */
		TIM_CCxCmd(TIM1, 	 TIM_Channel_1, TIM_CCx_Enable);
		/* 使能互补通道1 */
		TIM_CCxNCmd(TIM1, 	 TIM_Channel_1, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, 	 TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, 	 TIM_Channel_2, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, 	 TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, 	 TIM_Channel_3, TIM_CCxN_Enable);

#ifdef HW_HAS_DUAL_PARALLEL
		TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Enable);

		PHASE_FILTER_ON_M2();
#endif

		// Generate COM event in ADC interrupt to get better synchronization
		//	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DRV8313
		ENABLE_BR();
#endif
		PHASE_FILTER_ON();
	} else {
		TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, 	 TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, 	 TIM_Channel_1, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, 	 TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, 	 TIM_Channel_2, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, 	 TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, 	 TIM_Channel_3, TIM_CCxN_Enable);

#ifdef HW_HAS_DRV8313_2
		ENABLE_BR_2();
#endif

		PHASE_FILTER_ON_M2();
	}

    return;
}

/**
 * @Description: 电机1pwm输出
 * @author: lkc
 * @Date: 2023-02-19 09:46:46
 * @return {*}
 */
VOID Bsp_Tim_Pwm1(ULONG f_zv)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	/* 初始化定时器时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* 恢复默认 */
	TIM_DeInit(TIM1);
	TIM1->CNT = 0;
    
    /* 定时器基本初始化 */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	/* 中央对齐模式1,交替上下 */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	/* 168000000 / 168 * f_zv */
	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / f_zv);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	/* 上桥臂和下桥臂导通,需要输出互补pwm */
	/* 比较输出使能 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	/* 互补输出使能 */
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	/* 占空比 */
	TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;

	/* 意味着上下桥臂有没有接反 */
#ifndef INVERTED_TOP_DRIVER_INPUT
	/* gpio high = top fets on */
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
#else
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
#endif

    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

	/*  互补通道输出极性 */
#ifndef INVERTED_BOTTOM_DRIVER_INPUT
	/* gpio high = bottom fets on */
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
#else
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
#endif
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	// TODO 这里为什么初始化的是四个通道
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	/* 都是预装载开启 影子寄存器 也就是暂时不生效 */
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Automatic Output enable, Break, dead time and lock configuration
	/* 运行模式下的关闭状态选择 */ 
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	/* 空闲模式下的关闭状态选择 */
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	/* 锁定 */
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	/* 死区时间 避免上下桥臂同时导通现象 */
	TIM_BDTRInitStructure.TIM_DeadTime  =  calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	/* 自动输出使能 */
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

// 故障检测功能
#ifdef HW_USE_BRK
	// Enable BRK function. Hardware will asynchronously stop any PWM activity upon an
	// external fault signal. PWM outputs remain disabled until MCU is reset.
	// software will catch the BRK flag to report the fault code
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
#else
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
#endif

	// 高级功能使能
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	// 使能预装载之后 只有产生COM事件发生后 才能写如影子寄存器
	TIM_CCPreloadControl(TIM1, ENABLE);
	// 预装载是呢个
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM1->CNT = 0;
    TIM_Cmd(TIM1, ENABLE);

	Bsp_Tim_StopPwm(MOTOR1);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIMER_UPDATE_SAMP(MCPWM_FOC_CURRENT_SAMP_OFFSET);

    return;
}

/**
 * @Description: 电机2pwm输出
 * @author: lkc
 * @Date: 2023-02-19 09:47:01
 * @return {*}
 */
VOID Bsp_Tim_Pwm2(ULONG f_zv)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_DeInit(TIM8);
	TIM8->CNT = 0;
    
    /* 定时器基本初始化 */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / f_zv);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
    
    #ifndef INVERTED_TOP_DRIVER_INPUT
	/* gpio high = top fets on */
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
#else
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
#endif

    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

#ifndef INVERTED_BOTTOM_DRIVER_INPUT
	/* gpio high = bottom fets on */
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; 
#else
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
#endif
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime =  calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    
    #ifdef HW_USE_BRK
	// Enable BRK function. Hardware will asynchronously stop any PWM activity upon an
	// external fault signal. PWM outputs remain disabled until MCU is reset.
	// software will catch the BRK flag to report the fault code
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
#else
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
#endif

	TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);
	TIM_CCPreloadControl(TIM8, ENABLE);
	TIM_ARRPreloadConfig(TIM8, ENABLE);

    #ifdef HW_HAS_DUAL_MOTORS
        TIM8->CNT = TIM1->ARR;
    #else
        TIM8->CNT = 0;
    #endif
    TIM_Cmd(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);

    return;
}

/**
 * @description: 触发adc
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim_TrigAdc(VOID)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	//TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    TIM_DeInit(TIM2);
	TIM2->CNT = 0;
    
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 250;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_CCPreloadControl(TIM2, ENABLE);

	// PWM outputs have to be enabled in order to trigger ADC on CCx
	TIM_CtrlPWMOutputs(TIM2, ENABLE);

    TIM_Cmd(TIM2, ENABLE);

    return;
}

/**
 * @Description: 定时器触发方式
 * @author: lkc
 * @Date: 2023-02-19 10:07:08
 * @return {*}
 */
VOID Bsp_Tim_Trig(VOID)
{
    #if defined HW_HAS_DUAL_MOTORS || defined HW_HAS_DUAL_PARALLEL
        // See: https://www.cnblogs.com/shangdawei/p/4758988.html
        TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Enable);
        TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
        TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
        TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Trigger);
        TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Enable);
        TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);
        TIM_SelectInputTrigger(TIM2, TIM_TS_ITR1);
        TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
    #else
        TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
        TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
        TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
        TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
    #endif

    return;
}

/**
 * @description: 定时器统计cpu运行时间
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim3_RunCount(VOID)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* 42000000 / 4200 = 10000 这里大约20khz */
	TIM_TimeBaseStructure.TIM_Period = (4200 / 2) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
	TIM_Cmd(TIM3, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStre;
	NVIC_InitStre.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStre.NVIC_IRQChannelPreemptionPriority = IRQ_TIM3;
    NVIC_InitStre.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStre.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStre);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

	return;
}

/**
 * @description: 定时器初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim_Init(VOID)
{
	/* 多少hz 30000 30hz  */
    Bsp_Tim_Pwm1(MCCONF_FOC_F_ZV);
    Bsp_Tim_TrigAdc();
	Bsp_Tim_Trig();

	// todo 这里其实还有定时器3中断,只不过是写在freertos里边了

	return;
}

/**
 * @description: 定时器3中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
extern ULONG ulFreeRTOSRunTimeTicks;
VOID Bsp_Tim3_IRQ(VOID)
{
	if(SET == TIM_GetITStatus(TIM3, TIM_IT_Update)) //溢出中断
	{
		ulFreeRTOSRunTimeTicks++;
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	return;
}

#ifdef __cplusplus
}
#endif
