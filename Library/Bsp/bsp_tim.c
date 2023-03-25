/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-25 18:21:55
 * @detail: 逻辑分析仪 死区时间350ns左右 一个周期66.6us
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
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 定时器1 base
 * @detail: 对齐方式中央对齐 频率SYSTEM_CORE_CLOCK / f_zv
 * @return {*}
 * @author: lkc
 */
STATIC VOID Bsp_Tim1_Base(ULONG f_zv)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* 初始化定时器时钟 */
    RCC_APB2PeriphClockCmd(HW_PWM1_RCC, ENABLE);

	/* 恢复默认 */
	TIM_DeInit(HW_PWM1_TIM);
	HW_PWM1_TIM->CNT = 0;

    /* 定时器基本初始化 */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	/* 中央对齐模式1,交替上下 */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	/* 168000000 / 168 * f_zv */
	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / f_zv);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(HW_PWM1_TIM, &TIM_TimeBaseStructure);

	return;
}

/**
 * @description: 定时器1 捕获输出
 * @detail: 互补pwm 占空比 输出极性
 * @return {*}
 * @author: lkc
 */
STATIC VOID Bsp_Tim1_OC(VOID)
{
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	/* 上桥臂和下桥臂导通,需要输出互补pwm */
	/* 比较输出使能 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	/* 互补输出使能 */
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	/* 占空比 */
	TIM_OCInitStructure.TIM_Pulse = HW_PWM1_TIM->ARR / 2;

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
    TIM_OC1Init(HW_PWM1_TIM, &TIM_OCInitStructure);
	TIM_OC2Init(HW_PWM1_TIM, &TIM_OCInitStructure);
	TIM_OC3Init(HW_PWM1_TIM, &TIM_OCInitStructure);
	TIM_OC4Init(HW_PWM1_TIM, &TIM_OCInitStructure);

	/* 都是预装载开启 影子寄存器 也就是暂时不生效 */
	TIM_OC1PreloadConfig(HW_PWM1_TIM, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(HW_PWM1_TIM, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(HW_PWM1_TIM, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(HW_PWM1_TIM, TIM_OCPreload_Enable);

	return;
}

/**
 * @description: 定时器1 高级功能
 * @detail: 死区设置
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim1_BDTR(VOID)
{
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	/* Automatic Output enable, Break, dead time and lock configuration */
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

	/* 高级功能使能 */
	TIM_BDTRConfig(HW_PWM1_TIM, &TIM_BDTRInitStructure);
	/* 使能预装载之后 只有产生COM事件发生后 才能写如影子寄存器 */
	TIM_CCPreloadControl(HW_PWM1_TIM, ENABLE);
	/* 预装载,先保存更新时写入 */
	TIM_ARRPreloadConfig(HW_PWM1_TIM, ENABLE);

	return;
}

/**
 * @description: 定时器pwm1
 * @detail: 
 * @return {*}
 * @author: lkc
 */
STATIC VOID Bsp_Tim_Pwm1(ULONG f_zv)
{
	Bsp_Tim1_Base(f_zv);
	Bsp_Tim1_OC();
	Bsp_Tim1_BDTR();

	/* 开启定时器 */
	HW_PWM1_TIM->CNT = 0;
    TIM_Cmd(HW_PWM1_TIM, ENABLE);

	/* 停止pwm */
	Bsp_Tim_StopPwm(MOTOR1);

	/* 输出pwm */
	// TODO 必须有这一句才会输出pwm,默认pwm引脚全是低电平
    TIM_CtrlPWMOutputs(HW_PWM1_TIM, ENABLE);

	/* */
	TIMER_UPDATE_SAMP(MCPWM_FOC_CURRENT_SAMP_OFFSET);
	return;
}

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
		TIM_SelectOCxM(HW_PWM1_TIM,  TIM_Channel_1, TIM_ForcedAction_InActive);
		/* 使能通道1 */
		TIM_CCxCmd(HW_PWM1_TIM, 	 TIM_Channel_1, TIM_CCx_Enable);
		/* 互补失能 */
		TIM_CCxNCmd(HW_PWM1_TIM, 	 TIM_Channel_1, TIM_CCxN_Disable);

		TIM_SelectOCxM(HW_PWM1_TIM,  TIM_Channel_2, TIM_ForcedAction_InActive);
		TIM_CCxCmd(HW_PWM1_TIM, 	 TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(HW_PWM1_TIM, 	 TIM_Channel_2, TIM_CCxN_Disable);

		TIM_SelectOCxM(HW_PWM1_TIM,  TIM_Channel_3, TIM_ForcedAction_InActive);
		TIM_CCxCmd(HW_PWM1_TIM, 	 TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(HW_PWM1_TIM, 	 TIM_Channel_3, TIM_CCxN_Disable);

		/* 更新影子寄存器,处理状态 */
		TIM_GenerateEvent(HW_PWM1_TIM, TIM_EventSource_COM);

		//PRINTF("Stop \n");
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

// #ifdef HW_HAS_DRV8313
// 		DISABLE_BR();
// #endif
// 		PHASE_FILTER_OFF();
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

// #ifdef HW_HAS_DRV8313_2
// 		DISABLE_BR_2();
// #endif

// 		PHASE_FILTER_OFF_M2();
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
		TIM_SelectOCxM(HW_PWM1_TIM,  TIM_Channel_1, TIM_OCMode_PWM1);
		/* 使能通道1 */
		TIM_CCxCmd(HW_PWM1_TIM, 	 TIM_Channel_1, TIM_CCx_Enable);
		/* 使能互补通道1 */
		TIM_CCxNCmd(HW_PWM1_TIM, 	 TIM_Channel_1, TIM_CCxN_Enable);

		TIM_SelectOCxM(HW_PWM1_TIM,  TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(HW_PWM1_TIM, 	 TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(HW_PWM1_TIM, 	 TIM_Channel_2, TIM_CCxN_Enable);

		TIM_SelectOCxM(HW_PWM1_TIM,  TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(HW_PWM1_TIM, 	 TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(HW_PWM1_TIM, 	 TIM_Channel_3, TIM_CCxN_Enable);

		PRINTF("Start \n");
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

		/* Generate COM event in ADC interrupt to get better synchronization */
		/* 只有产生com事件才会生效,因为预先写入影子寄存器,需要触发写入
			后续这里可能在dma中断生效9
		 */
		TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

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
        TIM8->CNT = HW_PWM1_TIM->ARR;
    #else
        TIM8->CNT = 0;
    #endif
    TIM_Cmd(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);

    return;
}

/**
 * @description: 定时器base
 * @detail: 
 * @return {*}
 * @author: lkc
 */
STATIC VOID Bsp_Tim2_Base(VOID)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(HE_TRIG_RCC, ENABLE);

	TIM_DeInit(HW_TRIG_TIM);
	HW_TRIG_TIM->CNT = 0;
    
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(HW_TRIG_TIM, &TIM_TimeBaseStructure);

	return;
}

/**
 * @description: 定时器2 输出捕获
 * @detail: 
 * @return {*}
 * @author: lkc
 */
STATIC VOID Bsp_Tim2_OC(VOID)
{
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 250;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	
	TIM_OC1Init(HW_TRIG_TIM, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(HW_TRIG_TIM, TIM_OCPreload_Enable);
	TIM_OC2Init(HW_TRIG_TIM, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(HW_TRIG_TIM, TIM_OCPreload_Enable);
	TIM_OC3Init(HW_TRIG_TIM, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(HW_TRIG_TIM, TIM_OCPreload_Enable);

	return;
}

/**
 * @description: 定时器2中断初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim2_NVIC(VOID)
{
#if 0
	NVIC_InitTypeDef NVIC_InitStre;
	NVIC_InitStre.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStre.NVIC_IRQChannelPreemptionPriority = IRQ_TIM2;
    NVIC_InitStre.NVIC_IRQChannelSubPriority = 6;
    NVIC_InitStre.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStre);

	// Enable CC2 interrupt, which will be fired in V0 and V7
	TIM_ITConfig(HW_TRIG_TIM, TIM_IT_CC2, ENABLE);
	TIM_ClearITPendingBit(HW_TRIG_TIM, TIM_IT_CC2);
#endif
	return;
}

/**
 * @description: 定时器2输出pwm
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim2_Pwm(VOID)
{
	Bsp_Tim2_Base();
	Bsp_Tim2_OC();
	Bsp_Tim2_NVIC();

    TIM_ARRPreloadConfig(HW_TRIG_TIM, ENABLE);
	TIM_CCPreloadControl(HW_TRIG_TIM, ENABLE);
	// PWM outputs have to be enabled in order to trigger ADC on CCx
	TIM_CtrlPWMOutputs(HW_TRIG_TIM, ENABLE);

    TIM_Cmd(HW_TRIG_TIM, ENABLE);

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
	/* pwm触发adc */
	Bsp_Tim2_Pwm();

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
        TIM_SelectOutputTrigger(HW_PWM1_TIM, TIM_TRGOSource_Enable);
        TIM_SelectMasterSlaveMode(HW_PWM1_TIM, TIM_MasterSlaveMode_Enable);
        TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
        TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Trigger);
        TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Enable);
        TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);
        TIM_SelectInputTrigger(HW_TRIG_TIM, TIM_TS_ITR1);
        TIM_SelectSlaveMode(HW_TRIG_TIM, TIM_SlaveMode_Reset);
    #else
		/*
			如何触发,定时器1
		*/
		/* 更新事件触发外部 */
        TIM_SelectOutputTrigger(HW_PWM1_TIM, TIM_TRGOSource_Update);
		/* 从模式开启 */
        TIM_SelectMasterSlaveMode(HW_PWM1_TIM, TIM_MasterSlaveMode_Enable);
		/* 触发的输入源 TIM_TS_ITR0定时器的触发源为定时器1  */
        TIM_SelectInputTrigger(HW_TRIG_TIM, TIM_TS_ITR0);
		/* 触发之后定时器2,reset count */
        TIM_SelectSlaveMode(HW_TRIG_TIM, TIM_SlaveMode_Reset);
    #endif

    return;
}


VOID Bsp_Tim3_Base(VOID)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(HE_COUNT_RCC, ENABLE);

	/* 42000000 / 4200 = 10000 这里大约20khz */
	TIM_TimeBaseStructure.TIM_Period = (4200 / 2) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(HW_COUNT_TIM, &TIM_TimeBaseStructure); 
	TIM_Cmd(HW_COUNT_TIM, ENABLE);

	return;
}

VOID Bsp_Tim3_NVIC(VOID)
{
	NVIC_InitTypeDef NVIC_InitStre;
	NVIC_InitStre.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStre.NVIC_IRQChannelPreemptionPriority = IRQ_TIM3;
    NVIC_InitStre.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStre.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStre);

	TIM_ITConfig(HW_COUNT_TIM, TIM_IT_Update, ENABLE);
	TIM_ClearITPendingBit(HW_COUNT_TIM, TIM_IT_Update);

	return;
}

/**
 * @description: 定时器统计cpu运行时间
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim_RunCount(VOID)
{
	Bsp_Tim3_Base();
	Bsp_Tim3_NVIC();

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
	Bsp_Tim_RunCount();
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
	if(SET == TIM_GetITStatus(HW_COUNT_TIM, TIM_IT_Update)) //溢出中断
	{
		ulFreeRTOSRunTimeTicks++;
	}
	TIM_ClearITPendingBit(HW_COUNT_TIM, TIM_IT_Update);
	return;
}

/**
 * @description: 定时器2中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim2_IRQ(VOID)
{
	if(SET == TIM_GetITStatus(HW_TRIG_TIM, TIM_IT_CC2)) //溢出中断
	{
		ulFreeRTOSRunTimeTicks++;
	}
	TIM_ClearITPendingBit(HW_TRIG_TIM, TIM_IT_CC2);
	return;
}

#ifdef __cplusplus
}
#endif
