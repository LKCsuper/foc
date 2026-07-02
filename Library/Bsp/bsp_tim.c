/*
 * @Description: 定时器初始化 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-09-30 20:36:17
 * @detail: 逻辑分析仪 死区时间350ns左右 一个周期66.6us 15khz
 * 
 * 
 * 
 * 	TIM_Period 重载值 TIM_Prescaler 预分频值
 * 
 * 
 * 
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
 * @description: 定时器1 时基初始化（高级定时器，用于电机PWM输出）
 * @detail:
 *   - 对齐方式：中央对齐模式1（CenterAligned1），计数器先向上计数到ARR，再向下计数到0
 *   - PWM频率：f_pwm = SYSTEM_CORE_CLOCK / (2 * TIM_Period)，TIM_Period = SYSTEM_CORE_CLOCK / f_zv
 *     因此 f_pwm = f_zv / 2，传入 f_zv 为期望 PWM 中心频率的 2 倍
 *   - 预分频器为 0，定时器时钟 = 系统核心时钟（168MHz）
 *   - 时钟分频 CKD_DIV1，死区时钟与定时器时钟相同
 *   - 重复计数器为 0，每次计数器溢出立即产生更新事件（UEV）
 * @param {ULONG} f_zv - PWM中心频率参数，实际 PWM 频率 = f_zv / 2
 * @return {VOID}
 * @author: lkc
 */
STATIC VOID Bsp_Tim1_Base(ULONG f_zv)
{
	/* 定时器时基初始化结构体 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* 使能定时器1时钟（APB2总线） */
    RCC_APB2PeriphClockCmd(HW_PWM1_RCC, ENABLE);

	/* 复位定时器至默认状态，清零计数器 */
	TIM_DeInit(HW_PWM1_TIM);
	HW_PWM1_TIM->CNT = 0;

	/* 预分频值：0 不分频，定时器时钟 = 系统核心时钟 */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	/* 中央对齐模式1：计数器先递增再递减，生成对称PWM波形，减少谐波 */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	/* 自动重载值：决定PWM周期，SYSTEM_CORE_CLOCK(168M) / f_zv */
	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / f_zv);
	/* 时钟分频：不分频，死区发生器时钟 = 定时器时钟 */
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	/* 重复计数器：0 表示每次溢出都产生更新事件（UEV） */
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	/* 写入时基配置到硬件寄存器 */
    TIM_TimeBaseInit(HW_PWM1_TIM, &TIM_TimeBaseStructure);

	return;
}

/**
 * @description: 定时器1 输出比较通道初始化（PWM输出配置）
 * @detail:
 *   - PWM模式：PWM1，CNT < CCR 时输出有效电平，CNT >= CCR 时输出无效电平
 *   - 同时使能主输出和互补输出，用于驱动三相半桥（上桥臂 + 下桥臂）
 *   - 初始占空比 50%（Pulse = ARR / 2），上电默认半占空比，避免瞬间电流冲击
 *   - 输出极性根据 INVERTED_TOP_DRIVER_INPUT / INVERTED_BOTTOM_DRIVER_INPUT 宏适配硬件
 *     未反相时：GPIO 高电平 = 上管导通；反相时：GPIO 低电平 = 上管导通
 *   - 空闲状态（IdleState）设为 Set，刹车或停机时输出确定电平
 *   - 初始化全部四个通道：CH1/CH2/CH3 用于三相电机 UVW，CH4 用于触发 ADC 采样
 *   - 开启预装载（影子寄存器），写入的占空比值暂存，等待 COM 事件后才更新到实际输出
 * @return {VOID}
 * @author: lkc
 */
STATIC VOID Bsp_Tim1_OC(VOID)
{
	/* 输出比较初始化结构体 */
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* PWM模式1：CNT < CCR 时输出有效电平 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	/* 主输出使能：上桥臂（高端MOS）PWM信号 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	/* 互补输出使能：下桥臂（低端MOS）PWM信号，与主输出相位相反 */
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	/* 初始占空比 50%，ARR/2 确保上下桥臂导通时间相等 */
	TIM_OCInitStructure.TIM_Pulse = HW_PWM1_TIM->ARR / 2;

	/* 上桥臂输出极性：根据驱动电路是否反相来选择
	 * 未反相(默认)：OCPolarity_High → CNT<CCR时输出高电平，上管导通
	 * 反相：OCPolarity_Low → CNT<CCR时输出低电平，适配反相驱动芯片 */
#ifndef INVERTED_TOP_DRIVER_INPUT
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
#else
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
#endif

	/* 主通道空闲状态：刹车或停机时输出高电平，关闭上管（高侧关断） */
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

	/* 下桥臂互补通道输出极性：同样根据驱动电路是否反相来选择 */
#ifndef INVERTED_BOTTOM_DRIVER_INPUT
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
#else
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
#endif
	/* 互补通道空闲状态：刹车或停机时输出高电平，关闭下管 */
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	/* 初始化四个输出比较通道
	 * CH1/CH2/CH3 → 三相电机 UVW 驱动
	 * CH4 → 用于触发 ADC 采样（通过定时器触发 ADC 注入组） */
    TIM_OC1Init(HW_PWM1_TIM, &TIM_OCInitStructure);
	TIM_OC2Init(HW_PWM1_TIM, &TIM_OCInitStructure);
	TIM_OC3Init(HW_PWM1_TIM, &TIM_OCInitStructure);
	TIM_OC4Init(HW_PWM1_TIM, &TIM_OCInitStructure);

	/* 开启各通道预装载（影子寄存器）
	 * 写入新的占空比值不会立即生效，等待 COM 事件（如在 ADC 中断中产生）
	 * 将 CCR 更新同步到 PWM 周期的安全窗口，避免输出毛刺 */
	TIM_OC1PreloadConfig(HW_PWM1_TIM, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(HW_PWM1_TIM, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(HW_PWM1_TIM, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(HW_PWM1_TIM, TIM_OCPreload_Enable);

	return;
}

/**
 * @description: 定时器1 刹车/死区/锁定寄存器配置（高级定时器保护功能）
 * @detail:
 *   本函数配置 TIM1 的 BDTR 寄存器，包含高级定时器特有的保护与安全功能：
 *   1. 死区插入（Dead-Time）：在上下桥臂切换之间插入延迟，防止半桥直通（shoot-through）
 *      死区时间通过 calculate_deadtime() 根据系统时钟和 HW_DEAD_TIME_NSEC 宏计算
 *   2. 刹车功能（Break）：可选，通过 HW_USE_BRK 宏控制
 *      外部故障信号（如过流检测）可异步关闭所有 PWM 输出，硬件直接关闭，无需软件干预
 *      PWM 输出保持关闭直到 MCU 复位，软件通过读取 BRK 标志上报故障码
 *   3. 关闭状态（OSSR/OSSI）：运行模式/空闲模式下输出电平默认状态
 *      OSSR_Enable + OSSI_Enable → 刹车/空闲时输出 IdleState 设定的电平
 *   4. 自动输出使能（MOE）：未启用自动模式，MOE 由软件控制（TIM_CtrlPWMOutputs）
 *   5. COM 事件预装载：CCPreloadControl 使能后，占空比的修改暂存于影子寄存器
 *      必须产生 COM 事件（通常在 ADC 中断中）才会从影子寄存器同步到实际输出
 *   6. ARR 预装载：ARR 值也暂存于影子寄存器，更新事件时同步，避免周期突变导致波形异常
 * @return {VOID}
 * @author: lkc
 */
VOID Bsp_Tim1_BDTR(VOID)
{
	/* 刹车/死区定时器初始化结构体 */
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	/* 运行模式下的关闭状态：刹车/故障时输出 IdleState 设定的电平 */
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	/* 空闲模式下的关闭状态：定时器未使能时输出 IdleState 设定的电平 */
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	/* 寄存器锁定等级：OFF 不锁定，调试阶段允许随时修改 BDTR */
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	/* 死区时间：根据 HW_DEAD_TIME_NSEC（死区时间 ns）和系统时钟自动计算 CKD 分频与 DTG 值
	 * 死区插入位置：上管关断后延迟死区时间，下管才导通；反之亦然，避免上下桥臂直通短路 */
	TIM_BDTRInitStructure.TIM_DeadTime  =  calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	/* 自动输出使能：禁用，MOE（主输出使能）由软件通过 TIM_CtrlPWMOutputs() 手动控制 */
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

#ifdef HW_USE_BRK
	/* 使能刹车功能：外部故障信号（如过流比较器输出）低电平触发
	 * 硬件异步关闭所有 PWM 通道输出，不依赖 CPU 中断响应
	 * PWM 保持关闭直到 MCU 复位，软件读取 BRK 标志以记录故障类型 */
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	/* 刹车极性：低电平有效（BREAK 引脚拉低时触发刹车） */
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
#else
	/* 禁用刹车功能 */
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
#endif

	/* 写入 BDTR 配置到硬件寄存器 */
	TIM_BDTRConfig(HW_PWM1_TIM, &TIM_BDTRInitStructure);
	/* 捕获/比较预装载控制：使能后 CCR 写入影子寄存器
	 * 需产生 COM 事件（通常在 ADC 中断 ISR 中）才会将影子寄存器更新到实际输出
	 * 这样可将新的占空比同步到 PWM 周期的安全时刻，避免输出中出现毛刺 */
	TIM_CCPreloadControl(HW_PWM1_TIM, ENABLE);
	/* ARR 自动重载预装载：使能后 ARR 值先写入影子寄存器
	 * 在更新事件（UEV）时才同步到实际寄存器，防止运行中改变周期产生异常波形 */
	TIM_ARRPreloadConfig(HW_PWM1_TIM, ENABLE);

	return;
}

/**
 * @description: 定时器1 PWM 初始化（电机1 主PWM输出）
 * @detail:
 *   本函数是电机1 PWM 输出的顶层初始化入口，完成 TIM1 全部配置并启动：
 *   1. 调用 Bsp_Tim1_Base() → 时基初始化（中央对齐、PWM频率 = f_zv/2）
 *   2. 调用 Bsp_Tim1_OC()  → 输出比较通道初始化（互补PWM、占空比50%、极性适配）
 *   3. 调用 Bsp_Tim1_BDTR() → 刹车/死区/预装载配置（死区时间、故障保护）
 *
 *   初始化顺序说明：
 *   - 先清零 CNT 再使能定时器，避免计数器从随机值开始
 *   - 使能后立即调用 StopPwm → 强制所有通道输出无效电平，上电瞬间 MOS 全关
 *   - 然后使能 MOE 主输出（TIM_CtrlPWMOutputs），默认引脚全低电平，必须手动使能
 *   - 最后设置 ADC 采样触发偏移（TIMER_UPDATE_SAMP），对齐电流采样点
 *
 *   注意：初始化完成后 PWM 波形仍处于停止状态（StopPwm 已将通道设为强制无效电平），
 *   需等待 FOC 算法准备好后再调用 Bsp_Tim_StartPwm() 恢复 PWM1 模式输出。
 * @param {ULONG} f_zv - PWM中心频率参数，实际 PWM 频率 = f_zv / 2，典型值 30000（15kHz PWM）
 * @return {VOID}
 * @author: lkc
 */
STATIC VOID Bsp_Tim_Pwm1(ULONG f_zv)
{
	/* 第1步：时基初始化（PWM周期、对齐模式） */
	Bsp_Tim1_Base(f_zv);
	/* 第2步：输出比较通道初始化（占空比、极性、互补输出） */
	Bsp_Tim1_OC();
	/* 第3步：刹车/死区/预装载配置（保护功能） */
	Bsp_Tim1_BDTR();

	/* 清零计数器并从0开始计数，使能定时器1 */
	HW_PWM1_TIM->CNT = 0;
    TIM_Cmd(HW_PWM1_TIM, ENABLE);

	/* 上电默认关闭所有 PWM 输出（强制无效电平，MOS 管全关）
	 * 防止 FOC 算法未就绪时误输出驱动信号导致短路 */
	Bsp_Tim_StopPwm(MOTOR1);

	/* 使能主输出（MOE）：高级定时器需要此操作才真正驱动 PWM 引脚
	 * 默认所有引脚为低电平，必须手动使能 MOE 才会有波形输出 */
    TIM_CtrlPWMOutputs(HW_PWM1_TIM, ENABLE);

	/* 配置 ADC 采样触发点偏移：在 PWM 周期中心采样相电流
	 * 使用 MCPWM_FOC_CURRENT_SAMP_OFFSET 偏移，避开开关噪声 */
	TIMER_UPDATE_SAMP(MCPWM_FOC_CURRENT_SAMP_OFFSET);
	return;
}

/**
 * @description: 停止 PWM 输出（安全关断三相桥臂）
 * @detail:
 *   将指定电机的三相 PWM 输出强制设为无效电平（MOS 全关），用于：
 *   - 初始化阶段：上电后先将所有 MOS 管关闭，等待 FOC 算法就绪
 *   - 紧急停机：检测到故障时快速切断电机驱动
 *   - 电机切换：切换控制策略前先关闭输出
 *
 *   实现原理（每相通道）：
 *   1. TIM_SelectOCxM(ForcedAction_InActive) → 强制通道输出无效电平（不跟随 CCR 比较）
 *   2. TIM_CCxCmd(Enable)  → 保持主通道（上桥臂）使能状态
 *   3. TIM_CCxNCmd(Disable) → 关闭互补通道（下桥臂），下管彻底截止
 *   4. TIM_GenerateEvent(COM) → 产生 COM 事件，将配置从影子寄存器更新到输出
 *
 *   - is_second = MOTOR1(0)：停止 TIM1（电机1）三相输出
 *     若定义 HW_HAS_DUAL_PARALLEL，同步停止 TIM8 的三相输出（双电机并联场景）
 *   - is_second = MOTOR2(1)：停止 TIM8（电机2）三相输出
 * @param {BOOL} is_second - 电机选择：MOTOR1(0) 停止电机1，MOTOR2(1) 停止电机2
 * @return {VOID}
 * @author: lkc
 */
VOID Bsp_Tim_StopPwm(BOOL is_second)
{
    /* 电机1（TIM1）：停止三相 PWM 输出 */
    if (!is_second)
    {
		/* U 相：强制主通道输出无效电平，关闭互补通道 */
		TIM_SelectOCxM(HW_PWM1_TIM,  TIM_Channel_1, TIM_ForcedAction_InActive);
		TIM_CCxCmd(HW_PWM1_TIM, 	 TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(HW_PWM1_TIM, 	 TIM_Channel_1, TIM_CCxN_Disable);

		/* V 相：同上 */
		TIM_SelectOCxM(HW_PWM1_TIM,  TIM_Channel_2, TIM_ForcedAction_InActive);
		TIM_CCxCmd(HW_PWM1_TIM, 	 TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(HW_PWM1_TIM, 	 TIM_Channel_2, TIM_CCxN_Disable);

		/* W 相：同上 */
		TIM_SelectOCxM(HW_PWM1_TIM,  TIM_Channel_3, TIM_ForcedAction_InActive);
		TIM_CCxCmd(HW_PWM1_TIM, 	 TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(HW_PWM1_TIM, 	 TIM_Channel_3, TIM_CCxN_Disable);

		/* 产生 COM 事件：将强制无效电平的配置立即同步到输出 */
		TIM_GenerateEvent(HW_PWM1_TIM, TIM_EventSource_COM);

#ifdef HW_HAS_DUAL_PARALLEL
		/* 双电机并联模式：同步停止 TIM8（电机2 并联在电机1 上）的三相输出 */
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
    }
    /* 电机2（TIM8）：停止三相 PWM 输出 */
    else
    {
		/* U 相：强制主通道输出无效电平，关闭互补通道 */
		TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, 	 TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, 	 TIM_Channel_1, TIM_CCxN_Disable);

		/* V 相 */
		TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, 	 TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, 	 TIM_Channel_2, TIM_CCxN_Disable);

		/* W 相 */
		TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, 	 TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, 	 TIM_Channel_3, TIM_CCxN_Disable);

		/* 产生 COM 事件同步到输出 */
		TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
    }

    return;
}

/**
 * @description: 启动 PWM 输出（从强制无效电平恢复为正常 PWM 模式）
 * @detail:
 *   StopPwm 的反操作，将三相通道从强制无效电平切换回 PWM1 模式，使能互补输出。
 *   调用时机：FOC 算法完成初始化、占空比计算就绪后，调用此函数开始驱动电机。
 *
 *   实现原理（每相通道）：
 *   1. TIM_SelectOCxM(PWM1) → 从 ForcedAction_InActive 切换回正常的 PWM1 比较输出模式
 *   2. TIM_CCxCmd(Enable)    → 使能主通道（上桥臂）输出
 *   3. TIM_CCxNCmd(Enable)   → 使能互补通道（下桥臂）输出
 *   4. TIM_GenerateEvent(COM) → 产生 COM 事件，将模式切换立即更新到硬件输出
 *
 *   - is_second = MOTOR1(0)：启动 TIM1（电机1）三相 PWM + 使能滤波器 + 可选 DRV8313 刹车
 *     若定义 HW_HAS_DUAL_PARALLEL，同步启动 TIM8 三相（双电机并联场景）
 *   - is_second = MOTOR2(1)：启动 TIM8（电机2）三相 PWM + 使能滤波器 + 可选 DRV8313_2 刹车
 * @param {BOOL} is_second - 电机选择：MOTOR1(0) 启动电机1，MOTOR2(1) 启动电机2
 * @return {VOID}
 * @author: lkc
 */
VOID Bsp_Tim_StartPwm(BOOL is_second)
{
	/* 电机1（TIM1）：恢复三相 PWM1 模式输出 */
	if (!is_second) {

		/* U 相：切换为 PWM1 模式，使能主通道和互补通道 */
		TIM_SelectOCxM(HW_PWM1_TIM,  TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(HW_PWM1_TIM, 	 TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(HW_PWM1_TIM, 	 TIM_Channel_1, TIM_CCxN_Enable);

		/* V 相：同上 */
		TIM_SelectOCxM(HW_PWM1_TIM,  TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(HW_PWM1_TIM, 	 TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(HW_PWM1_TIM, 	 TIM_Channel_2, TIM_CCxN_Enable);

		/* W 相：同上 */
		TIM_SelectOCxM(HW_PWM1_TIM,  TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(HW_PWM1_TIM, 	 TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(HW_PWM1_TIM, 	 TIM_Channel_3, TIM_CCxN_Enable);


#ifdef HW_HAS_DUAL_PARALLEL
		/* 双电机并联模式：同步启动 TIM8（电机2 并联在电机1 上）的三相 PWM */
		TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Enable);

		/* 使能电机2 相电流采样滤波器 */
		PHASE_FILTER_ON_M2();
#endif
		/* 产生 COM 事件：将模式切换（PWM1）从影子寄存器立即更新到硬件输出
		 * 之前写入的 CCR 占空比值也随此 COM 事件一并生效
		 * 后续在 ADC 中断中也会产生 COM 事件，以保证占空比更新与采样同步 */
		TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
#ifdef HW_HAS_DRV8313
		/* DRV8313 驱动芯片：使能刹车释放（退出刹车保护状态） */
		ENABLE_BR();
#endif
		/* 使能电机1 相电流采样滤波器：开启滤波电容，滤除 ADC 采样噪声 */
		PHASE_FILTER_ON();
	} else {
		/* 电机2（TIM8）：恢复三相 PWM1 模式输出 */

		/* U 相 */
		TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, 	 TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, 	 TIM_Channel_1, TIM_CCxN_Enable);

		/* V 相 */
		TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, 	 TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, 	 TIM_Channel_2, TIM_CCxN_Enable);

		/* W 相 */
		TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, 	 TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, 	 TIM_Channel_3, TIM_CCxN_Enable);

		/* 产生 COM 事件同步到输出 */
		TIM_GenerateEvent(TIM8, TIM_EventSource_COM);

#ifdef HW_HAS_DRV8313_2
		ENABLE_BR_2();
#endif

		/* 使能电机2 相电流采样滤波器 */
		PHASE_FILTER_ON_M2();
	}

    return;
}

/**
 * @description: 定时器8 PWM 初始化（电机2 主PWM输出）
 * @detail:
 *   电机2 的 PWM 初始化，使用 TIM8（高级定时器），配置与 TIM1（电机1）相同的三步骤：
 *   - 时基初始化：中央对齐模式1，PWM频率 = f_zv / 2
 *   - 输出比较通道：互补PWM、四通道、50%初始占空比、预装载使能
 *   - BDTR保护功能：死区插入、刹车、预装载控制
 *
 *   与 Bsp_Tim_Pwm1 的关键区别：
 *   1. 本函数将三步配置内联在一个函数中，而非拆分为三个 STATIC 子函数调用
 *   2. TIM_OCInitStructure 缺少 .TIM_OCMode 和 .TIM_OutputState/.TIM_OutputNState/.TIM_Pulse 的显式赋值
 *      （依赖结构体初始化时的默认 0 值或前序代码的设置）
 *   3. CNT 初始化策略：
 *      - HW_HAS_DUAL_MOTORS 定义时：TIM8->CNT = TIM1->ARR，使电机2 PWM 与电机1 错相 180°
 *        目的：减少双电机同时开关造成的母线电流尖峰（interleaving 交错并联技术）
 *      - 否则：TIM8->CNT = 0，独立从零开始
 *   4. 初始化后不调用 StopPwm，直接使能 MOE 输出（TIM8 可能依赖 TIM1 触发链启动）
 * @param {ULONG} f_zv - PWM中心频率参数，实际 PWM 频率 = f_zv / 2
 * @return {VOID}
 * @author: lkc
 */
VOID Bsp_Tim_Pwm2(ULONG f_zv)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

    /* 使能 TIM8 时钟（APB2总线） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	/* 复位 TIM8 至默认状态，清零计数器 */
	TIM_DeInit(TIM8);
	TIM8->CNT = 0;

    /* ===== 第1步：时基初始化 ===== */
	/* 预分频值：0 不分频 */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	/* 中央对齐模式1：对称PWM，减少谐波 */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	/* 自动重载值：SYSTEM_CORE_CLOCK(168M) / f_zv */
	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / f_zv);
	/* 时钟分频：不分频 */
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	/* 重复计数器：0，每次溢出产生更新事件 */
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    /* ===== 第2步：输出比较通道初始化 ===== */
	/* 上桥臂输出极性 */
    #ifndef INVERTED_TOP_DRIVER_INPUT
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
#else
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
#endif
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

	/* 下桥臂互补通道输出极性 */
#ifndef INVERTED_BOTTOM_DRIVER_INPUT
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
#else
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
#endif
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	/* 初始化四个通道：CH1-3 三相电机，CH4 ADC触发 */
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);

	/* 开启预装载，CCR写入影子寄存器，COM事件后生效 */
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

    /* ===== 第3步：刹车/死区/预装载配置 ===== */
	/* 运行模式和空闲模式下，刹车时输出 IdleState 设定电平 */
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	/* 寄存器不锁定，调试阶段可修改 */
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	/* 死区时间：动态计算，避免上下桥臂直通 */
	TIM_BDTRInitStructure.TIM_DeadTime =  calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	/* MOE 由软件手动控制 */
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

    #ifdef HW_USE_BRK
	/* 使能刹车：外部故障信号低电平触发，硬件异步关闭 PWM */
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
#else
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
#endif

	TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);
	/* COM事件预装载：CCR值等COM事件后更新 */
	TIM_CCPreloadControl(TIM8, ENABLE);
	/* ARR预装载：周期值等UEV更新事件后生效 */
	TIM_ARRPreloadConfig(TIM8, ENABLE);

    /* ===== 启动定时器 ===== */
    #ifdef HW_HAS_DUAL_MOTORS
        /* 双电机模式：TIM8 CNT 初始化为 TIM1 的 ARR 值（PWM周期值）
         * 效果：TIM8 从周期顶端开始计数，与 TIM1 形成 180° 错相（interleaving）
         * 优势：两个电机的 PWM 开关时刻交错，降低母线电容纹波电流峰值 */
        TIM8->CNT = HW_PWM1_TIM->ARR;
    #else
        /* 单电机模式：从 0 开始计数 */
        TIM8->CNT = 0;
    #endif
    /* 使能 TIM8 计数器 */
    TIM_Cmd(TIM8, ENABLE);
    /* 使能 MOE 主输出，驱动 PWM 引脚 */
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
VOID Bsp_Tim1_TrigAdc(VOID)
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
VOID Bsp_Tim1_Trig(VOID)
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

VOID Bsp_Tim3_Nvic(VOID)
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
	Bsp_Tim3_Nvic();

	return;
}

#define TIMER_DT_HZ					1.4e7
/**
 * @description: 微秒级高精度计时器初始化（TIM5 自由运行计数器）
 * @detail:
 *   使用 TIM5 32位定时器作为系统高精度时间基准，用于测量代码段执行时间：
 *   - 定时器时钟：APB1 定时器时钟 = SYSTEM_CORE_CLOCK / 2 = 84MHz（TIM5 在 APB1 上）
 *   - 预分频器：84 - 1 = 83 → 计数频率 = 84MHz / 84 = 1MHz（每 tick = 1µs）
 *   - 周期值：0xFFFFFFFF（32位满量程），定时器自由运行不产生溢出中断
 *   - 模式：向上计数，单次溢出周期约 4295 秒（~71 分钟）
 *
 *   配套 API：
 *   - Bsp_Tim_DtNow()：读取当前 CNT 值（单位：µs）
 *   - Bsp_Tim_DtSecondElapsedSince(t)：计算从时刻 t 到现在的秒数差值
 *
 *   注意：
 *   - 当前硬编码 Prescaler = 84-1（1µs 分辨率），注释掉的 PrescalerValue 计算
 *     可达到 TIMER_DT_HZ(14MHz) 的计数频率，但因某种原因暂未使用
 *   - 32位计数器约 71 分钟溢出一次，Bsp_Tim_DtSecondElapsedSince 使用无符号减法
 *     天然处理溢出回绕（两次 CNT 差值在溢出后仍正确）
 * @return {VOID}
 * @author: lkc
 */
VOID Bsp_Tim5_Dt(VOID)
{
	/* 使能 TIM5 时钟（APB1总线，定时器时钟 = SYSTEM_CORE_CLOCK / 2 = 84MHz） */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	/* 理论预分频值：如启用可使计数频率达到 TIMER_DT_HZ (14MHz)
	 * 计算公式：Prescaler = (APB1定时器时钟 / 目标频率) - 1 */
	uint16_t PrescalerValue = (uint16_t) ((SYSTEM_CORE_CLOCK / 2) / TIMER_DT_HZ) - 1;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* 周期值：32位满量程，自由运行不产生更新中断 */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	/* 预分频器：84-1=83，计数频率 = 84MHz / 84 = 1MHz，每 tick = 1µs
	 * TODO: 当前使用微秒分辨率，注释掉的 PrescalerValue 可达到 14MHz 更高精度 */
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
	/* 时钟分频：不分频 */
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	/* 向上计数模式 */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	/* 清零计数器，开始计时 */
	TIM5->CNT = 0;
	TIM_Cmd(TIM5, ENABLE);

	return;
}

// 当前定时器count
ULONG Bsp_Tim_DtNow(void) 
{
	return TIM5->CNT;
}

// 时间差值
float Bsp_Tim_DtSecondElapsedSince(ULONG time) 
{
	uint32_t diff = TIM5->CNT - time;
	return (float)diff / (float)TIMER_DT_HZ;
}

/**
 * @description: 编码器定时器初始化
 * @detail:  ab脉冲 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim4_EncInit(VOID)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimBaseInitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	/* 时钟初始化 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* 引脚初始化 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 复用引脚 */
	GPIO_PinAFConfig(GPIOB, GPIO_Pin_6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_Pin_7, GPIO_AF_TIM4);

	/* 定时器初始化 */
	TIM_TimBaseInitStructure.TIM_Period = 1024 * 4 - 1;
	TIM_TimBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &TIM_TimBaseInitStructure);

	/* 4倍频 */
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 

	// TODO 滤波最后加
	// TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	// TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	// TIM_ICInitStructure.TIM_ICFilter =

	TIM_Cmd(TIM4, ENABLE);

	return;
}

/**
 * @description: 定时器霍尔
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim3_Hall()
{

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
    Bsp_Tim_Pwm1(30000);
    Bsp_Tim1_TrigAdc();
	Bsp_Tim1_Trig();
	Bsp_Tim5_Dt();
	//Bsp_Tim_RunCount();
	// todo 这里其实还有定时器3中断,只不过是写在freertos里边了
	
	return;
}
#ifdef __cplusplus
}
#endif
