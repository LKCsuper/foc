/*
 * @Description: 时钟与系统基础初始化（PLL 时钟树 / SysTick / 复位）
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-06 17:40:19
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
extern uint32_t SystemCoreClock;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 时钟初始化（HSE 晶振 + PLL）
 * @Description: 时钟初始化
 * @author: lkc
 * @Date: 2023-02-18 10:11:54
 * @param {u32} ulPllm PLL 输入分频系数 M
 * @param {u32} ulPlln PLL 倍频系数 N
 * @param {u32} ulPllp PLL 主时钟分频系数 P
 * @param {u32} ulPllq PLL 48 MHz 域分频系数 Q
 * @return {void} 无
 */
VOID Bsp_Rcu_Init(ULONG ulPllm, ULONG ulPlln, ULONG ulPllp, ULONG ulPllq)
{
	RCC_DeInit();

	RCC_HSEConfig(RCC_HSE_ON);
	while(RCC_WaitForHSEStartUp() != SUCCESS );

	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	if(RCC_WaitForHSEStartUp() == SUCCESS)
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div2);
		RCC_PCLK1Config(RCC_HCLK_Div4);
		RCC_PLLConfig(RCC_PLLSource_HSE, ulPllm, ulPlln, ulPllp, ulPllq);
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
	}

    // todo 这里应该对apb进行配置一下

	return;
}

/**
 * @description: systick初始化
 * @param {int} ulFzv 系统节拍频率 (Hz)，作为 SystemCoreClock 的除数
 * @return {void} 无
 * @author: lkc
 */
VOID Bsp_Sys_Init(ULONG ulFzv)
{
	/* 	中断初始化
		优先级分组4,4位抢占优先级,4位响应优先级
		向量表偏移量设置为0x0,中断向量表在flash中
		配置系统滴答定时器,每1ms中断一次
	*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	if (SysTick_Config(SystemCoreClock / ulFzv)) 
	{
		/* Capture error */
		while (1) {};
	}
	// 使能SysTick中断
	NVIC_EnableIRQ(SysTick_IRQn); 

	return;
}

/**
 * @description: hse设置
 * @param {ULONG} ulPllm PLL 输入分频因子 M
 * @param {ULONG} ulPlln PLL 倍频因子 N
 * @param {ULONG} ulPllp 系统时钟分频因子 P
 * @param {ULONG} ulPllq 外设时钟分频因子 Q
 * @return {void} 无
 * @author: lkc
 */
VOID HSE_SetSysClock(ULONG ulPllm, ULONG ulPlln, ULONG ulPllp, ULONG ulPllq)
{
	__IO ULONG ulHSEStartUpStatus = 0;

	/* 使能HSE，开启外部晶振，野火F407使用 HSE=8M*/
	RCC_HSEConfig(RCC_HSE_ON);

	/* 等待HSE启动稳定 */
	ulHSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (ulHSEStartUpStatus == SUCCESS) 
	{
		/* 调压器电压输出级别配置为1，以便在器件为最大频率
			工作时使性能和功耗实现平衡*/
		RCC->APB1ENR |= RCC_APB1ENR_PWREN;
		PWR->CR |= PWR_CR_VOS;

		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div2);
		RCC_PCLK1Config(RCC_HCLK_Div4);
		/* 如果要超频就得在这里下手啦
			设置PLL来源时钟，设置VCO分频因子m，设置VCO倍频因子n，
			设置系统时钟分频因子p，设置OTG FS,SDIO,RNG分频因子q
		*/
		RCC_PLLConfig(RCC_PLLSource_HSE, ulPllm, ulPlln, ulPllp, ulPllq);

		/* 使能PLL */
		RCC_PLLCmd(ENABLE);

		/* 等待 PLL稳定 */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

		/* 配置FLASH预取指,指令缓存,数据缓存和等待状态 */
		FLASH->ACR = FLASH_ACR_PRFTEN
					| FLASH_ACR_ICEN
					| FLASH_ACR_DCEN
					| FLASH_ACR_LATENCY_5WS;

		/* 当PLL稳定之后，把PLL时钟切换为系统时钟SYSCLK */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* 读取时钟切换状态位，确保PLLCLK被选为系统时钟 */
		while (RCC_GetSYSCLKSource() != 0x08) {}
	} 
	else 
	{
		while (1) {}
	}
}

/**
 * @description: hsi设置
 * @param {ULONG} ulPllm PLL 输入分频因子 M
 * @param {ULONG} ulPlln PLL 倍频因子 N
 * @param {ULONG} ulPllp 系统时钟分频因子 P
 * @param {ULONG} ulPllq 外设时钟分频因子 Q
 * @return {void} 无
 * @author: lkc
 */
VOID HSI_SetSysClock(ULONG ulPllm, ULONG ulPlln, ULONG ulPllp, ULONG ulPllq)
{
	__IO uint32_t HSIStartUpStatus = 0;

	/* 把RCC外设初始化成复位状态 */
	RCC_DeInit();

	/* 使能是16m */
	RCC_HSICmd(ENABLE);

	/* 等待 HSI 就绪 */
	HSIStartUpStatus = RCC->CR & RCC_CR_HSIRDY;
	/* 只有 HSI就绪之后则继续往下执行 */
	if (HSIStartUpStatus == RCC_CR_HSIRDY) 
	{
		/*  调压器电压输出级别配置为1，以便在器件为最大频率
			工作时使性能和功耗实现平衡*/
		RCC->APB1ENR |= RCC_APB1ENR_PWREN;
		PWR->CR |= PWR_CR_VOS;

		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div2);
		RCC_PCLK1Config(RCC_HCLK_Div4);
		/* 如果要超频就得在这里下手啦
			设置PLL来源时钟，设置VCO分频因子m，设置VCO倍频因子n，
			设置系统时钟分频因子p，设置OTG FS,SDIO,RNG分频因子q
		*/
		RCC_PLLConfig(RCC_PLLSource_HSI, ulPllm, ulPlln, ulPllp, ulPllq);

		/* 使能PLL */
		RCC_PLLCmd(ENABLE);
		/* 等待 PLL稳定*/
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

		/* 配置FLASH预取指,指令缓存,数据缓存和等待状态*/
		FLASH->ACR = FLASH_ACR_PRFTEN
					| FLASH_ACR_ICEN
					|FLASH_ACR_DCEN
					|FLASH_ACR_LATENCY_5WS;
		/* 当PLL稳定之后，把PLL时钟切换为系统时钟SYSCLK */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* 读取时钟切换状态位，确保PLLCLK被选为系统时钟 */
		while (RCC_GetSYSCLKSource() != 0x08) {}
	} 
	else {
		/* HSI启动出错处理 */
		while (1) {}
	}

	return;
}

/**
 * @description: 重启设备
 * @return {void} 无
 * @author: lkc
 */
VOID Bsp_Misc_Reset(VOID)
{
	NVIC_SystemReset();

	return;
}
#ifdef __cplusplus
}
#endif
