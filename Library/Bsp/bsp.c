/*
 * @Description: bsp初始化
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-10-06 20:14:07
 * @FilePath: \foc\Library\Bsp\bsp_dma.c
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
 * @description: 初始化系统时钟和 FreeRTOS 基础外设
 * @detail:
 *   本函数完成两件事：① 配置 STM32F4 的 PLL 时钟树 ② 初始化 SysTick 为 FreeRTOS 节拍源
 *
 *   ┌─────────────────────────────────────────────────────────────────────┐
 *   │                    STM32F407 时钟树（当前配置）                      │
 *   │                                                                     │
 *   │   HSE/HSI (8MHz)                                                    │
 *   │       │                                                             │
 *   │       ├── /M (÷8) ──→ VCO输入 = 1MHz                                │
 *   │       │                   │                                         │
 *   │       │                   ├── ×N (×168) ──→ VCO输出 = 168MHz         │
 *   │       │                   │               │                         │
 *   │       │                   │               ├── /P (÷2) ──→ PLLCLK = 84MHz ×2 = 168MHz ──→ SYSCLK = 168MHz
 *   │       │                   │               │                              │
 *   │       │                   │               │                    ┌────────┴────────┐
 *   │       │                   │               │                    │                 │
 *   │       │                   │               │              AHB (/1)           APB1 预分频
 *   │       │                   │               │            HCLK = 168MHz          │
 *   │       │                   │               │               │                    │
 *   │       │                   │               │         ┌─────┴──────┐       ┌──────┴──────┐
 *   │       │                   │               │         │            │       │             │
 *   │       │                   │               │    APB1 (/4)   APB2 (/2)   │             │
 *   │       │                   │               │   PCLK1=42MHz  PCLK2=84MHz │             │
 *   │       │                   │               │         │            │       │             │
 *   │       │                   │               │   ×2=84MHz   ×2=168MHz      │             │
 *   │       │                   │               │  (定时器时钟)  (定时器时钟)    │             │
 *   │       │                   │               │                              │             │
 *   │       │                   └── /Q (÷4) ──→ PLL48CLK = 42MHz  (USB/SDIO/RNG)             │
 *   │       │                                ⚠️ 42MHz 不满足 USB 48MHz 要求                    │
 *   │       │                                                             │             │
 *   │   └──────────────────────────────────────────────────────────────────┘             │
 *   │                                                                     │
 *   │   各总线频率汇总：                                                   │
 *   │   ┌──────────┬──────────────┬───────────────────────────────────┐    │
 *   │   │ 总线     │ 频率         │ 挂载外设                          │    │
 *   │   ├──────────┼──────────────┼───────────────────────────────────┤    │
 *   │   │ SYSCLK   │ 168 MHz      │ CPU 内核                          │    │
 *   │   │ HCLK     │ 168 MHz      │ AHB: DMA1/2, GPIO, CRC           │    │
 *   │   │ APB1     │ 42 MHz       │ TIM2-7, TIM12-14, USART2-5,      │    │
 *   │   │          │              │ I2C1-3, SPI2-3, CAN1-2           │    │
 *   │   │ APB1定时器│ 84 MHz       │ TIM2/3/4/5 实际计数时钟          │    │
 *   │   │ APB2     │ 84 MHz       │ TIM1/8, USART1/6, ADC1-3,        │    │
 *   │   │          │              │ SPI1, SDIO, SYSCFG               │    │
 *   │   │ APB2定时器│ 168 MHz      │ TIM1/8 实际计数时钟              │    │
 *   │   │ PLL48CLK │ 42 MHz       │ USB OTG, SDIO, RNG (不合规)      │    │
 *   │   │ ADC时钟  │ 42 MHz       │ APB2/2=42MHz (超手册36MHz上限)   │    │
 *   │   └──────────┴──────────────┴───────────────────────────────────┘    │
 *   └─────────────────────────────────────────────────────────────────────┘
 *
 *   两种时钟源配置：
 *
 *   【HSE 模式】Bsp_Rcu_Init(8, 336, 2, 4)
 *     - 输入：外部 8MHz 晶振（HSE）
 *     - PLL 参数：M=8, N=336, P=2, Q=4
 *     - VCO = 8/8 × 336 = 336MHz
 *     - SYSCLK = 336/2 = 168MHz
 *     - PLL48CLK = 336/4 = 84MHz  ⚠️ 84MHz ≠ 48MHz，USB 不可用
 *     - 注意：注释说"q分频超了最大48"，实际 Q=4 时 84MHz 已远超 USB 48MHz 要求
 *
 *   【HSI 模式（当前使用）】HSI_SetSysClock(8, 168, 2, 4)
 *     - 输入：内部 16MHz RC 振荡器（HSI），但 PLL M=8 分频后 VCO 输入 = 2MHz
 *     - PLL 参数：M=8, N=168, P=2, Q=4
 *     - VCO = 16/8 × 168 = 336MHz
 *     - SYSCLK = 336/2 = 168MHz
 *     - PLL48CLK = 336/4 = 84MHz  ⚠️ 同样不满足 USB 48MHz
 *     - HSI 精度 (±1%) 不如 HSE (±30ppm)，电机控制场景建议使用 HSE
 *
 *   SysTick 配置：
 *     - Bsp_Sys_Init(1000) → SysTick_Config(168000000 / 1000) = 每 1ms 中断一次
 *     - 作为 FreeRTOS 的 tick 源，tick 频率 = 1000Hz
 *     - NVIC 优先级组：Group4（4位抢占优先级，0位子优先级）
 *
 * @return {VOID}
 * @author: lkc
 */
VOID Bsp_RTOS_Init(VOID)
{
#ifdef HSE_USE
	/* HSE 外部晶振模式：8MHz 晶振 → PLL → SYSCLK 168MHz */
	Bsp_Rcu_Init(8, 336, 2, 4);
#else
	/* HSI 内部振荡器模式：16MHz HSI → PLL → SYSCLK 168MHz */
	HSI_SetSysClock(8, 168, 2, 4);
#endif
	/* SysTick 1ms 中断 + NVIC 优先级分组4（FreeRTOS 要求） */
	Bsp_Sys_Init(10000);

	return;
}

/**
 * @description: 硬件初始化
 * @detail:    
 *  
串初始化 如果在打印后边就会导致卡死 
定时器问题,如果屏蔽定时器初始化,因为freertos时间统计需要定时器,所以也会卡死
定时器初始化,因为可能和adc dma有挂钩,所以不初始化adc和 dma也会卡死
电脑串口长时间插着,自己的串口必须要再插拔一下才能有回显,应该是电脑驱动问题
外设不起作用,时钟没有使能,gpio没有初始化
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Init(VOID)
{
	/* gpio 初始化 */
	Bsp_Gpio_Init();

	/* 串口初始化 */
	Bsp_Usart_DebugInit(DEBUG_USART_BAUD);

	/* 初始化定时器1 pwm */
	// TODO 注意: 如果初始化顺序 Tim Adc Dma 如果tim和dma中断中间没有加打印,串口就会死掉
	// TODO 更改初始化顺序 或者 中间加打印 串口正常
	Bsp_Adc_Init();
	Bsp_Dma_Init();
	Bsp_Tim_Init();

	PRINTF("Hardware Init Success\r\n");

	return;
}

#ifdef __cplusplus
}
#endif
