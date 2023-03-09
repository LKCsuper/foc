/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-05 10:43:58
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
 * @Description: 时钟初始化
 * @author: lkc
 * @Date: 2023-02-18 10:11:54
 * @param {u32} pllm
 * @param {u32} plln
 * @param {u32} pllp
 * @param {u32} pllq
 * @return {*}
 */
void Bsp_Rcu_Init(u32 pllm,u32 plln,u32 pllp,u32 pllq)
{
	RCC_DeInit();

	RCC_HSEConfig(RCC_HSE_ON);
	while( RCC_WaitForHSEStartUp() != SUCCESS );

	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	if(RCC_WaitForHSEStartUp()==SUCCESS)
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div2);
		RCC_PCLK1Config(RCC_HCLK_Div4);
		RCC_PLLConfig(RCC_PLLSource_HSE,pllm,plln,pllp,pllq);
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource()!=0x08);
	}

	return;
}

/**
 * @description: systick初始化
 * @detail: 
 * @param {int} fzv
 * @return {*}
 * @author: lkc
 */
void Bsp_Sys_Init(int fzv)
{
    // 中断初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

    if (SysTick_Config(SystemCoreClock / 1000)) 
    {
        /* Capture error */
        while (1);
    }
    NVIC_EnableIRQ(SysTick_IRQn); 

	return;
}

void HSE_SetSysClock(uint32_t m, uint32_t n, uint32_t p, uint32_t q)
{
    __IO uint32_t HSEStartUpStatus = 0;

    // 使能HSE，开启外部晶振，野火F407使用 HSE=25M
    RCC_HSEConfig(RCC_HSE_ON);

    // 等待HSE启动稳定
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS) {
        // 调压器电压输出级别配置为1，以便在器件为最大频率
        // 工作时使性能和功耗实现平衡
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_VOS;

        // HCLK = SYSCLK / 1
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        // PCLK2 = HCLK / 2
        RCC_PCLK2Config(RCC_HCLK_Div2);

        // PCLK1 = HCLK / 4
        RCC_PCLK1Config(RCC_HCLK_Div4);

        // 如果要超频就得在这里下手啦
        // 设置PLL来源时钟，设置VCO分频因子m，设置VCO倍频因子n，
        //  设置系统时钟分频因子p，设置OTG FS,SDIO,RNG分频因子q
        RCC_PLLConfig(RCC_PLLSource_HSE, m, n, p, q);

        // 使能PLL
        RCC_PLLCmd(ENABLE);

        // 等待 PLL稳定
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
        }

        /*-----------------------------------------------------*/
        // 配置FLASH预取指,指令缓存,数据缓存和等待状态
        FLASH->ACR = FLASH_ACR_PRFTEN
                    | FLASH_ACR_ICEN
                    | FLASH_ACR_DCEN
                    | FLASH_ACR_LATENCY_5WS;
        /*-----------------------------------------------------*/

        // 当PLL稳定之后，把PLL时钟切换为系统时钟SYSCLK
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        // 读取时钟切换状态位，确保PLLCLK被选为系统时钟
        while (RCC_GetSYSCLKSource() != 0x08) {
        }
    } else {
        // HSE启动出错处理

        while (1) {
        }
    }
}

void HSI_SetSysClock(uint32_t m, uint32_t n, uint32_t p, uint32_t q)
{
    __IO uint32_t HSIStartUpStatus = 0;

    // 把RCC外设初始化成复位状态
    RCC_DeInit();

    //使能HSI, HSI=16M
    RCC_HSICmd(ENABLE);

    // 等待 HSI 就绪
    HSIStartUpStatus = RCC->CR & RCC_CR_HSIRDY;

    // 只有 HSI就绪之后则继续往下执行
    if (HSIStartUpStatus == RCC_CR_HSIRDY) {
        // 调压器电压输出级别配置为1，以便在器件为最大频率
        // 工作时使性能和功耗实现平衡
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_VOS;

        // HCLK = SYSCLK / 1
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        // PCLK2 = HCLK / 2
        RCC_PCLK2Config(RCC_HCLK_Div2);

        // PCLK1 = HCLK / 4
        RCC_PCLK1Config(RCC_HCLK_Div4);

        // 如果要超频就得在这里下手啦
        // 设置PLL来源时钟，设置VCO分频因子m，设置VCO倍频因子n，
        //  设置系统时钟分频因子p，设置OTG FS,SDIO,RNG分频因子q
        RCC_PLLConfig(RCC_PLLSource_HSI, m, n, p, q);

        // 使能PLL
        RCC_PLLCmd(ENABLE);

        // 等待 PLL稳定
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
        }

        /*-----------------------------------------------------*/
        // 配置FLASH预取指,指令缓存,数据缓存和等待状态
        FLASH->ACR = FLASH_ACR_PRFTEN
                    | FLASH_ACR_ICEN
                    |FLASH_ACR_DCEN
                    |FLASH_ACR_LATENCY_5WS;
        /*-----------------------------------------------------*/

        // 当PLL稳定之后，把PLL时钟切换为系统时钟SYSCLK
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        // 读取时钟切换状态位，确保PLLCLK被选为系统时钟
        while (RCC_GetSYSCLKSource() != 0x08) {
        }
    } else {
        // HSI启动出错处理
        while (1) {
        }
    }
}

/**
 * @description: 延时初始化
 * @detail: 
 * @param {unsigned int} ms 毫秒
 * @return {*}
 * @author: lkc
 */
void Ymodem_Delay(unsigned int ms)
{
    osDelay(ms);
    return;
}
#ifdef __cplusplus
}
#endif
