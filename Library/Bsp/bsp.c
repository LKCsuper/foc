/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-13 23:06:46
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
 * @description: 初始化操作系统外设RTOS
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_RTOS_Init(VOID)
{
    /* 时钟初始化 */
    Bsp_Rcu_Init(8, 336, 4, 7);
    /* hsi 时钟初始化 */
    /* HSI_SetSysClock(16, 336, 2, 7); */
    
    /* 中断 sys初始化 */
    Bsp_Sys_Init(1000);

    return;
}

/**
 * @description: 硬件初始化
 * @detail:    
 *  
串口初始化 如果在打印后边就会导致卡死 
定时器问题,如果屏蔽定时器初始化,因为freertos时间统计需要定时器,所以也会卡死
定时器初始化,因为可能和adc dma有挂钩,所以不初始化adc和 dma也会卡死
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Init(VOID)
{
    /* gpio 初始化 */
    Bsp_Gpio_Init();

    /* 串口初始化 */
    Bsp_Usart_DebugInit(115200);
	
    /* 初始化定时器1 pwm */
    Bsp_Tim_Init();
    Bsp_Adc_Init();
    Bsp_Dma_Init();

    PRINTF("hardware init success\r\n");

    return;
}

#ifdef __cplusplus
}
#endif
