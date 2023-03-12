/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-12 09:26:57
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
 * @description: 硬件初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void Bsp_Init(void)
{
    /* 时钟初始化 */
    Bsp_Rcu_Init(8, 336, 4, 7);
    //HSI_SetSysClock(16, 336, 2, 7);
    /* 中断 sys初始化 */
    Bsp_Sys_Init(1000);

    /* 串口初始化 */
    // ! 这里如果也初始化串口,就会导致命令行起不来 并且打印也不能使用
    Bsp_Usart_DebugInit(115200);
    
    /* 初始化定时器1 pwm */
    Bsp_Gpio_Init();
    Bsp_Tim_Init();
    Bsp_Adc_Init();
    Bsp_Dma_Init();

    PRINTF("hardware init success\r\n");

    return;
}

#ifdef __cplusplus
}
#endif
