/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-02-25 10:41:38
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
 * @description: led初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void Bsp_Gpio_Led(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(GREEN_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GREEN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉，默认为高电平
    GPIO_Init(GREEN_PORT, &GPIO_InitStructure);

    RCC_AHB1PeriphClockCmd(RED_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = RED_PIN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉，默认为高电平
    GPIO_Init(RED_PORT, &GPIO_InitStructure);

    return;
}


/**
 * @description: GPIO初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void Bsp_Gpio_Init(void)
{
    Bsp_Gpio_Led();

    return;
}

#ifdef __cplusplus
}
#endif
