/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-11 23:25:26
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
 * @description: 输出模式 推挽
 * @detail: 
 * @param {GPIO_TypeDef*} GPIOx
 * @param {GPIO_InitTypeDef*} GPIO_InitStruct
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Gpio_Mode1(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, GPIOMode_TypeDef GPIO_Mode)
{
    /* gpio时钟默认前边初始化 */

    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOx, &GPIO_InitStructure);

    return;
}

/**
 * @description: led初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Gpio_Led(VOID)
{
    Bsp_Gpio_Mode1(RED_PORT, RED_PIN, GPIO_Mode_OUT);
    Bsp_Gpio_Mode1(GREEN_PORT, GREEN_PIN, GPIO_Mode_OUT);
    
    return;
}

/**
 * @description: 定时器gpio初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim1_Gpio(VOID)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(T_MOTOR1_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = T_CHANNEL1_PIN | T_CHANNEL2_PIN | T_CHANNEL3_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    /* 设置最高翻转速度*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(T_CHANNEL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = T_CHANNEL1_N_PIN | T_CHANNEL2_N_PIN | T_CHANNEL3_N_PIN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(T_CHANNEL_N_PORT, &GPIO_InitStructure);

    /* 复用 */
    GPIO_PinAFConfig(T_CHANNEL_PORT, T_CHANNEL1_PIN | T_CHANNEL2_PIN | T_CHANNEL3_PIN, GPIO_AF_TIM1);
    GPIO_PinAFConfig(T_CHANNEL_N_PORT, T_CHANNEL1_N_PIN | T_CHANNEL2_N_PIN | T_CHANNEL3_N_PIN, GPIO_AF_TIM1);

	return;
}

/**
 * @description: 定时器gpio初始化
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Tim8_Gpio(VOID)
{

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
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    Bsp_Gpio_Led();
    Bsp_Gpio_Mode1(GATE_PORT, GATE_PIN, GPIO_Mode_OUT);
    Bsp_Gpio_Mode1(DCCAL_PORT, DCCAL_PIN, GPIO_Mode_OUT);
    palClearPad(DCCAL_PORT, DCCAL_PIN);
    ENABLE_GATE();
    Bsp_Tim1_Gpio();

    /* 编码器 霍尔 输入 */

    /* phase filters */
    Bsp_Gpio_Mode1(PHASE_FILTER_PORT, PHASE_FILTER_PIN, GPIO_Mode_OUT);
    PHASE_FILTER_OFF();

    /* Fault */
    Bsp_Gpio_Mode1(PHASE_FILTER_PORT, PHASE_FILTER_PIN, GPIO_Mode_IN);

    /* ADC */
    Bsp_Gpio_Mode1(ADC_0_PORT, ADC_0_PIN, GPIO_Mode_AN);
    Bsp_Gpio_Mode1(ADC_1_PORT, ADC_1_PIN, GPIO_Mode_AN);
    Bsp_Gpio_Mode1(ADC_2_PORT, ADC_2_PIN, GPIO_Mode_AN);
    Bsp_Gpio_Mode1(ADC_3_PORT, ADC_3_PIN, GPIO_Mode_AN);
    Bsp_Gpio_Mode1(ADC_5_PORT, ADC_5_PIN, GPIO_Mode_AN);
    Bsp_Gpio_Mode1(ADC_6_PORT, ADC_6_PIN, GPIO_Mode_AN);

    Bsp_Gpio_Mode1(ADC_00_PORT, ADC_00_PIN, GPIO_Mode_AN);
    Bsp_Gpio_Mode1(ADC_11_PORT, ADC_11_PIN, GPIO_Mode_AN);
    Bsp_Gpio_Mode1(ADC_22_PORT, ADC_22_PIN, GPIO_Mode_AN);
    Bsp_Gpio_Mode1(ADC_33_PORT, ADC_33_PIN, GPIO_Mode_AN);
    Bsp_Gpio_Mode1(ADC_44_PORT, ADC_44_PIN, GPIO_Mode_AN);

    return;
}

#ifdef __cplusplus
}
#endif
