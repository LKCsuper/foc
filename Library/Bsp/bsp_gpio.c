/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-10-09 21:37:15
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
VOID Bsp_Gpio_Mode(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, GPIOMode_TypeDef GPIO_Mode)
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
 * @description: 输出模式 推挽
 * @detail: 
 * @param {GPIO_TypeDef*} GPIOx
 * @param {GPIO_InitTypeDef*} GPIO_InitStruct
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Gpio_ModeAF(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, uint16_t GPIO_PinSource, uint8_t GPIO_AF)
{
	/* gpio时钟默认前边初始化 */

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	/* 定时器复用使用浮空.其他复用未知 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOx, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOx, GPIO_PinSource, GPIO_AF);

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
	Bsp_Gpio_Mode(RED_PORT, RED_PIN, GPIO_Mode_OUT);
	Bsp_Gpio_Mode(GREEN_PORT, GREEN_PIN, GPIO_Mode_OUT);

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
	/* 复用pwm通道 */
	Bsp_Gpio_ModeAF(T_CHANNEL_PORT, T_CHANNEL_PIN, T_CHANNEL_SOURCE1, GPIO_AF_TIM1);
	Bsp_Gpio_ModeAF(T_CHANNEL_PORT, T_CHANNEL_PIN, T_CHANNEL_SOURCE2, GPIO_AF_TIM1);
	Bsp_Gpio_ModeAF(T_CHANNEL_PORT, T_CHANNEL_PIN, T_CHANNEL_SOURCE3, GPIO_AF_TIM1);

	/* 复用pwm互补通道 */
	Bsp_Gpio_ModeAF(T_CHANNEL_N_PORT, T_CHANNEL_N_PIN, T_CHANNEL_N_SOURCE1, GPIO_AF_TIM1);
	Bsp_Gpio_ModeAF(T_CHANNEL_N_PORT, T_CHANNEL_N_PIN, T_CHANNEL_N_SOURCE2, GPIO_AF_TIM1);
	Bsp_Gpio_ModeAF(T_CHANNEL_N_PORT, T_CHANNEL_N_PIN, T_CHANNEL_N_SOURCE3, GPIO_AF_TIM1);

	return;
}

/**
 * @description: 
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID Bsp_Usart3_Gpio(VOID)
{
	// TODO 复用source不能| ,PIN 和 PIN SOURCE有区别,一个是引脚 一个是复用资源
	Bsp_Gpio_ModeAF(DEBUG_PORT, DEBUG_PIN, DEBUG_USART_TX_SOURCE, DEBUG_USART_TX_AF);
	Bsp_Gpio_ModeAF(DEBUG_PORT, DEBUG_PIN, DEBUG_USART_RX_SOURCE, DEBUG_USART_RX_AF);

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
	/* 初始化时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Bsp_Gpio_Mode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, GPIO_Mode_OUT);
	// HW_SHUTDOWN_HOLD_ON();

	// vTaskDelay(1000);

	// Bsp_Gpio_Led();
	// GREEN_ON();
	// RED_ON();

	// Bsp_Gpio_Mode(GATE_PORT, GATE_PIN, GPIO_Mode_OUT);

	// Bsp_Gpio_Mode(DCCAL_PORT, DCCAL_PIN, GPIO_Mode_OUT);
	// palClearPad(DCCAL_PORT, DCCAL_PIN);
	// ENABLE_GATE();

	/* 串口gpio */
	Bsp_Usart3_Gpio();

	// /* 定时器gpio */                                   
	Bsp_Tim1_Gpio();

	// /* 编码器 霍尔 输入 */
	/* 初始化霍尔 */
	// Bsp_Gpio_Mode(HW_HALL_A_PORT, HW_HALL_A_PIN, GPIO_Mode_IN);
	// Bsp_Gpio_Mode(HW_HALL_B_PORT, HW_HALL_B_PIN, GPIO_Mode_IN);
	// Bsp_Gpio_Mode(HW_HALL_C_PORT, HW_HALL_C_PIN, GPIO_Mode_IN);

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = HW_HALL_A_PIN | HW_HALL_B_PIN | HW_HALL_C_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(HW_HALL_A_PORT, &GPIO_InitStructure);
	
	/*
	 * 保持 3e5d564 已上板验证过的电气状态：PC13 不配置为输出。
	 * 当前板型对该引脚的真实用途尚未重新核实，主动拉高/拉低可能改变
	 * 外部模拟开关或驱动相关电路。确认原理图前只保留输入态。
	 */

	// /* Fault */
	// Bsp_Gpio_Mode(PHASE_FILTER_PORT, PHASE_FILTER_PIN, GPIO_Mode_IN);

	/* ADC */
	Bsp_Gpio_Mode(ADC_0_PORT, ADC_0_PIN, GPIO_Mode_AN);
	Bsp_Gpio_Mode(ADC_1_PORT, ADC_1_PIN, GPIO_Mode_AN);
	Bsp_Gpio_Mode(ADC_2_PORT, ADC_2_PIN, GPIO_Mode_AN);
	Bsp_Gpio_Mode(ADC_3_PORT, ADC_3_PIN, GPIO_Mode_AN);
	Bsp_Gpio_Mode(ADC_5_PORT, ADC_5_PIN, GPIO_Mode_AN);
	Bsp_Gpio_Mode(ADC_6_PORT, ADC_6_PIN, GPIO_Mode_AN);

	Bsp_Gpio_Mode(ADC_00_PORT, ADC_00_PIN, GPIO_Mode_AN);
	Bsp_Gpio_Mode(ADC_11_PORT, ADC_11_PIN, GPIO_Mode_AN);
	Bsp_Gpio_Mode(ADC_22_PORT, ADC_22_PIN, GPIO_Mode_AN);
	Bsp_Gpio_Mode(ADC_33_PORT, ADC_33_PIN, GPIO_Mode_AN);
	Bsp_Gpio_Mode(ADC_44_PORT, ADC_44_PIN, GPIO_Mode_AN);

	return;
}

#ifdef __cplusplus
}
#endif
