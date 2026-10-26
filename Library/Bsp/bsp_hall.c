/* 三相霍尔传感器板级初始化（TIM3 霍尔接口模式） */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
#define HALL_TIM                TIM3
#define HALL_TIM_CLK            RCC_APB1Periph_TIM3

#define HALL_CH1_PIN            GPIO_Pin_6
#define HALL_CH1_GPIO_PORT      GPIOC
#define HALL_CH1_GPIO_CLK       RCC_AHB1Periph_GPIOC
#define HALL_CH1_SOURCE         GPIO_PinSource6
#define HALL_CH1_AF             GPIO_AF_TIM3

#define HALL_CH2_PIN            GPIO_Pin_7
#define HALL_CH2_GPIO_PORT      GPIOC
#define HALL_CH2_GPIO_CLK       RCC_AHB1Periph_GPIOC
#define HALL_CH2_SOURCE         GPIO_PinSource7
#define HALL_CH2_AF             GPIO_AF_TIM3

#define HALL_CH3_PIN            GPIO_Pin_8
#define HALL_CH3_GPIO_PORT      GPIOC
#define HALL_CH3_GPIO_CLK       RCC_AHB1Periph_GPIOC
#define HALL_CH3_SOURCE         GPIO_PinSource8
#define HALL_CH3_AF             GPIO_AF_TIM3
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 霍尔传感器初始化
 * @detail: 这种方式只适合高级定时器，但是定时器3并不是高级定时器
 * @return {void} 无
 * @author: lkc
 */
VOID Bsp_Hall_Init(VOID)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_HALLTimeBaseInitStructure;
    TIM_ICInitTypeDef       TIM_HALLICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = HALL_CH1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HALL_CH1_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = HALL_CH2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HALL_CH2_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = HALL_CH3_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HALL_CH3_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(HALL_CH1_GPIO_PORT, HALL_CH1_SOURCE, HALL_CH1_AF);
    GPIO_PinAFConfig(HALL_CH2_GPIO_PORT, HALL_CH2_SOURCE, HALL_CH2_AF);
    GPIO_PinAFConfig(HALL_CH3_GPIO_PORT, HALL_CH3_SOURCE, HALL_CH3_AF);
    
    TIM_DeInit(HALL_TIM);
    TIM_TimeBaseStructInit(&TIM_HALLTimeBaseInitStructure); 
    TIM_HALLTimeBaseInitStructure.TIM_Period = 0xffffffff;
    TIM_HALLTimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(HALL_TIM,&TIM_HALLTimeBaseInitStructure);
    
    TIM_ICStructInit(&TIM_HALLICInitStructure);
    TIM_HALLICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_HALLICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    TIM_HALLICInitStructure.TIM_ICFilter = 0x0f;
    TIM_ICInit(HALL_TIM,&TIM_HALLICInitStructure);
    
    TIM_PrescalerConfig(HALL_TIM, (u16)0, TIM_PSCReloadMode_Immediate);     /* 通过产生一个更新事件来立即重载预分频寄存器和重复计数寄存器 */
    TIM_InternalClockConfig(HALL_TIM);
    TIM_SelectHallSensor(HALL_TIM, ENABLE);
    TIM_SelectInputTrigger(HALL_TIM, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(HALL_TIM,TIM_SlaveMode_Reset);
    TIM_ClearFlag(HALL_TIM, TIM_FLAG_Update|TIM_FLAG_CC1|TIM_FLAG_CC2|TIM_FLAG_CC3|TIM_FLAG_CC4|
                            TIM_FLAG_Trigger|TIM_FLAG_CC1OF|TIM_FLAG_CC2OF|TIM_FLAG_CC3OF|TIM_FLAG_CC4OF);
    TIM_ITConfig(HALL_TIM, TIM_IT_CC1, ENABLE);
    TIM_SetCounter(HALL_TIM, 0);
    TIM_Cmd(HALL_TIM, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return;
}

#ifdef __cplusplus
}
#endif