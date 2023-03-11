/*
 * @Description: 
 * @Date: 2023-02-18 23:51:32
 * @LastEditTime: 2023-03-11 22:40:26
 * @FilePath: \foc\Library\Bsp\inc\bsp_dma.h
 */
#ifndef _BSP_GPIO_H
#define _BSP_GPIO_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* LED */
#define GREEN_PORT              GPIOB
#define GREEN_PIN               GPIO_Pin_0
#define GREEN_RCC               RCC_AHB1Periph_GPIOB

#define RED_PORT                GPIOB
#define RED_PIN                 GPIO_Pin_1
#define RED_RCC                 RCC_AHB1Periph_GPIOB

#define GREEN_ON()              GPIO_SetBits(GREEN_PORT, GREEN_PIN)
#define GREEN_OFF()             GPIO_ResetBits(GREEN_PORT, GREEN_PIN)

#define RED_ON()                GPIO_SetBits(RED_PORT, RED_PIN)
#define RED_OFF()               GPIO_ResetBits(RED_PORT, RED_PIN)

/* ENABLE GATE */
#define GATE_PORT               GPIOB
#define GATE_PIN                GPIO_Pin_5

/* DCCAL */
#define DCCAL_PORT              GPIOD
#define DCCAL_PIN               GPIO_Pin_2

/* TIM1 CHANNEL */
#define T_MOTOR1_RCC            RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB
#define T_CHANNEL_PORT          GPIOA
#define T_CHANNEL1_PIN          GPIO_Pin_8
#define T_CHANNEL2_PIN          GPIO_Pin_9
#define T_CHANNEL3_PIN          GPIO_Pin_10

#define T_CHANNEL_N_PORT        GPIOB
#define T_CHANNEL1_N_PIN        GPIO_Pin_13
#define T_CHANNEL2_N_PIN        GPIO_Pin_14
#define T_CHANNEL3_N_PIN        GPIO_Pin_15

/* 霍尔 */
#define HW_HALL_A_PORT		    GPIOC
#define HW_HALL_A_PIN		    GPIO_Pin_6
#define HW_HALL_B_PORT		    GPIOC
#define HW_HALL_B_PIN		    GPIO_Pin_7
#define HW_HALL_C_PORT		    GPIOC
#define HW_HALL_C_PIN		    GPIO_Pin_8

/* 编码器 */
#define HW_ENC_A_PORT		    GPIOC
#define HW_ENC_A_PIN		    GPIO_Pin_6
#define HW_ENC_B_PORT		    GPIOC
#define HW_ENC_B_PIN		    GPIO_Pin_7
#define HW_ENC_C_PORT		    GPIOC
#define HW_ENC_C_PIN		    GPIO_Pin_8

/* Phase filters */
#define PHASE_FILTER_PORT		GPIOC
#define PHASE_FILTER_PIN		GPIO_Pin_13

/* Fault pin */
#define FAULT_PORT		        GPIOB
#define FAULT_PIN		        GPIO_Pin_7

/* ADC */
// TODO 名字后期再定义 具体有电流啥的
#define ADC_0_PORT              GPIOA
#define ADC_0_PIN               GPIO_Pin_0
#define ADC_1_PORT              GPIOA
#define ADC_1_PIN               GPIO_Pin_1
#define ADC_2_PORT              GPIOA
#define ADC_2_PIN               GPIO_Pin_2
#define ADC_3_PORT              GPIOA
#define ADC_3_PIN               GPIO_Pin_3
#define ADC_5_PORT              GPIOA
#define ADC_5_PIN               GPIO_Pin_5
#define ADC_6_PORT              GPIOA
#define ADC_6_PIN               GPIO_Pin_6

#define ADC_00_PORT              GPIOC
#define ADC_00_PIN               GPIO_Pin_0
#define ADC_11_PORT              GPIOC
#define ADC_11_PIN               GPIO_Pin_1
#define ADC_22_PORT              GPIOC
#define ADC_22_PIN               GPIO_Pin_2
#define ADC_33_PORT              GPIOC
#define ADC_33_PIN               GPIO_Pin_3
#define ADC_44_PORT              GPIOC
#define ADC_44_PIN               GPIO_Pin_4

#define palSetPad(x, y)		    GPIO_SetBits(x, y);
#define palClearPad(x, y)	    GPIO_ResetBits(x, y);
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Bsp_Gpio_Led(void);
void Bsp_Gpio_Init(void);

#ifdef __cplusplus
}
#endif

#endif
