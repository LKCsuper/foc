/*
 * @Description: 
 * @Date: 2023-02-18 23:51:32
 * @LastEditTime: 2023-02-25 10:51:22
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

#define palSetPad(x, y)		GPIO_SetBits(x, y);
#define palClearPad(x, y)	GPIO_ResetBits(x, y);
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
