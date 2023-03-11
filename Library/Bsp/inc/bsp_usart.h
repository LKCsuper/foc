
/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 10:03:36
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-10 23:25:41
 */
#ifndef _BSP_USART_H
#define _BSP_USART_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USART_NUM						   (3)

#if (1 == USART_NUM)
#define DEBUG_USART                        USART1
#define DEBUG_USART_TX_AF                  GPIO_AF_USART1
#define DEBUG_USART_RX_AF                  GPIO_AF_USART1
#define DEBUG_USART_CLK_EN()               RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE)
#define DEBUG_USART_TX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)		
#define DEBUG_USART_RX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define DEBUG_USART_TX_PIN                 GPIO_Pin_9
#define DEBUG_USART_TX_GPIO_PORT           GPIOA
#define DEBUG_USART_TX_SOURCE              GPIO_PinSource9
#define DEBUG_USART_RX_PIN                 GPIO_Pin_10
#define DEBUG_USART_RX_GPIO_PORT           GPIOA
#define DEBUG_USART_RX_SOURCE              GPIO_PinSource10
#define DEBUG_USART_IRQn                   USART1_IRQn
#elif (3 == USART_NUM)
#define DEBUG_USART                        USART3
#define DEBUG_USART_TX_AF                  GPIO_AF_USART3
#define DEBUG_USART_RX_AF                  GPIO_AF_USART3
#define DEBUG_USART_CLK_EN()               RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE)
#define DEBUG_USART_TX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)		
#define DEBUG_USART_RX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define DEBUG_USART_TX_PIN                 GPIO_Pin_10
#define DEBUG_USART_TX_SOURCE              GPIO_PinSource10
#define DEBUG_USART_TX_GPIO_PORT           GPIOB
#define DEBUG_USART_RX_PIN                 GPIO_Pin_11
#define DEBUG_USART_RX_SOURCE              GPIO_PinSource11
#define DEBUG_USART_RX_GPIO_PORT           GPIOB
#define DEBUG_USART_IRQn                   USART3_IRQn
#elif (4 == USART_NUM)
#define DEBUG_USART                        UART4
#define DEBUG_USART_TX_AF                  GPIO_AF_UART4
#define DEBUG_USART_RX_AF                  GPIO_AF_UART4
#define DEBUG_USART_CLK_EN()               RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE)
#define DEBUG_USART_TX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)		
#define DEBUG_USART_RX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define DEBUG_USART_TX_PIN                 GPIO_Pin_10
#define DEBUG_USART_TX_SOURCE              GPIO_PinSource10
#define DEBUG_USART_TX_GPIO_PORT           GPIOC
#define DEBUG_USART_RX_PIN                 GPIO_Pin_11
#define DEBUG_USART_RX_SOURCE              GPIO_PinSource11
#define DEBUG_USART_RX_GPIO_PORT           GPIOC
#define DEBUG_USART_IRQn                   UART4_IRQn
#endif

/* 如果\n有前缀,例如windows是\r\n */
#define ENTER_PRI                           '\r'

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
VOID  Bsp_Usart_DebugInit(ULONG ulBound);
VOID  Bsp_Usart_PutString(const CHAR *cStr);
ULONG Bsp_Usart_GetChar(UCHAR* cKey);
VOID  Bsp_Usart_PutChar(UCHAR cKey);
VOID  Bsp_Usart_U3Handler(VOID);
int fputc(int ch, FILE *f);

#ifdef __cplusplus
}
#endif

#endif
