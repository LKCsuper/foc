/*
 * @Description: 
 * @Date: 2023-02-18 23:51:32
 * @LastEditTime: 2023-03-15 22:55:25
 * @FilePath: \foc\Library\Bsp\inc\bsp_dma.h
 */
#ifndef _BSP_DMA_H
#define _BSP_DMA_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* 串口接收buff大小 */
#define USART_RX_BUF_SIZE                  (128U)

#if (4 == USART_NUM)
#define DEBUG_USART_RX_DMA_STREAM          DMA1_Stream2
#define DEBUG_USART_RX_DMA_CHANNEL         DMA_Channel_4
#define DEBUG_USART_RX_DMA_FLAGS           (DMA_FLAG_TCIF2 | DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2)
#else
#define DEBUG_USART_RX_DMA_STREAM          DMA1_Stream1
#define DEBUG_USART_RX_DMA_CHANNEL         DMA_Channel_4
#define DEBUG_USART_RX_DMA_FLAGS           (DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1)
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
VOID Bsp_Dma_AdcSample(VOID);
VOID Bsp_Dma_UsartRx(VOID);
VOID Bsp_Dma_Init(VOID);

#ifdef __cplusplus
}
#endif

#endif
