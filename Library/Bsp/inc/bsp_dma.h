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
