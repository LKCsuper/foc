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
/* 串口接收缓冲区大小（须能容纳一次粘贴的多行命令；过小会丢字节） */
#define USART_RX_BUF_SIZE                  (1024U)

/* USART RX DMA destination buffer, defined in bsp_dma.c. */
extern UCHAR gucURxBuf[USART_RX_BUF_SIZE];

/*
 * 串口发送软件缓冲区大小。
 *
 * printf 产生的字符先写入这个环形缓冲区，再由 DMA 自动搬到串口数据寄存器。
 * 这样命令任务不需要等待每个字符发送完成，电机运行时也不会因为 30kHz FOC
 * 中断反复抢占而让 olstatus 等长输出明显卡顿。
 *
 * 环形缓冲区使用按位与完成下标回绕，因此大小必须是 2 的整数次幂。
 * 2048 字节可以完整容纳一次 olstatus 输出；若更长的输出写满缓冲区，普通
 * 任务会短暂等待 DMA 腾出空间，而中断上下文不会死等。
 */
#define USART_TX_BUF_SIZE                  (2048U)

#if ((USART_TX_BUF_SIZE & (USART_TX_BUF_SIZE - 1U)) != 0U)
#error "USART_TX_BUF_SIZE must be a power of two"
#endif

#if (4 == USART_NUM)
#define DEBUG_USART_RX_DMA_STREAM          DMA1_Stream2
#define DEBUG_USART_RX_DMA_CHANNEL         DMA_Channel_4
#define DEBUG_USART_RX_DMA_FLAGS           (DMA_FLAG_TCIF2 | DMA_FLAG_FEIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2)
#define DEBUG_USART_TX_DMA_STREAM          DMA1_Stream4
#define DEBUG_USART_TX_DMA_CHANNEL         DMA_Channel_4
#define DEBUG_USART_TX_DMA_IRQn            DMA1_Stream4_IRQn
#define DEBUG_USART_TX_DMA_TC_IT           DMA_IT_TCIF4
#define DEBUG_USART_TX_DMA_TC_FLAG         DMA_FLAG_TCIF4
#define DEBUG_USART_TX_DMA_FLAGS           (DMA_FLAG_TCIF4 | DMA_FLAG_FEIF4 | DMA_FLAG_DMEIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4)
#define DEBUG_USART_TX_DMA_CLK_EN()         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE)
#elif (3 == USART_NUM)
#define DEBUG_USART_RX_DMA_STREAM          DMA1_Stream1
#define DEBUG_USART_RX_DMA_CHANNEL         DMA_Channel_4
#define DEBUG_USART_RX_DMA_FLAGS           (DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1)
#define DEBUG_USART_TX_DMA_STREAM          DMA1_Stream3
#define DEBUG_USART_TX_DMA_CHANNEL         DMA_Channel_4
#define DEBUG_USART_TX_DMA_IRQn            DMA1_Stream3_IRQn
#define DEBUG_USART_TX_DMA_TC_IT           DMA_IT_TCIF3
#define DEBUG_USART_TX_DMA_TC_FLAG         DMA_FLAG_TCIF3
#define DEBUG_USART_TX_DMA_FLAGS           (DMA_FLAG_TCIF3 | DMA_FLAG_FEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3)
#define DEBUG_USART_TX_DMA_CLK_EN()         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE)
#else
#define DEBUG_USART_RX_DMA_STREAM          DMA1_Stream1
#define DEBUG_USART_RX_DMA_CHANNEL         DMA_Channel_4
#define DEBUG_USART_RX_DMA_FLAGS           (DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1)
#define DEBUG_USART_TX_DMA_STREAM          DMA2_Stream7
#define DEBUG_USART_TX_DMA_CHANNEL         DMA_Channel_4
#define DEBUG_USART_TX_DMA_IRQn            DMA2_Stream7_IRQn
#define DEBUG_USART_TX_DMA_TC_IT           DMA_IT_TCIF7
#define DEBUG_USART_TX_DMA_TC_FLAG         DMA_FLAG_TCIF7
#define DEBUG_USART_TX_DMA_FLAGS           (DMA_FLAG_TCIF7 | DMA_FLAG_FEIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_TEIF7 | DMA_FLAG_HTIF7)
#define DEBUG_USART_TX_DMA_CLK_EN()         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE)
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
