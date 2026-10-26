
/*
 * @Description: 调试串口配置与接口声明
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 10:03:36
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-15 22:40:15
 */
#ifndef _BSP_USART_H
#define _BSP_USART_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if (1 == USART_NUM)
#define DEBUG_USART                        USART1
#define DEBUG_USART_TX_AF                  GPIO_AF_USART1
#define DEBUG_USART_RX_AF                  GPIO_AF_USART1
#define DEBUG_USART_CLK_EN()               RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE)
#define DEBUG_USART_TX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)		
#define DEBUG_USART_RX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define DEBUG_USART_IRQn                   USART1_IRQn
#elif (3 == USART_NUM)
#define DEBUG_USART                        USART3
#define DEBUG_USART_TX_AF                  GPIO_AF_USART3
#define DEBUG_USART_RX_AF                  GPIO_AF_USART3
#define DEBUG_USART_CLK_EN()               RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE)
#define DEBUG_USART_TX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)		
#define DEBUG_USART_RX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define DEBUG_USART_IRQn                   USART3_IRQn
#elif (4 == USART_NUM)
#define DEBUG_USART                        UART4
#define DEBUG_USART_TX_AF                  GPIO_AF_UART4
#define DEBUG_USART_RX_AF                  GPIO_AF_UART4
#define DEBUG_USART_CLK_EN()               RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE)
#define DEBUG_USART_TX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)		
#define DEBUG_USART_RX_CLK_EN()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define DEBUG_USART_IRQn                   UART4_IRQn
#endif

/* 如果\n有前缀,例如windows是\r\n */
#define ENTER_PRI                           '\r'

/*
 * 调试串口波特率。
 *
 * 状态页和连续诊断输出量较大，使用 2 Mbaud 缩短 DMA 队列占用时间。
 * 当前 UART4 的 APB1 时钟为 42 MHz，2 Mbaud 可精确分频。下载固件后，
 * 上位机必须同步设为 2000000、8N1、无流控。
 */
#define DEBUG_USART_BAUD                    2000000U

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 调试串口初始化
 * @param {ULONG} ulBound 波特率
 * @return {void} 无
 */
VOID  Bsp_Usart_DebugInit(ULONG ulBound);

/**
 * @description: 串口输出字符串
 * @param {const CHAR *} cStr 以 '\0' 结尾的字符串
 * @return {void} 无
 */
VOID  Bsp_Usart_PutString(const CHAR *cStr);

/**
 * @description: 从串口读取一个字符
 * @param {UCHAR *} cKey 输出字符
 * @return {ULONG} SUCCESS=读到，ERROR=无数据
 */
ULONG Bsp_Usart_GetChar(UCHAR* cKey);

/**
 * @description: 串口输出一个字符
 * @param {UCHAR} cKey 待发送字符
 * @return {void} 无
 */
VOID  Bsp_Usart_PutChar(UCHAR cKey);

/**
 * @description: 串口输出一段数据
 * @param {const UCHAR *} data 待发送数据
 * @param {ULONG} len 字节数
 * @return {ULONG} 已发送字节数
 */
ULONG Bsp_Usart_Write(const UCHAR *data, ULONG len);

/**
 * @description: 任务侧整帧非阻塞入队，最多 96 B；空间不足整帧放弃，不等待 DMA。
 * @param {const UCHAR *} data 数据缓冲
 * @param {ULONG} len 缓冲长度或元素数量，以接口中的缓冲类型为准
 * @return {bool} true=整帧已入队；false=参数非法或剩余队列空间不足，不等待 DMA
 */
bool Bsp_Usart_TryWriteFrame(const UCHAR *data, ULONG len);

/**
 * @description: 等待发送队列清空并等待最后一个字符发出
 * @return {void} 无
 */
VOID  Bsp_Usart_Flush(VOID);

/**
 * @description: 调试串口空闲中断处理
 * @return {void} 无
 */
VOID  Bsp_Usart_Handler(VOID);

/**
 * @description: 调试串口发送 DMA 初始化
 * @return {void} 无
 */
VOID  Bsp_Usart_TxDmaInit(VOID);

/**
 * @description: 调试串口发送 DMA 完成中断处理
 * @return {void} 无
 */
VOID  Bsp_Usart_TxDmaHandler(VOID);

/**
 * @description: 任务侧补轮询 TX DMA 完成标志
 * @return {void} 无
 */
VOID  Bsp_Usart_TxDmaPoll(VOID);

/**
 * @description: 查询调试串口是否已切换到异步 DMA 发送
 * @return {bool} true=DMA 发送已就绪
 */
bool  Bsp_Usart_TxDmaIsReady(VOID);

/**
 * @description: 读取已进入 DMA 发送队列的累计字节数
 * @return {ULONG} 累计入队字节数
 */
ULONG Bsp_Usart_GetTxBytes(VOID);

/**
 * @description: 读取队列满时丢弃的累计字节数
 * @return {ULONG} 累计丢弃字节数
 */
ULONG Bsp_Usart_GetTxDrops(VOID);

/**
 * @description: 串口输出 N 个字符
 * @param {const UCHAR *} cStr 待发送数据
 * @param {ULONG} ulLen 字节数
 * @return {void} 无
 */
VOID Bsp_Usart_PutBuf(const UCHAR *cStr, ULONG ulLen);

/**
 * @description: 从串口读取数据到缓冲区
 * @param {UCHAR *} ucBuf 接收缓冲区
 * @param {ULONG} ulLen 期望读取字节数
 * @return {ULONG} 实际读取结果
 */
ULONG Bsp_Usart_GetBuf(UCHAR *ucBuf, ULONG ulLen);

/**
 * @description: 重定向 fputc 到调试串口（供 printf 使用）
 * @param {int} ch 待输出字符
 * @param {FILE *} f 标准输出句柄（未使用）
 * @return {int} 输出字符
 */
int fputc(int ch, FILE *f);

#ifdef __cplusplus
}
#endif

#endif
