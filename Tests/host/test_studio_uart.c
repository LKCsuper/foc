/* Studio UART 主机测试：整帧拒绝、环绕与 IRQ 掩码恢复 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "ringbuf.h"
typedef uint8_t UCHAR;
typedef unsigned long ULONG;
static ringbuf_t s_usart_tx;
static bool sgbUsartTxDmaReady=true;
static ULONG sgulUsartTxBytes,sgulUsartTxDrops;
static uint32_t mask;
static unsigned polls;
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：读取测试模拟的全局中断屏蔽位。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 mask
 */
static uint32_t __get_PRIMASK(void) { return mask; }
/**
 * @description: 主机测试替身：模拟屏蔽中断并更新测试记录。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void __disable_irq(void) { mask=1; }
/**
 * @description: 主机测试替身：模拟恢复中断并更新测试记录。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void __enable_irq(void) { mask=0; }
/**
 * @description: 主机测试替身：任务侧补看 TX DMA 完成标志，并在队列非空时尝试启动下一段。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void Bsp_Usart_TxDmaPollLocked(void) { polls++; }
/**
 * @description: 主机测试替身：启动环形缓冲区中的下一段 DMA 发送。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void Bsp_Usart_TxDmaStartLocked(void) {}
#include "studio_uart_under_test.inc"
/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    uint8_t storage[128],data[120],out[120];
    unsigned i;
    for(i=0;i<sizeof(data);i++)data[i]=(uint8_t)i;
    assert(ringbuf_init(&s_usart_tx,storage,sizeof(storage)));
    assert(ringbuf_write(&s_usart_tx,data,120)==120);
    assert(!Bsp_Usart_TryWriteFrame(data,82));assert(ringbuf_len(&s_usart_tx)==120);
    assert(sgulUsartTxDrops==82&&polls==1&&mask==0);
    assert(ringbuf_read(&s_usart_tx,out,120)==120);
    assert(Bsp_Usart_TryWriteFrame(data,82));assert(sgulUsartTxBytes==82);
    assert(ringbuf_read(&s_usart_tx,out,82)==82);assert(memcmp(out,data,82)==0);
    mask=1;assert(Bsp_Usart_TryWriteFrame(data,10));assert(mask==1);
    assert(!Bsp_Usart_TryWriteFrame(data,97));assert(!Bsp_Usart_TryWriteFrame(NULL,1));
    sgbUsartTxDmaReady=false;assert(!Bsp_Usart_TryWriteFrame(data,1));
    puts("studio UART: whole-frame rejection, wrapping, bounded size and IRQ-mask restore PASS");
    return 0;
}
