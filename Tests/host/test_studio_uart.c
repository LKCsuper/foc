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
static uint32_t __get_PRIMASK(void) { return mask; }
static void __disable_irq(void) { mask=1; }
static void __enable_irq(void) { mask=0; }
static void Bsp_Usart_TxDmaPollLocked(void) { polls++; }
static void Bsp_Usart_TxDmaStartLocked(void) {}
#include "studio_uart_under_test.inc"
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
