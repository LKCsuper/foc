/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-03 16:19:58
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-03 16:32:05
 * @detail: 
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** jscop缓冲区大小 */
#define JSCOP_BUFF_SIZE  1024
/** JSCOP 缓冲区 */
UCHAR g_ucJscopBuff[JSCOP_BUFF_SIZE];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
VOID RTT_Debug_Init(VOID)
{
    /* 配置上行通道 1，缓冲区g_ucJscopBuff */
    //#define SEGGER_RTT_MODE_NO_BLOCK_SKIP         (0)     // 如果缓冲区放不下当前帧数据，则舍弃当前帧
    //#define SEGGER_RTT_MODE_NO_BLOCK_TRIM         (1)     // 如果缓冲区放不下当前帧数据，则缓冲区能放几个当前帧byte就放几个
    //#define SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL    (2)     // 如果缓冲区放不下当前帧数据，则堵塞直到缓冲区能放下当前帧所有数据
    SEGGER_RTT_ConfigUpBuffer(1, "JScope_u4i4", g_ucJscopBuff, JSCOP_BUFF_SIZE, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    return;
}
#ifdef __cplusplus
}
#endif


