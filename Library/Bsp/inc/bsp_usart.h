
/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 10:03:36
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-08 19:11:52
 */
#ifndef _BSP_USART_H
#define _BSP_USART_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DEBUG   1

#if 0
#ifdef  DEBUG
    #define PRINTF(format,args...)    printf(format,##args)
#else   
    #define PRINTF(format,args...)     ((void)0)
#endif
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Bsp_Usart_DebugInit(uint32_t ulBound);
void Bsp_Usart_PutString(const char *str);
uint32_t Bsp_Usart_GetChar(uint8_t* key);
void Bsp_Usart_PutChar(uint8_t c);
int fputc(int ch, FILE *f);
void Bsp_Usart_U3Handler(void);

#ifdef __cplusplus
}
#endif

#endif
