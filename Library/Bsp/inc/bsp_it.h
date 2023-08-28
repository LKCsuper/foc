#ifndef _BSP_IT_H
#define _BSp_IT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
VOID Bsp_Dma2Stream4_Handler(VOID);
VOID Bsp_Usart_Handler(VOID);
VOID Bsp_Tim3_IRQ(VOID);
VOID Bsp_Tim2_IRQ(VOID);
#ifdef __cplusplus
}
#endif
#endif
