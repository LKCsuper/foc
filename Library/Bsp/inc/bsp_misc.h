
/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 10:03:36
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-11 00:04:35
 */
#ifndef _BSP_MISC_H
#define _BSP_MISC_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
VOID Bsp_Rcu_Init(ULONG ulPllm, ULONG ulPlln, ULONG ulPllp, ULONG ulPllq);
VOID Bsp_Sys_Init(ULONG fzv);
VOID HSI_SetSysClock(ULONG ulPllm, ULONG ulPlln, ULONG ulPllp, ULONG ulPllq);
VOID HSE_SetSysClock(ULONG ulPllm, ULONG ulPlln, ULONG ulPllp, ULONG ulPllq);
VOID Bsp_Misc_Reset(VOID);
#ifdef __cplusplus
}
#endif

#endif
