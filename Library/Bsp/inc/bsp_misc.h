
/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 10:03:36
 * @LastEditors: lkc
 * @LastEditTime: 2023-02-25 22:15:24
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
void Bsp_Rcu_Init(u32 pllm,u32 plln,u32 pllp,u32 pllq);
void Bsp_Sys_Init(int fzv);
void HSI_SetSysClock(uint32_t m, uint32_t n, uint32_t p, uint32_t q);
#ifdef __cplusplus
}
#endif

#endif
