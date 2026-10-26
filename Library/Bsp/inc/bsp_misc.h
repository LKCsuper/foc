
/*
 * @Description: 时钟与系统基础初始化接口声明
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
/**
 * @description: 时钟初始化（HSE 晶振 + PLL）
 * @param {ULONG} ulPllm PLL 输入分频因子 M
 * @param {ULONG} ulPlln PLL 倍频因子 N
 * @param {ULONG} ulPllp 系统时钟分频因子 P
 * @param {ULONG} ulPllq 外设时钟分频因子 Q
 * @return {void} 无
 */
VOID Bsp_Rcu_Init(ULONG ulPllm, ULONG ulPlln, ULONG ulPllp, ULONG ulPllq);

/**
 * @description: SysTick 系统节拍初始化（FreeRTOS tick 源）
 * @param {ULONG} fzv tick 频率 (Hz)
 * @return {void} 无
 */
VOID Bsp_Sys_Init(ULONG fzv);

/**
 * @description: 使用内部 HSI 配置系统时钟
 * @param {ULONG} ulPllm PLL 输入分频因子 M
 * @param {ULONG} ulPlln PLL 倍频因子 N
 * @param {ULONG} ulPllp 系统时钟分频因子 P
 * @param {ULONG} ulPllq 外设时钟分频因子 Q
 * @return {void} 无
 */
VOID HSI_SetSysClock(ULONG ulPllm, ULONG ulPlln, ULONG ulPllp, ULONG ulPllq);

/**
 * @description: 使用外部 HSE 配置系统时钟
 * @param {ULONG} ulPllm PLL 输入分频因子 M
 * @param {ULONG} ulPlln PLL 倍频因子 N
 * @param {ULONG} ulPllp 系统时钟分频因子 P
 * @param {ULONG} ulPllq 外设时钟分频因子 Q
 * @return {void} 无
 */
VOID HSE_SetSysClock(ULONG ulPllm, ULONG ulPlln, ULONG ulPllp, ULONG ulPllq);

/**
 * @description: 软件复位设备
 * @return {void} 无
 */
VOID Bsp_Misc_Reset(VOID);
#ifdef __cplusplus
}
#endif

#endif
