/*
 * @Description: BM75 MT6701 SSI 板级驱动
 * @Author: lkc
 */
#ifndef BSP_MT6701_SSI_H
#define BSP_MT6701_SSI_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 初始化 SPI1，供 MT6701 SSI 读数。
 * @detail: PA5=CLK、PA6=DO、PA7=MOSI、PB11=CSN。时钟约 5.25 MHz，mode 1。不改 TIM1/ADC。
 * @return {void} 无
 * @author: lkc
 */
void Bsp_MT6701_SsiInit(void);

/**
 * @description: 读取一帧 24 位 SSI 数据。
 * @param {uint32_t *} frame  输出原始帧，不可为空
 * @return {bool} true=三字节都读到
 * @author: lkc
 */
bool Bsp_MT6701_SsiReadFrame(uint32_t *frame);
#ifdef __cplusplus
}
#endif
#endif
