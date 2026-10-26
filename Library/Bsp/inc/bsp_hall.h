/* 三相霍尔传感器板级接口声明 */
#ifndef __BSP_HALL_H__
#define __BSP_HALL_H__
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
/**
 * @description: 初始化三相霍尔传感器（TIM3 霍尔接口模式）
 * @return {void} 无
 */
VOID Bsp_Hall_Init(VOID);
#ifdef __cplusplus
}
#endif
#endif
