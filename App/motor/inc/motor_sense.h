/*
 * @Description: 电流/相电压/母线采样换算（ADC → 物理量）
 * @Author: lkc
 */
#ifndef _MOTOR_SENSE_H
#define _MOTOR_SENSE_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 从 HAL 读取 ADC，换算三相电流/相电压/母线电压写入运行时
 * @detail: 每个 ADC-DMA 回调调用一次。vb/vc 通道与相序非一一对应，映射在本模块内。
 * @param {MOTOR_PARAM_S} *m  电机运行时参数
 * @return {*}
 * @author: lkc
 */
void Motor_Sense_Update(MOTOR_PARAM_S *m);

#ifdef __cplusplus
}
#endif
#endif
