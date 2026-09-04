/*
 * @Description: 位置传感器硬件抽象（编码器 / 霍尔）
 * @Author: lkc
 */
#ifndef _HAL_SENSOR_H
#define _HAL_SENSOR_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 初始化 AB 正交编码器定时器
 * @return {*}
 * @author: lkc
 */
void Hal_Encoder_Init(void);

/**
 * @description: 读取编码器计数值
 * @return {uint16_t} 当前计数
 * @author: lkc
 */
uint16_t Hal_Encoder_GetCount(void);

/**
 * @description: 设置编码器计数值
 * @param {uint16_t} count 目标计数
 * @return {*}
 * @author: lkc
 */
void Hal_Encoder_SetCount(uint16_t count);

/**
 * @description: 每机械圈计数值（含倍频）
 * @return {uint32_t} count/rev
 * @author: lkc
 */
uint32_t Hal_Encoder_CountsPerRev(void);

/**
 * @description: 读取三相霍尔状态
 * @param {bool} is_second_motor  true 时读第二路霍尔
 * @param {int} samples           额外滤波次数，0 表示只采一次
 * @return {int} 霍尔状态 0~7（bit0=A, bit1=B, bit2=C）
 * @author: lkc
 */
int Hal_Hall_ReadRaw(bool is_second_motor, int samples);

#ifdef __cplusplus
}
#endif
#endif
