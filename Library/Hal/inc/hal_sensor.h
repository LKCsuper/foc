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
 * @description: 初始化当前编码器硬件接口（MT6701 使用 I2C2）
 * @return {*}
 * @author: lkc
 */
void Hal_Encoder_Init(void);

/**
 * @description: 读取最近一次完整编码器角度快照
 * @return {uint16_t} 当前计数
 * @author: lkc
 */
uint16_t Hal_Encoder_GetCount(void);

/**
 * @description: 发布编码器角度快照
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
 * @description: 读取位置传感器 I2C 寄存器。
 * @param {uint8_t} address_7bit 7 位设备地址
 * @param {uint8_t} reg          寄存器地址
 * @param {uint8_t *} value      输出数据，不可为空
 * @return {bool} true=读取成功
 */
bool Hal_Encoder_I2cReadRegister(uint8_t address_7bit, uint8_t reg, uint8_t *value);

void Hal_Encoder_SsiInit(void);
bool Hal_Encoder_SsiReadFrame(uint32_t *frame);

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
