/*
 * @Description: BM75 外设口 I2C2 驱动（PB10/PB11）
 * @Author: lkc
 */
#ifndef BSP_I2C_H
#define BSP_I2C_H
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
 * @description: 初始化 BM75 外设口 I2C2。
 * @detail: PB10=SCL、PB11=SDA，开漏复用，400 kHz；不涉及电机 ADC/PWM 外设。
 * @return {void} 无
 */
void Bsp_I2c2_Init(void);

/**
 * @description: 从 7 位地址设备读取一个寄存器。
 * @param {uint8_t} address_7bit  7 位从机地址
 * @param {uint8_t} reg           寄存器地址
 * @param {uint8_t *} value       输出数据，不可为空
 * @return {bool} true=读取成功
 */
bool Bsp_I2c2_ReadRegister(uint8_t address_7bit, uint8_t reg, uint8_t *value);

#ifdef __cplusplus
}
#endif
#endif
