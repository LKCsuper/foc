/*
 * @Description: 三相霍尔读取与霍尔表标定
 * @Author: lkc
 */
#ifndef _MOTOR_HALL_H
#define _MOTOR_HALL_H
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
 * @description: 读取三相霍尔状态（bit0=A, bit1=B, bit2=C）
 * @param {bool} is_second_motor  true 时读第二路霍尔口
 * @param {int} samples           额外滤波次数，0 表示只采一次
 * @return {int} 霍尔状态 0~7
 * @author: lkc
 */
int Motor_Hall_Read(bool is_second_motor, int samples);

/**
 * @description: 开环转一圈，标定霍尔状态与电角度对应表
 * @param {float} current        对齐电流 (A)，建议用峰值电流 / 3
 * @param {uint8_t} *hall_table  输出 8 项霍尔表
 * @param {bool} *result         true 表示标定成功
 * @return {int} 故障码
 * @author: lkc
 */
int Motor_Hall_Detect(float current, uint8_t *hall_table, bool *result);

#ifdef __cplusplus
}
#endif
#endif
