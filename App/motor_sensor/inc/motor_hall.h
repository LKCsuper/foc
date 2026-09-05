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
#include <stdint.h>
#include <stdbool.h>
#include "motor_param.h"
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
 * @description: 霍尔标定是否占用 FOC 中断路径
 * @return {bool} true 表示标定中，motor_it 走专用电流环
 * @author: lkc
 */
bool Motor_Hall_DetectIsActive(void);

/**
 * @description: 中止正在进行的霍尔标定（olstop 或故障时调用）
 * @return {*}
 * @author: lkc
 */
void Motor_Hall_DetectAbort(void);

/**
 * @description: 霍尔标定期间 ISR 读取本周期电角度与 d/q 电流目标
 * @param {MOTOR_PARAM_S} *m         电机运行时参数
 * @param {float} *theta_rad         输出电角度 (rad)
 * @param {float} *id_target         输出 D 轴目标电流 (A)
 * @param {float} *iq_target         输出 Q 轴目标电流 (A)
 * @return {bool} true 表示标定中且目标有效
 * @author: lkc
 */
bool Motor_Hall_DetectGetTargets(MOTOR_PARAM_S *m, float *theta_rad,
                                 float *id_target, float *iq_target);

/**
 * @description: 开环转一圈，标定霍尔状态与电角度对应表
 * @detail: 通过 MOTOR_PARAM_S 与 motor_it 电流环驱动电机；结果写入 hall_table 输出缓冲。
 * @param {float} current        对齐电流 (A)，建议用峰值电流 / 3
 * @param {uint8_t} *hall_table  输出 8 项霍尔表，255 表示无效扇区
 * @param {bool} *result         true 表示标定成功（6 个有效扇区）
 * @return {int} 故障码，FAULT_CODE_NONE 表示无故障
 * @author: lkc
 */
int Motor_Hall_Detect(float current, uint8_t *hall_table, bool *result);

#ifdef __cplusplus
}
#endif
#endif
