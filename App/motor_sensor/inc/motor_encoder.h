/*
 * @Description: AB 增量编码器（TIM4 正交 4 倍频）
 * @Author: lkc
 */
#ifndef _MOTOR_ENCODER_H
#define _MOTOR_ENCODER_H
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
 * @description: 初始化 AB 编码器定时器
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Init(void);

/**
 * @description: 读取正交解码计数值
 * @return {uint16_t} 当前计数，范围 [0, 线数×4)
 * @author: lkc
 */
uint16_t Motor_Encoder_GetCount(void);

/**
 * @description: 把当前计数清零，并清除电角度偏置
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Reset(void);

/**
 * @description: 机械角度
 * @return {float} 机械角度 (rad)，范围 [0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetMechAngle(void);

/**
 * @description: 电角度
 * @return {float} 电角度 (rad)，范围 [0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetElecAngle(void);

/**
 * @description: 把当前位置记为电角度 0，用于无 Z 信号时的零点标定
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Detect(void);

#ifdef __cplusplus
}
#endif
#endif
