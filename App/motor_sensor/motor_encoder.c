/*
 * @Description: AB 增量编码器（TIM4 PB6/PB7 正交 4 倍频）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static float s_elec_offset_rad; /* 电角度零点偏置 (rad) */
static bool s_inverted;         /* true 时 A/B 方向取反 */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 将角度循环归一化到 [0, 2π)
 * @param {float} angle  待归一化角度 (rad)
 * @return {float} 归一化后的角度 (rad)
 * @author: lkc
 */
static float Motor_Encoder_Wrap2Pi(float angle)
{
    while (angle >= (2.0f * (float)M_PI)) {
        angle -= 2.0f * (float)M_PI;
    }
    while (angle < 0.0f) {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

/**
 * @description: 初始化 AB 编码器定时器
 * @detail: TIM4 CH1/CH2 = PB6/PB7，1024 线 × 4 倍频 = 4096 count/机械圈
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Init(void)
{
    s_elec_offset_rad = 0.0f;
    s_inverted = MCCONF_FOC_ENCODER_INVERTED;
    Bsp_Tim4_EncInit();
    Bsp_Tim4_EncSetCount(0U);
    PRINTF("Encoder AB: TIM4 PB6/PB7, %u count/rev\r\n", (unsigned)BSP_ENCODER_COUNTS);
}

/**
 * @description: 读取正交解码计数值
 * @return {uint16_t} 当前计数
 * @author: lkc
 */
uint16_t Motor_Encoder_GetCount(void)
{
    return Bsp_Tim4_EncGetCount();
}

/**
 * @description: 把当前计数清零，并清除电角度偏置
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Reset(void)
{
    Bsp_Tim4_EncSetCount(0U);
    s_elec_offset_rad = 0.0f;
}

/**
 * @description: 机械角度
 * @return {float} 机械角度 (rad)，范围 [0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetMechAngle(void)
{
    float turns;
    float angle;

    turns = (float)Bsp_Tim4_EncGetCount() / (float)BSP_ENCODER_COUNTS;
    if (s_inverted) {
        turns = 1.0f - turns;
    }
    angle = turns * 2.0f * (float)M_PI;
    return Motor_Encoder_Wrap2Pi(angle);
}

/**
 * @description: 电角度
 * @detail: elec = mech × 极对数 − 零点偏置
 * @return {float} 电角度 (rad)，范围 [0, 2π)
 * @author: lkc
 */
float Motor_Encoder_GetElecAngle(void)
{
    float pole_pairs = motor.ol_pole_pairs;

    if (pole_pairs < 1.0f) {
        pole_pairs = 1.0f;
    }
    return Motor_Encoder_Wrap2Pi(Motor_Encoder_GetMechAngle() * pole_pairs - s_elec_offset_rad);
}

/**
 * @description: 把当前位置记为电角度 0，用于无 Z 信号时的零点标定
 * @detail: 先用 D 轴把转子吸到电角度 0，再执行本函数。
 * @return {*}
 * @author: lkc
 */
void Motor_Encoder_Detect(void)
{
    s_elec_offset_rad = 0.0f;
    s_elec_offset_rad = Motor_Encoder_GetElecAngle();
    PRINTF("Encoder detect: count=%u mech=%.3f rad elec0=%.3f rad\r\n",
           (unsigned)Motor_Encoder_GetCount(),
           Motor_Encoder_GetMechAngle(),
           Motor_Encoder_GetElecAngle());
}

#ifdef __cplusplus
}
#endif
