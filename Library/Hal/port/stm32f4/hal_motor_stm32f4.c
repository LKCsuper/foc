/*
 * @Description: STM32F4 电机 HAL 移植层（委托现有 Bsp）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include "hal_motor.h"
#include "hal_sensor.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 启动三相 PWM 输出
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmStart(void)
{
    Bsp_Tim_StartPwm(MOTOR1);
}

/**
 * @description: 停止三相 PWM 输出
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmStop(void)
{
    Bsp_Tim_StopPwm(MOTOR1);
}

/**
 * @description: 设置三相 PWM 比较值
 * @param {uint32_t} duty_a
 * @param {uint32_t} duty_b
 * @param {uint32_t} duty_c
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmSetDuty(uint32_t duty_a, uint32_t duty_b, uint32_t duty_c)
{
    TIMER_UPDATE_DUTY_M1(duty_a, duty_b, duty_c);
}

/**
 * @description: 提交 PWM 更新（TIM1 COM 事件）
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmCommit(void)
{
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}

/**
 * @description: 输出安全中点占空比并提交
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmSetSafe(void)
{
    uint32_t mid = Hal_Motor_PwmPeriodCounts() / 2U;

    Hal_Motor_PwmSetDuty(mid, mid, mid);
    Hal_Motor_PwmCommit();
}

/**
 * @description: 仅使能指定相的互补 PWM
 * @param {uint8_t} phase  1=A，2=B，3=C
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_EnablePhase(uint8_t phase)
{
    uint16_t channel = TIM_Channel_1;

    if (phase == 2U) {
        channel = TIM_Channel_2;
    } else if (phase == 3U) {
        channel = TIM_Channel_3;
    } else if (phase != 1U) {
        return;
    }

    Bsp_Tim_StopPwm(MOTOR1);
    TIM_SelectOCxM(TIM1, channel, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIM1, channel, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, channel, TIM_CCxN_Enable);
    Hal_Motor_PwmCommit();
}

/**
 * @description: 读取 PWM 周期计数
 * @return {uint32_t} TIM1 ARR
 * @author: lkc
 */
uint32_t Hal_Motor_PwmPeriodCounts(void)
{
    return (uint32_t)TIM1->ARR;
}

/**
 * @description: 估算 ADC 采样触发频率
 * @return {float} 采样频率 (Hz)
 * @author: lkc
 */
float Hal_Motor_AdcSampleRateHz(void)
{
    uint32_t arr = (uint32_t)TIM1->ARR;
    uint32_t psc = (uint32_t)TIM1->PSC + 1U;

    if (arr == 0U || psc == 0U) {
        return 0.0f;
    }

    /* 中心对齐：计数顶部与底部各产生一次 ADC 触发 */
    return (float)SYSTEM_CORE_CLOCK / ((float)arr * (float)psc);
}

/**
 * @description: 读取微秒时基
 * @return {uint32_t} us
 * @author: lkc
 */
uint32_t Hal_Motor_TimeUs(void)
{
    return (uint32_t)Bsp_Tim_DtNow();
}

/**
 * @description: 初始化 AB 正交编码器
 * @return {*}
 * @author: lkc
 */
void Hal_Encoder_Init(void)
{
    Bsp_Tim4_EncInit();
}

/**
 * @description: 读取编码器计数值
 * @return {uint16_t} 当前计数
 * @author: lkc
 */
uint16_t Hal_Encoder_GetCount(void)
{
    return Bsp_Tim4_EncGetCount();
}

/**
 * @description: 设置编码器计数值
 * @param {uint16_t} count
 * @return {*}
 * @author: lkc
 */
void Hal_Encoder_SetCount(uint16_t count)
{
    Bsp_Tim4_EncSetCount(count);
}

/**
 * @description: 每机械圈计数值
 * @return {uint32_t} count/rev
 * @author: lkc
 */
uint32_t Hal_Encoder_CountsPerRev(void)
{
    return (uint32_t)BSP_ENCODER_COUNTS;
}

/**
 * @description: 读取三相霍尔原始状态
 * @param {bool} is_second_motor
 * @param {int} samples
 * @return {int} 0~7
 * @author: lkc
 */
int Hal_Hall_ReadRaw(bool is_second_motor, int samples)
{
    int h1 = 0;
    int h2 = 0;
    int h3 = 0;
    int n;
    int tres;

    n = 1 + 2 * samples;
    tres = n / 2;

    if (is_second_motor) {
        while (n--) {
            h1 += READ_HALL1_2();
            h2 += READ_HALL2_2();
            h3 += READ_HALL3_2();
        }
    } else {
        while (n--) {
            h1 += READ_HALL1();
            h2 += READ_HALL2();
            h3 += READ_HALL3();
        }
    }

    return (h1 > tres) | ((h2 > tres) << 1) | ((h3 > tres) << 2);
}

#ifdef __cplusplus
}
#endif
