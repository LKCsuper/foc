/*
 * @Description: STM32F4 电机 HAL 移植层（委托现有 Bsp）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#ifndef HAL_INTERNAL
#define HAL_INTERNAL
#endif
#include "includes.h"
#include "common_assert.h"
#include "hal_motor.h"
#include "hal_sensor.h"
#if defined(__CC_ARM)
#pragma O2
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint16_t s_encoder_count;
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

static bool s_irq_test_active;
static uint32_t s_irq_test_idle;

/** 返回三相主输出已强制无效且互补输出关闭的停机模式。 */
static bool Hal_Motor_StoppedIrqTestPwmOff(void)
{
    return (TIM1->CCMR1 & 0x7070U) == 0x4040U && (TIM1->CCMR2 & 0x70U) == 0x40U &&
           (TIM1->CCER & 0x444U) == 0U && (GPIOA->MODER & 0x3f0000U) == 0x2a0000U &&
           (GPIOB->MODER & 0xfc000000U) == 0xa8000000U &&
           (GPIOA->AFR[1] & 0xfffU) == 0x111U && (GPIOB->AFR[1] & 0xfff00000U) == 0x11100000U;
}

/**
 * @description: 读回TIM1输出关闭、六路输入全低和ADC-DMA中断屏蔽状态。
 * @param {bool} masked 预期屏蔽状态
 * @return {bool} true=窗口有效且寄存器和引脚读回正确
 */
bool Hal_Motor_StoppedIrqTestCheck(bool masked)
{
    bool enabled = (NVIC->ISER[(unsigned)DMA2_Stream4_IRQn >> 5U] &
                    (1UL << ((unsigned)DMA2_Stream4_IRQn & 31U))) != 0U;
    return s_irq_test_active && Hal_Motor_StoppedIrqTestPwmOff() &&
           !(TIM1->BDTR & TIM_BDTR_MOE) && (TIM1->BDTR & TIM_BDTR_OSSI) &&
           !(TIM1->CR2 & 0x3f00U) && !(GPIOA->IDR & 0x700U) && !(GPIOB->IDR & 0xe000U) && enabled != masked;
}

/**
 * @description: 临时清六路空闲电平和MOE，确认六路输入全低后可选屏蔽DMA IRQ。
 * @param {bool} masked true=屏蔽ADC-DMA IRQ
 * @return {bool} true=成功，失败时不主动开启驱动
 */
bool Hal_Motor_StoppedIrqTestBegin(bool masked)
{
    uint32_t key = __get_PRIMASK();
    if (key || s_irq_test_active || !Hal_Motor_StoppedIrqTestPwmOff() ||
        !(TIM1->BDTR & TIM_BDTR_MOE) || !(TIM1->BDTR & TIM_BDTR_OSSI) || (TIM1->BDTR & TIM_BDTR_AOE) ||
        !(NVIC->ISER[(unsigned)DMA2_Stream4_IRQn >> 5U] & (1UL << ((unsigned)DMA2_Stream4_IRQn & 31U))))
        return false;
    __disable_irq();
    s_irq_test_idle = TIM1->CR2 & 0x3f00U;
    TIM1->CR2 &= ~0x3f00U;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    __DSB();
    __ISB();
    if ((GPIOA->IDR & 0x700U) || (GPIOB->IDR & 0xe000U)) {
        __enable_irq();
        return false;
    }
    s_irq_test_active = true;
    if (masked) NVIC_DisableIRQ(DMA2_Stream4_IRQn);
    __DSB();
    __ISB();
    __enable_irq();
    return true;
}

/**
 * @description: 先恢复ADC-DMA IRQ，再按停机条件恢复MOE和六路空闲电平。
 * @param {bool} restore_gate true=调用者确认允许恢复原EN
 * @return {bool} true=原使能状态恢复并读回一致
 */
bool Hal_Motor_StoppedIrqTestEnd(bool restore_gate)
{
    bool valid;
    if (!s_irq_test_active) return false;
    __disable_irq();
    valid = !(TIM1->BDTR & TIM_BDTR_MOE) && !(TIM1->CR2 & 0x3f00U) &&
            !(GPIOA->IDR & 0x700U) && !(GPIOB->IDR & 0xe000U) && Hal_Motor_StoppedIrqTestPwmOff();
    NVIC_EnableIRQ(DMA2_Stream4_IRQn);
    __DSB();
    __ISB();
    valid = valid && restore_gate &&
            (NVIC->ISER[(unsigned)DMA2_Stream4_IRQn >> 5U] & (1UL << ((unsigned)DMA2_Stream4_IRQn & 31U)));
    if (valid) {
        TIM1->BDTR |= TIM_BDTR_MOE;
        TIM1->CR2 = (TIM1->CR2 & ~0x3f00U) | s_irq_test_idle;
    }
    __DSB();
    valid = valid && (TIM1->BDTR & TIM_BDTR_MOE) && (TIM1->CR2 & 0x3f00U) == s_irq_test_idle;
    if (!valid) {
        TIM1->CR2 &= ~0x3f00U;
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    }
    s_irq_test_active = false;
    __enable_irq();
    return valid;
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
static volatile bool s_pwm_diagnostic_enabled;
static volatile uint32_t s_pwm_diagnostic_sequence;
void Hal_Motor_PwmDiagnosticEnable(bool enable)
{
    s_pwm_diagnostic_enabled = false;
    if (enable) s_pwm_diagnostic_sequence = 0U;
    s_pwm_diagnostic_enabled = enable;
}
uint32_t Hal_Motor_PwmDiagnosticSequence(void) { return s_pwm_diagnostic_sequence; }
void Hal_Motor_PwmCommit(void)
{
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    if (s_pwm_diagnostic_enabled) s_pwm_diagnostic_sequence++;
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

    if (arr == 0U || psc == 0U) return 0.0f;

    /* 中心对齐：计数顶部与底部各产生一次 ADC 触发 */
    return (float)SYSTEM_CORE_CLOCK / ((float)arr * (float)psc);
}

/**
 * @description: 读取电机通用微秒时基
 * @return {uint32_t} 当前时间 (us)
 * @author: lkc
 */
uint32_t Hal_Motor_TimeUs(void)
{
    return (uint32_t)Bsp_Tim_DtNow();
}

/**
 * @description: 计算自 start_us 以来流逝的秒数
 * @param {uint32_t} start_us  Hal_Motor_TimeUs() 采样值
 * @return {float} 流逝时间 (s)
 * @author: lkc
 */
float Hal_Motor_TimeElapsedSec(uint32_t start_us)
{
    return (float)(uint32_t)(Hal_Motor_TimeUs() - start_us) * (1.0f / BSP_TIM_DT_HZ);
}

/**
 * @description: 捕获本周期一致的采样帧
 * @param {hal_motor_sample_t *} out
 * @return {bool} true=成功
 * @author: lkc
 */
bool Hal_Motor_SampleCapture(hal_motor_sample_t *out)
{
    if (out == NULL) return false;

    out->current_adc[0] = Hal_Motor_CurrentAdc(0U);
    out->current_adc[1] = Hal_Motor_CurrentAdc(1U);
    out->current_adc[2] = Hal_Motor_CurrentAdc(2U);
    out->sense_volts[0] = Hal_Motor_SenseVolts(0U);
    out->sense_volts[1] = Hal_Motor_SenseVolts(1U);
    out->sense_volts[2] = Hal_Motor_SenseVolts(2U);
    out->v_bus = Hal_Motor_BusVoltage();
    out->time_us = Hal_Motor_TimeUs();
    out->three_shunts = Hal_Motor_HasThreeShunts();
    out->current_mask = out->three_shunts ? 0x07U : 0x03U;
    return true;
}

/**
 * @description: 读取单个 ADC 通道原始值
 * @param {uint8_t} channel
 * @return {uint16_t} 原始计数
 * @author: lkc
 */
uint16_t Hal_Motor_AdcRaw(uint8_t channel)
{
    if (channel >= (uint8_t)(HW_ADC_CHANNELS + HW_ADC_CHANNELS_EXTRA)) return 0U;
    return ADC_Value[channel];
}

/**
 * @description: 拷贝 ADC DMA 缓冲
 * @param {uint16_t} *dst
 * @param {uint32_t} len
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_AdcGetRaw(uint16_t *dst, uint32_t len)
{
    uint32_t max_ch = (uint32_t)(HW_ADC_CHANNELS + HW_ADC_CHANNELS_EXTRA);
    uint32_t i;
    uint32_t n;

    COMMON_ASSERT(dst != NULL);
    n = (len < max_ch) ? len : max_ch;
    for (i = 0U; i < n; i++) {
        dst[i] = ADC_Value[i];
    }
}

/**
 * @description: 读取相电流 ADC 原始值
 * @param {uint8_t} phase
 * @return {uint16_t} ADC 计数
 * @author: lkc
 */
uint16_t Hal_Motor_CurrentAdc(uint8_t phase)
{
    switch (phase) {
    case 0U:
        return (uint16_t)GET_CURRENT1();
    case 1U:
        return (uint16_t)GET_CURRENT2();
    case 2U:
        return (uint16_t)GET_CURRENT3();
    default:
        return 0U;
    }
}

/**
 * @description: 读取相电压采样点电压
 * @param {uint8_t} phase
 * @return {float} V
 * @author: lkc
 */
float Hal_Motor_SenseVolts(uint8_t phase)
{
    switch (phase) {
    case 0U:
        return ADC_VOLTS(ADC_IND_SENS1);
    case 1U:
        return ADC_VOLTS(ADC_IND_SENS2);
    case 2U:
        return ADC_VOLTS(ADC_IND_SENS3);
    default:
        return 0.0f;
    }
}

/**
 * @description: 读取直流母线电压
 * @return {float} V
 * @author: lkc
 */
float Hal_Motor_BusVoltage(void)
{
    /* 保留GET_INPUT_VOLTAGE的4095分母和板级分压参数。
     * 固定系数在编译期换算为float，避免ADC拍内的软件双精度乘法。 */
    const float volts_per_count = (float)((V_REG / 4095.0) * ((VIN_R1 + VIN_R2) / VIN_R2));
    return (float)ADC_Value[ADC_IND_VIN_SENS] * volts_per_count;
}

/**
 * @description: 电流 ADC 计数到安培的换算系数
 * @return {float} A/count
 * @author: lkc
 */
float Hal_Motor_CurrentScale(void)
{
    return (float)FAC_CURRENT;
}

/**
 * @description: 相电压采样点到相电压的增益
 * @return {float} 增益
 * @author: lkc
 */
float Hal_Motor_PhaseVoltageGain(void)
{
    return (float)PHASE_VOLTAGE_DIVIDER_GAIN * (float)ADC_VOLTS_PH_FACTOR;
}

/**
 * @description: 是否为三电阻采样硬件
 * @return {bool}
 * @author: lkc
 */
bool Hal_Motor_HasThreeShunts(void)
{
#ifdef HW_HAS_3_SHUNTS
    return true;
#else
    return false;
#endif
}

/**
 * @description: 初始化 MT6701 使用的 I2C2 外设
 * @return {*}
 * @author: lkc
 */
void Hal_Encoder_Init(void)
{
    Bsp_I2c2_Init();
}

/**
 * @description: 读取后台任务发布的 MT6701 14 位角度快照
 * @return {uint16_t} 当前绝对角度计数
 * @author: lkc
 */
uint16_t Hal_Encoder_GetCount(void)
{
    return s_encoder_count;
}

/**
 * @description: 发布 MT6701 14 位角度快照
 * @param {uint16_t} count 角度计数
 * @return {*}
 * @author: lkc
 */
void Hal_Encoder_SetCount(uint16_t count)
{
    s_encoder_count = count;
}

/**
 * @description: MT6701 14 位绝对角度每机械圈计数值
 * @return {uint32_t} count/rev
 * @author: lkc
 */
uint32_t Hal_Encoder_CountsPerRev(void)
{
    return 16384U;
}

bool Hal_Encoder_I2cReadRegister(uint8_t address_7bit, uint8_t reg, uint8_t *value)
{
    return Bsp_I2c2_ReadRegister(address_7bit, reg, value);
}

void Hal_Encoder_SsiInit(void)
{
    Bsp_MT6701_SsiInit();
}

bool Hal_Encoder_SsiReadFrame(uint32_t *frame)
{
    return Bsp_MT6701_SsiReadFrame(frame);
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
