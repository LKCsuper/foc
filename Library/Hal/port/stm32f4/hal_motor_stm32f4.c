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

/**
 * @description: 当前 STM32F4 基线：TIM1 = 168 MHz、PSC = 0。
 * 与 ADC到PWM生效时序核对_2026-10-21 保持一致；更换时钟树时须同步移植契约。
 * 此处只描述既有配置，不修改定时器或采样触发。
 * @param {hal_motor_pwm_phase_t *} phase 输出，不可为空
 * @return {void} 无
 */
void Hal_Motor_PwmPhaseCapture(hal_motor_pwm_phase_t *phase)
{
    uint32_t cr1;
    COMMON_ASSERT(phase != NULL);
    cr1 = TIM1->CR1;
    phase->count = TIM1->CNT;
    phase->period = TIM1->ARR;
    phase->counts_per_us = 168U;
    phase->counting_down = (cr1 & TIM_CR1_DIR) != 0U;
}

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 启动三相 PWM 输出
 * @return {void} 无
 * @author: lkc
 */
void Hal_Motor_PwmStart(void)
{
    Bsp_Tim_StartPwm(MOTOR1);
}

/**
 * @description: 停止三相 PWM 输出
 * @return {void} 无
 * @author: lkc
 */
void Hal_Motor_PwmStop(void)
{
    Bsp_Tim_StopPwm(MOTOR1);
}

static bool s_irq_test_active;
static uint32_t s_irq_test_idle;

/**
 * @description: 返回三相主输出已强制无效且互补输出关闭的停机模式。
 * @return {bool} true=寄存器读回符合强制无效、互补通道关闭及引脚复用条件；false=任一条件不满足
 */
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

/* PWM 写入窗口及预约状态；只记录软件写入时刻，不代表硬件 CCR 生效。 */
hal_motor_pwm_window_t g_hal_motor_pwm_window;
static bool s_pwm_window_running;
static bool s_pwm_window_waiting;
static uint32_t s_pwm_window_ready_us;
static uint32_t s_pwm_window_stride = 1U;
static uint32_t s_pwm_window_skip;

/**
 * @description:  停机时启停有限 PWM 写入窗口采集；enable 清零并采集接下来 32 次写入。
 * @param {bool} enable 是否启用
 * @return {void} 无
 */
void Hal_Motor_PwmWindowEnable(bool enable)
{
    g_hal_motor_pwm_window.enabled = 0U;
    s_pwm_window_running = false;
    s_pwm_window_waiting = false;
    if (enable) {
        s_pwm_window_stride = 1U;
        s_pwm_window_skip = 0U;
        g_hal_motor_pwm_window.count = 0U;
        __DMB();
        g_hal_motor_pwm_window.enabled = 1U;
    }
}

/**
 * @description: 按指定步长预约 PWM 写入窗口采集。
 * @param {uint32_t} stride 采集步长（写入次数）
 * @return {void} 无
 */
static void Hal_Motor_PwmWindowReserveWithStride(uint32_t stride)
{
    Hal_Motor_PwmWindowEnable(false);
    g_hal_motor_pwm_window.count = 0U;
    s_pwm_window_stride = stride;
    s_pwm_window_skip = 0U;
    __DMB();
    g_hal_motor_pwm_window.enabled = 2U;
}

/**
 * @description:  停机预约：enabled=2 等待，1 采集，0 停止；调用前须确认无并发运行控制。
 * @return {void} 无
 */
void Hal_Motor_PwmWindowReserve(void) { Hal_Motor_PwmWindowReserveWithStride(1U); }
/**
 * @description:  停机预约长窗口：每 997 次写入记录一次，32 帧在 15kHz 下约跨 2.06s。
 * @return {void} 无
 */
void Hal_Motor_PwmWindowReserveLong(void) { Hal_Motor_PwmWindowReserveWithStride(997U); }
/**
 * @description:  返回当前或已完成窗口的写入步长；停止不清除该元数据。
 * @return {uint32_t} 当前值 s_pwm_window_stride
 */
uint32_t Hal_Motor_PwmWindowStride(void) { return s_pwm_window_stride; }

/**
 * @description:  控制 ISR 推进预约；ready 连续 1s 才开始，运行开始后再次停机取消剩余采集。
 * @param {bool} running 当前是否运行
 * @param {bool} ready 采集就绪条件
 * @return {void} 无
 */
void Hal_Motor_PwmWindowPoll(bool running, bool ready)
{
    uint32_t now;
    if (g_hal_motor_pwm_window.enabled == 0U) return;
    if (!running) {
        if (s_pwm_window_running) g_hal_motor_pwm_window.enabled = 0U;
        s_pwm_window_waiting = false;
        return;
    }
    s_pwm_window_running = true;
    if (g_hal_motor_pwm_window.enabled != 2U) return;
    if (!ready) {
        s_pwm_window_waiting = false;
        return;
    }
    now = Hal_Motor_TimeUs();
    if (!s_pwm_window_waiting) {
        s_pwm_window_ready_us = now;
        s_pwm_window_waiting = true;
    } else if ((uint32_t)(now - s_pwm_window_ready_us) >= 1000000U) {
        __DMB();
        g_hal_motor_pwm_window.enabled = 1U;
    }
}

/**
 * @description: 设置三相 PWM 预装载比较值。
 * @param {uint32_t} duty_a  A 相比较值，范围 [0, period]
 * @param {uint32_t} duty_b  B 相比较值，范围 [0, period]
 * @param {uint32_t} duty_c  C 相比较值，范围 [0, period]
 * @return {void} 无
 */
void Hal_Motor_PwmSetDuty(uint32_t duty_a, uint32_t duty_b, uint32_t duty_c)
{
    hal_motor_pwm_window_frame_t frame;
    uint32_t index = g_hal_motor_pwm_window.count;
    bool capture = g_hal_motor_pwm_window.enabled == 1U && index < HAL_MOTOR_PWM_WINDOW_CAPACITY;
    if (capture) {
        if (s_pwm_window_skip != 0U) {
            s_pwm_window_skip--;
            capture = false;
        } else {
            s_pwm_window_skip = s_pwm_window_stride - 1U;
        }
    }
    if (capture) {
        frame.before_us = Hal_Motor_TimeUs();
        frame.before_cr1 = TIM1->CR1;
        frame.before_count = TIM1->CNT;
    }
    TIMER_UPDATE_DUTY_M1(duty_a, duty_b, duty_c);
    if (capture) {
        frame.after_count = TIM1->CNT;
        frame.after_cr1 = TIM1->CR1;
        frame.after_us = Hal_Motor_TimeUs();
        frame.period = TIM1->ARR;
        g_hal_motor_pwm_window.frames[index] = frame;
        __DMB();
        g_hal_motor_pwm_window.count = index + 1U;
        if (index + 1U == HAL_MOTOR_PWM_WINDOW_CAPACITY)
            g_hal_motor_pwm_window.enabled = 0U;
    }
}

/**
 * @description: 提交 PWM 更新（TIM1 COM 事件）
 * @return {*}
 * @author: lkc
 */
static volatile bool s_pwm_diagnostic_enabled;
static volatile uint32_t s_pwm_diagnostic_sequence;
/**
 * @description:  停机设置软件提交计数，enable时清零；不改变定时器。计数不代表硬件装载事件。
 * @param {bool} enable 是否启用
 * @return {void} 无
 */
void Hal_Motor_PwmDiagnosticEnable(bool enable)
{
    s_pwm_diagnostic_enabled = false;
    if (enable) s_pwm_diagnostic_sequence = 0U;
    s_pwm_diagnostic_enabled = enable;
}
/**
 * @description:  读取停机诊断软件提交计数（enable 时清零，仅停机诊断使用）。
 * @return {uint32_t} 当前值 s_pwm_diagnostic_sequence
 */
uint32_t Hal_Motor_PwmDiagnosticSequence(void) { return s_pwm_diagnostic_sequence; }
/**
 * @description: 提交 PWM 通道控制更新。
 * @return {void} 无
 */
void Hal_Motor_PwmCommit(void)
{
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    if (s_pwm_diagnostic_enabled) s_pwm_diagnostic_sequence++;
}

/**
 * @description: 输出安全中点占空比并提交
 * @return {void} 无
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
 * @return {void} 无
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
 * @description: 按固定板级 ADC 下标换算采样点电压，供单相与整帧入口共用。
 * @detail: 保留 ADC_VOLTS 的运算顺序；内联后整帧无需逐相调用 switch。
 * @param {uint8_t} channel 板级相电压 ADC 下标
 * @return {float} 采样点电压 (V)
 */
static inline float Hal_Motor_SenseChannelVolts(uint8_t channel)
{
    return ADC_VOLTS(channel);
}

/**
 * @description: 捕获本周期一致的采样帧
 * @param {hal_motor_sample_t *} out 输出采样帧，不可为空
 * @return {bool} true=成功
 * @author: lkc
 */
bool Hal_Motor_SampleCapture(hal_motor_sample_t *out)
{
    if (out == NULL) return false;

    out->current_adc[0] = Hal_Motor_CurrentAdc(0U);
    out->current_adc[1] = Hal_Motor_CurrentAdc(1U);
    out->current_adc[2] = Hal_Motor_CurrentAdc(2U);
    out->sense_volts[0] = Hal_Motor_SenseChannelVolts(ADC_IND_SENS1);
    out->sense_volts[1] = Hal_Motor_SenseChannelVolts(ADC_IND_SENS2);
    out->sense_volts[2] = Hal_Motor_SenseChannelVolts(ADC_IND_SENS3);
    out->v_bus = Hal_Motor_BusVoltage();
    out->time_us = Hal_Motor_TimeUs();
    out->three_shunts = Hal_Motor_HasThreeShunts();
    out->current_mask = out->three_shunts ? 0x07U : 0x03U;
    return true;
}

/**
 * @description: 读取单个 ADC 通道原始值
 * @param {uint8_t} channel ADC DMA 缓冲区通道下标
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
 * @param {uint16_t} *dst 输出 ADC 原始值缓冲
 * @param {uint32_t} len 最多拷贝的 ADC 通道数量
 * @return {void} 无
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
 * @param {uint8_t} phase 相索引；范围及相序以本接口实现为准
 * @return {uint16_t} ADC 计数
 * @author: lkc
 */
#if defined(__CC_ARM)
__forceinline
#endif
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
 * @param {uint8_t} phase 相索引；范围及相序以本接口实现为准
 * @return {float} V
 * @author: lkc
 */
float Hal_Motor_SenseVolts(uint8_t phase)
{
    switch (phase) {
    case 0U:
        return Hal_Motor_SenseChannelVolts(ADC_IND_SENS1);
    case 1U:
        return Hal_Motor_SenseChannelVolts(ADC_IND_SENS2);
    case 2U:
        return Hal_Motor_SenseChannelVolts(ADC_IND_SENS3);
    default:
        return 0.0f;
    }
}

/**
 * @description: 读取直流母线电压
 * @return {float} V
 * @author: lkc
 */
#if defined(__CC_ARM)
__forceinline
#endif
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
#if defined(__CC_ARM)
__forceinline
#endif
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
 * @return {void} 无
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
 * @return {void} 无
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

/**
 * @description: 读取位置传感器 I2C 寄存器。
 * @param {uint8_t} address_7bit 7 位设备地址
 * @param {uint8_t} reg          寄存器地址
 * @param {uint8_t *} value      输出数据，不可为空
 * @return {bool} true=读取成功
 */
bool Hal_Encoder_I2cReadRegister(uint8_t address_7bit, uint8_t reg, uint8_t *value)
{
    return Bsp_I2c2_ReadRegister(address_7bit, reg, value);
}

/**
 * @description: 初始化位置传感器 SSI 读取接口（MT6701）
 * @return {void} 无
 */
void Hal_Encoder_SsiInit(void)
{
    Bsp_MT6701_SsiInit();
}

/**
 * @description: 读取一帧 24 位 SSI 角度数据
 * @param {uint32_t *} frame 输出原始帧，不可为空
 * @return {bool} true=读取成功
 */
bool Hal_Encoder_SsiReadFrame(uint32_t *frame)
{
    return Bsp_MT6701_SsiReadFrame(frame);
}

/**
 * @description: 读取三相霍尔原始状态
 * @param {bool} is_second_motor 是否选择第二电机霍尔接口；当前支持能力由 HAL 实现决定
 * @param {int} samples 额外滤波采样次数
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
