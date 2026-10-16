/*
 * @Description: 电机功率级硬件抽象（PWM / 采样时基）
 * @Author: lkc
 */
#ifndef _HAL_MOTOR_H
#define _HAL_MOTOR_H
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
 * @description: 启动三相 PWM 输出（模式已配置，不改占空比）。
 * @detail: 仅使能已配置通道；不等于写入零矢量或故障关断。
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmStart(void);

/**
 * @description: 停止三相 PWM 输出（关主通道/互补通道，具体由板级 BSP）。
 * @detail: 这是“关闭驱动输出”路径，不等于 PwmSetSafe 的中点占空比；
 *          也不保证所有板的故障安全（需结合门极极性与硬件 Break）。
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmStop(void);

/**
 * @description: 设置三相 PWM 比较值（尚未生效，须再 Commit）。
 * @param {uint32_t} duty_a  A 相比较值，范围 [0, period]
 * @param {uint32_t} duty_b  B 相比较值，范围 [0, period]
 * @param {uint32_t} duty_c  C 相比较值，范围 [0, period]
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmSetDuty(uint32_t duty_a, uint32_t duty_b, uint32_t duty_c);

/**
 * @description: 提交 PWM 更新，使三相占空比在同一更新时刻生效。
 * @detail: STM32 上为 TIM COM 事件；无预装载需求的平台可为空操作。
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmCommit(void);
/** 停机设置软件提交计数，enable时清零；不改变定时器。计数不代表硬件装载事件。 */
void Hal_Motor_PwmDiagnosticEnable(bool enable);
uint32_t Hal_Motor_PwmDiagnosticSequence(void);

/**
 * @description: 输出三相中点占空比并提交（各 period/2）。
 * @detail: 对应控制层 SAFE/零电压矢量请求；不等于保证无电流、无转矩或已关桥。
 *          故障硬关断请用板级 Stop/Break，勿把本函数当作普遍“安全态”命名依据。
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_PwmSetSafe(void);

/**
 * @description: 仅使能指定相的互补 PWM（偏移校准时逐相采样用）
 * @param {uint8_t} phase  相序号：1=A，2=B，3=C
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_EnablePhase(uint8_t phase);

/**
 * @description: 读取 PWM 周期计数（通常等于 ARR）
 * @return {uint32_t} 周期计数值
 * @author: lkc
 */
uint32_t Hal_Motor_PwmPeriodCounts(void);

/**
 * @description: 估算 ADC/控制采样触发频率
 * @detail: 中心对齐 PWM 顶底各触发一次时约为 PWM 频率的 2 倍
 * @return {float} 采样频率 (Hz)；未初始化时返回 0
 * @author: lkc
 */
float Hal_Motor_AdcSampleRateHz(void);

/**
 * @description: 读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: ISR 耗时、采样时标、速度环 dt、命令时间戳共用。当前 1 tick = 1 us。
 * @return {uint32_t} 当前时间 (us)
 * @author: lkc
 */
uint32_t Hal_Motor_TimeUs(void);

/**
 * @description: 计算自 start_us 以来流逝的时间
 * @param {uint32_t} start_us  起始时刻 (us)，必须来自 Hal_Motor_TimeUs
 * @return {float} 流逝时间 (s)，支持 32 位回绕
 * @author: lkc
 */
float Hal_Motor_TimeElapsedSec(uint32_t start_us);

/**
 * @description: 同一周期采样帧（由 HAL 填充，算法只读本结构换算）。
 * @detail: current_adc 为相序 A/B/C；sense_volts 为板级 SENS1/2/3 采样点电压 (V)，
 *          相序映射（如本板 vb←SENS3）在 Motor_Sense 中按历史约定完成。
 *          12 位量程/增益不在此结构硬编码，电流/电压比例经 CurrentScale/PhaseVoltageGain 查询。
 */
typedef struct {
    uint16_t current_adc[3]; /* 相电流 ADC 计数：0=A，1=B，2=C */
    float sense_volts[3];    /* SENS1/2/3 采样点电压 (V) */
    float v_bus;             /* 直流母线电压 (V) */
    uint32_t time_us;        /* 采样时刻 (us) */
    uint8_t current_mask;    /* bit0/1/2=A/B/C 电流有效 */
    bool three_shunts;       /* true=三电阻完整电流采样 */
} hal_motor_sample_t;

/**
 * @description: 捕获本周期一致的采样帧（电流/相感电压/母线/时标）。
 * @param {hal_motor_sample_t *} out 输出帧，不可为空
 * @return {bool} true=捕获成功；false=out 为空
 */
bool Hal_Motor_SampleCapture(hal_motor_sample_t *out);

/**
 * @description: 读取单个 ADC 通道原始值
 * @param {uint8_t} channel  ADC 缓冲下标
 * @return {uint16_t} 原始计数；通道越界返回 0
 * @author: lkc
 */
uint16_t Hal_Motor_AdcRaw(uint8_t channel);

/**
 * @description: 拷贝 ADC DMA 缓冲到调用方数组
 * @param {uint16_t} *dst  目标缓冲
 * @param {uint32_t} len   最多拷贝通道数
 * @return {*}
 * @author: lkc
 */
void Hal_Motor_AdcGetRaw(uint16_t *dst, uint32_t len);

/**
 * @description: 读取相电流 ADC 原始值（含分流极性处理）
 * @param {uint8_t} phase  0=A，1=B，2=C
 * @return {uint16_t} ADC 计数
 * @author: lkc
 */
uint16_t Hal_Motor_CurrentAdc(uint8_t phase);

/**
 * @description: 读取相电压采样点电压
 * @param {uint8_t} phase  0=SENS1，1=SENS2，2=SENS3
 * @return {float} 采样点电压 (V)
 * @author: lkc
 */
float Hal_Motor_SenseVolts(uint8_t phase);

/**
 * @description: 读取直流母线电压
 * @return {float} 母线电压 (V)
 * @author: lkc
 */
float Hal_Motor_BusVoltage(void);

/**
 * @description: 电流 ADC 计数到安培的换算系数
 * @return {float} 比例 (A/count)
 * @author: lkc
 */
float Hal_Motor_CurrentScale(void);

/**
 * @description: 相电压采样点到相电压的增益
 * @detail: 含分压网络与 ADC_VOLTS_PH_FACTOR
 * @return {float} 无量纲增益
 * @author: lkc
 */
float Hal_Motor_PhaseVoltageGain(void);

/**
 * @description: 是否为三电阻采样硬件
 * @return {bool} true 表示三相分流齐全，可用完整 Clarke
 * @author: lkc
 */
bool Hal_Motor_HasThreeShunts(void);

#ifdef __cplusplus
}
#endif
#endif
