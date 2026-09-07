/*
 * @Description: ADC-DMA 中断与 FOC 控制环入口
 * @Author: lkc
 */
#ifndef _MOTOR_IT_H
#define _MOTOR_IT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*
 * ADC 在中心对齐 PWM 的上、下半周期各完成一次采样，因此当前硬件每秒产生
 * 30000 次 ADC-DMA 回调，而功率管的完整 PWM 周期是 15000Hz。
 *
 * 电流环、无感磁链观测器、PLL、保护和 SVPWM 全部放在这个高优先级中断中。
 * 分频为 1 时每半个 PWM 周期运行一次，电机转动后无感运算会占用大量 CPU，
 * 普通任务（包括 Shell 浮点格式化）只能在很小的空隙中运行。
 *
 * 分频为 2 后，每个完整 PWM 周期运行一次电流环/无感/SVPWM，也就是 15000Hz。
 * 控制代码会把 FOC dt 同步乘以 2，因此转速斜坡、PI 和观测器仍按真实时间
 * 计算。采样换算和故障保护放在分频判断之前，继续保持 30000Hz 响应。
 */
#ifndef FOC_CONTROL_LOOP_FREQ_DIVIDER
#define FOC_CONTROL_LOOP_FREQ_DIVIDER	2
#endif

// Current ADC to amperes factor V_REG输入3.3v电  采样电阻CURRENT_SHUNT_RES
//#define FAC_CURRENT					((V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: ADC-DMA 传输完成中断回调，FOC 控制链驱动入口。
 * @param {void} *p       回调用户参数（未使用）
 * @param {uint32_t} flags  DMA 事件标志（未使用）
 * @return {*}
 * @author: lkc
 */
void Motor_DmaInt(void *p, uint32_t flags);

/**
 * @description: 查询当前是否正处于 I/F 启动前的转子预对齐阶段。
 * @return {bool} true=预对齐进行中
 * @author: lkc
 */
bool Motor_OpenLoop_IsAligning(void);

/**
 * @description: 返回本次转子预对齐已进行的时间。
 * @return {float} 已对齐时长 (s)
 * @author: lkc
 */
float Motor_OpenLoop_GetAlignElapsed(void);

/**
 * @description: 查询 HFI 凸极扫描启动流程是否正在运行。
 * @return {bool} true=HFI 扫描进行中
 * @author: lkc
 */
bool Motor_HfiStart_IsRunning(void);

/**
 * @description: 查询 HFI 凸极扫描是否已成功识别出转子初始电角度。
 * @return {bool} true=识别成功
 * @author: lkc
 */
bool Motor_HfiStart_Succeeded(void);

/**
 * @description: 返回 HFI 凸极扫描识别出的转子电角度。
 * @return {float} 转子电角度 (rad)
 * @author: lkc
 */
float Motor_HfiStart_GetAngle(void);

/**
 * @description: 返回 HFI 凸极扫描识别的可信度。
 * @return {float} 可信度 [0, 1]
 * @author: lkc
 */
float Motor_HfiStart_GetConfidence(void);

/**
 * @description: 返回 ADC-DMA 中断的平均执行时间。
 * @return {float} 平均耗时 (us)
 * @author: lkc
 */
float Motor_DmaInt_GetAverageRuntimeUs(void);

/**
 * @description: 返回 ADC-DMA 中断的历史最长执行时间。
 * @return {float} 最大耗时 (us)
 * @author: lkc
 */
float Motor_DmaInt_GetMaximumRuntimeUs(void);

/**
 * @description: 返回实际 ADC-DMA 触发频率。
 * @return {float} ADC 采样频率 (Hz)
 * @author: lkc
 */
float Motor_DmaInt_GetAdcFrequencyHz(void);

/**
 * @description: 返回 FOC 控制环实际运行频率。
 * @return {float} 控制环频率 (Hz)
 * @author: lkc
 */
float Motor_DmaInt_GetControlFrequencyHz(void);

/**
 * @description: 采样换算入口：捕获 ADC 帧并换算为电流/电压物理量。
 * @detail: 内部为 SampleCapture → ApplyFrame；可在每拍 ADC 回调中调用。
 * @return {*}
 * @author: lkc
 */
void Motor_It_ParamCal(void);

#ifdef __cplusplus
}
#endif

#endif
