/*
 * @Description: 
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-03-26 20:08:32
 * @FilePath: \foc\App\motor\inc\utils.h
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
void Motor_DmaInt(void *p, uint32_t flags);

/* 供 Shell 状态显示预对齐是否仍在进行，以及已经保持了多长时间。 */
bool Motor_OpenLoop_IsAligning(void);
float Motor_OpenLoop_GetAlignElapsed(void);

/* FOC 中断耗时统计，供 olstatus 判断 CPU 是否被实时控制占满。 */
float Motor_DmaInt_GetAverageRuntimeUs(void);
float Motor_DmaInt_GetMaximumRuntimeUs(void);
float Motor_DmaInt_GetAdcFrequencyHz(void);
float Motor_DmaInt_GetControlFrequencyHz(void);
#ifdef __cplusplus
}
#endif

#endif
