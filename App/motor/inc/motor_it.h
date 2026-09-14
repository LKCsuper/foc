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
#include <stdbool.h>
#include "motor_bldc_sensorless.h"
#include "motor_types.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct {
    bool selected;
    bool running;
    float duty;
    float target_duty;
    uint8_t step;
    motor_bldc_direction_t direction;
    motor_bldc_sensorless_mode_t mode;
    motor_bldc_sensorless_fault_t fault;
    mc_fault_code motor_fault;
    float bemf_v;
    float bemf_min_v;
    float bemf_max_v;
    float sector_elapsed_s;
    float commutation_period_s;
    float minimum_commutation_period_s;
    float maximum_commutation_period_s;
    float bemf_integral_vs;
    float bemf_reverse_integral_vs;
    float bemf_integral_threshold_vs;
    uint32_t bemf_sample_count;
    uint32_t integral_commutations;
    uint32_t forced_commutations;
    uint8_t consecutive_forced_commutations;
    uint32_t startup_valid_sectors;
    uint32_t startup_invalid_sectors;
    uint32_t startup_sync_resets;
    float startup_takeover_time_s;
    float startup_takeover_period_s;
    uint8_t startup_sync_sectors;
    uint16_t step_integral_commutations[6];
    uint16_t step_forced_commutations[6];
    float step_last_period_s[6];
    float step_last_integral_vs[6];
    float step_startup_integral_vs[6];
    float step_startup_reverse_integral_vs[6];
    float step_bemf_min_v[6];
    float step_bemf_max_v[6];
    float step_bemf_waveform_v[6][MOTOR_BLDC_BEMF_BIN_COUNT];
} motor_bldc_status_t;
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

/**
 * @description: 选择并启动默认无感 BLDC 六步控制实例。
 * @detail: 仅允许在 FOC 停机且保护未锁存时调用；成功后 ADC 中断不再执行 FOC 输出。
 * @param {motor_bldc_direction_t} direction 转向，只允许 CW 或 CCW
 * @param {float} duty PWM 占空比，当前实测安全范围 [0.08,0.10]
 * @return {bool} 成功进入 BLDC 定向状态返回 true
 * @author: lkc
 */
bool Motor_Bldc_Start(motor_bldc_direction_t direction, float duty);

/**
 * @description: 启动固定扇区周期的开环 BLDC 六步诊断。
 * @detail: 完成定向后不读取 BEMF、不接管闭环；用于区分功率换相与无感检测问题。
 * @param {motor_bldc_direction_t} direction 转向，只允许 CW 或 CCW
 * @param {float} duty PWM 占空比，范围 (0,0.25]
 * @param {float} sector_period_s 每个 60° 电气扇区周期 (s)，范围 [0.5ms,500ms]
 * @return {bool} 成功进入 BLDC 定向状态返回 true
 * @author: lkc
 */
bool Motor_Bldc_StartOpenLoop(motor_bldc_direction_t direction, float duty,
                              float sector_period_s);

/**
 * @description: 停机时设置默认 BLDC 积分换相调试参数。
 * @param {float} integral_threshold_vs BEMF 积分阈值 (V*s)
 * @param {float} minimum_period_s 闭环最短 60° 换相周期 (s)
 * @return {bool} 参数有效且停机时返回 true
 * @author: lkc
 */
bool Motor_Bldc_SetTuning(float integral_threshold_vs, float minimum_period_s);

/**
 * @description: 停止默认 BLDC 控制，但继续保留 PWM 出口选择权。
 * @detail: 请求三相悬空；须另调用 Motor_Bldc_Exit 才允许 FOC 再次启动。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Bldc_Stop(void);

/**
 * @description: 停止 BLDC 并释放其对 PWM 出口的选择权。
 * @detail: 退出后恢复原 FOC 调度，但不会自动提交 FOC 启动命令。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Bldc_Exit(void);

/**
 * @description: 查询 BLDC 是否占用主 PWM 控制出口。
 * @return {bool} BLDC 已选择返回 true
 * @author: lkc
 */
bool Motor_Bldc_IsSelected(void);

/**
 * @description: 获取默认 BLDC 控制实例的只读状态快照。
 * @param {motor_bldc_status_t *} status 输出状态，不可为空
 * @return {bool} 成功获取返回 true
 * @author: lkc
 */
bool Motor_Bldc_GetStatus(motor_bldc_status_t *status);

#ifdef __cplusplus
}
#endif

#endif
