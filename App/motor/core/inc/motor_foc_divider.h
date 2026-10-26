/*
 * @Description: FOC 控制环分频计数（ADC 回调 → 是否跑完整 FOC）
 * @Author: lkc
 *
 * 学习要点：分频只决定是否跑完整 FOC；采样和保护在完整拍，dt 须×divider。
 */
#ifndef MOTOR_FOC_DIVIDER_H
#define MOTOR_FOC_DIVIDER_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: FOC 控制环分频状态（不持久化）
 * @detail: skip 为累计拍数；满 divider 清零并返回“本拍跑 FOC”
 */
typedef struct {
    int skip;    /* 距下次 FOC 的累计拍数 */
    int divider; /* 分频数，<=0 时按 1 处理 */
} motor_foc_divider_t;

/**
 * @description: 按 PWM 半周期选拍的门控参数（计数单位均为定时器计数）。
 * @detail: 完整控制拍固定在向下计数半周期（与 Hall/编码器路径、VESC v7 判断同侧），
 *          并且只接受“顶部更新后 [min_counts, max_counts] 内”进入的回调：
 *          过早（上一中断拖过边界后补进来的旧回调，DMA 可能正在写新一组）或
 *          过晚（中断被长时间推迟）都放弃本拍，宁可少跑一拍，也不换半周期或用半新半旧的采样。
 */
typedef struct {
    uint32_t min_counts; /* 顶部更新后最早接受的计数 */
    uint32_t max_counts; /* 顶部更新后最晚接受的计数 */
    uint32_t repeat_us;  /* 两个完整拍之间的最小间隔 (us)，防止同一半周期执行两次 */
} motor_foc_phase_gate_config_t;

/**
 * @description: 半周期门控运行状态（不持久化）。
 */
typedef struct {
    uint32_t last_us; /* 上一次放行的时间戳 (us) */
    bool seeded;      /* last_us 是否有效 */
} motor_foc_phase_gate_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 初始化分频状态
 * @param {motor_foc_divider_t} *state  分频状态；NULL 直接返回
 * @param {int} divider                 分频数；<=0 时置为 1
 * @return {*}
 * @author: lkc
 */
void Motor_FocDivider_Init(motor_foc_divider_t *state, int divider);

/**
 * @description: 每个 ADC 回调推进一拍，判断本拍是否应跑完整 FOC
 * @detail: 返回 true 时调用方才采样、保护和跑电流环；返回 false 的拍不再补做这些
 * @param {motor_foc_divider_t} *state  分频状态；NULL 时恒返回 true
 * @return {bool} true=本拍执行采样、保护、电流环和 SVPWM
 * @author: lkc
 */
bool Motor_FocDivider_ShouldRun(motor_foc_divider_t *state);

/**
 * @description: 按计数方向与半周期内位置判断本回调是否跑完整 FOC。
 * @detail: 纯计算，不读寄存器；调用方先读 CR1.DIR 再读 CNT 传入。
 *          counting_down=false、count>period、位置在窗口外或距上次放行不足 repeat_us 时返回 false。
 * @param {motor_foc_phase_gate_t *} gate 门控状态，不可为空
 * @param {const motor_foc_phase_gate_config_t *} config 门控参数，不可为空
 * @param {bool} counting_down 定时器是否处于向下计数（TIM_CR1_DIR=1）
 * @param {uint32_t} count 定时器当前计数 CNT
 * @param {uint32_t} period 自动重装值 ARR
 * @param {uint32_t} now_us 当前微秒时间戳，允许回绕
 * @return {bool} true=本拍执行采样、保护、电流环和 SVPWM
 */
bool Motor_FocDivider_PhaseGate(motor_foc_phase_gate_t *gate, const motor_foc_phase_gate_config_t *config,
                                bool counting_down, uint32_t count, uint32_t period, uint32_t now_us);

bool Motor_FocDivider_ControlExpired(motor_foc_phase_gate_t *clock, bool running, bool execute,
                                     uint32_t now_us, uint32_t max_gap_us);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
