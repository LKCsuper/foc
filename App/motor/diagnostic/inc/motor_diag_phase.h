/* PWM 计数方向切换沿与 ADC-DMA 中断现场的相位诊断环形记录 */
#ifndef MOTOR_DIAG_PHASE_H
#define MOTOR_DIAG_PHASE_H
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Private define ------------------------------------------------------------*/
#define MOTOR_PHASE_CAPACITY 64U
/* Private typedef -----------------------------------------------------------*/
/* 一条相位沿事件：TIM1 计数方向切换附近的时间戳、CR1/分频/标志与 IRQ 现场。 */
typedef struct {
    uint32_t sequence;
    uint32_t time_us;
    uint32_t count;
    uint32_t cr1;
    uint32_t divider;
    uint32_t flags;
    uint32_t irq_entry_us;
    uint32_t irq_exit_us;
    uint32_t dma_status;
    uint32_t adc_status;
    uint32_t dma_remaining;
    uint32_t irq_pending;
} motor_phase_event_t;

/* 相位沿事件环形缓冲，供停机后按触发前后顺序读取。 */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t head;
    uint32_t count;
    uint32_t seen;
    uint32_t trigger_sequence;
    uint32_t reason;
    motor_phase_event_t events[MOTOR_PHASE_CAPACITY];
} motor_phase_debug_t;

/* Private variables ---------------------------------------------------------*/
extern motor_phase_debug_t g_motor_phase_debug;
extern volatile uint32_t g_motor_phase_active;
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 武装相位沿诊断，清空缓冲并进入采集态（active=1）。
 * @detail: 在临界区内清零结构体，写入魔数与版本，并复位方向种子与 IRQ 关联状态。
 * @return {void} 无
 */
void Motor_Timing_PhaseEdgeArm(void);

/**
 * @description: 武装并进入预留态（active=3），等待平台稳定就绪后再转为采集。
 * @detail: 先执行 Arm 再把 active 置 3；PhaseEdgePoll 在 running 且 ready 稳定约 1 s 后转回 active=1。
 * @return {void} 无
 */
void Motor_Timing_PhaseEdgeReserve3000(void);

/**
 * @description: 在预留态轮询平台运行与就绪状态，决定是否开始或停止采集。
 * @detail: 仅 active 为 3 时生效。running 变假时结束并记 reason=2；ready 持续约 1 s 后把 active 转回 1。
 * @param {bool} running    平台是否仍在运行
 * @param {bool} ready      平台是否已就绪
 * @param {uint32_t} time_us 轮询时间戳 (us)
 * @return {void} 无
 */
void Motor_Timing_PhaseEdgePoll(bool running, bool ready, uint32_t time_us);

/**
 * @description: 停止相位沿诊断。
 * @detail: 在临界区内把 active 置 0；若尚无原因则记为停机（reason=2）。
 * @return {void} 无
 */
void Motor_Timing_PhaseEdgeStop(void);

/**
 * @description: 记录一次 IRQ 入口时间并打开本拍 IRQ 现场关联窗口。
 * @param {uint32_t} time_us IRQ 入口时间戳 (us)
 * @return {void} 无
 */
void Motor_Timing_PhaseIrqBegin(uint32_t time_us);

/**
 * @description: 关闭 IRQ 窗口，把出口时间与 DMA/ADC 状态写入对应事件。
 * @detail: 仅当本拍已有关联事件且序号匹配时回填，并置事件 flags 的 bit1。
 * @param {uint32_t} time_us     IRQ 出口时间戳 (us)
 * @param {uint32_t} dma_status  DMA 状态寄存器
 * @param {uint32_t} adc_status  ADC 状态寄存器
 * @param {uint32_t} remaining   DMA 剩余计数
 * @param {uint32_t} pending     待处理挂起标志
 * @return {void} 无
 */
void Motor_Timing_PhaseIrqEnd(uint32_t time_us, uint32_t dma_status, uint32_t adc_status,
                              uint32_t remaining, uint32_t pending);

/**
 * @description: 给当前关联事件标记状态位。
 * @detail: IRQ 窗口打开且序号匹配时，置事件 flags 的 bit2。
 * @return {void} 无
 */
void Motor_Timing_PhaseMarkStatus(void);

/**
 * @description: 写入一条相位沿事件，并检测 PWM 计数方向切换以触发后采集。
 * @detail: active 为 0 或 3 时忽略；running 变假时结束采集（reason=2）。flags 的 bit0 为方向样本，
 *          检测到方向变化时记录触发序号并把 active 置 2，再采 16 条后停止；采集超过 10 s 未触发则记 reason=3 停止。
 * @param {uint32_t} time_us  事件时间戳 (us)
 * @param {uint32_t} count    当前计数
 * @param {uint32_t} cr1      TIM1 CR1 寄存器，bit4 为 DIR 计数方向
 * @param {uint32_t} divider  当前分频
 * @param {uint32_t} flags    事件标志，bit0 为方向样本
 * @param {bool} running      平台是否仍在运行
 * @return {void} 无
 */
void Motor_Timing_PhaseEdgeNote(uint32_t time_us, uint32_t count, uint32_t cr1,
                            uint32_t divider, uint32_t flags, bool running);
#endif
