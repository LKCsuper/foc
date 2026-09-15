/*
 * @Description: BLDC 六步换相功率桥硬件接口
 * @Author: lkc
 */
#ifndef HAL_MOTOR_BLDC_H
#define HAL_MOTOR_BLDC_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef enum {
    HAL_MOTOR_BLDC_PHASE_FLOAT = 0,
    HAL_MOTOR_BLDC_PHASE_PWM,
    HAL_MOTOR_BLDC_PHASE_LOW,
} hal_motor_bldc_phase_mode_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 配置 BLDC 六步运行所需的板级模拟采样路径。
 * @detail: BM75/HD75 在 BLDC 模式下断开 PC13 控制的三相采样滤波电容。
 * @return {void} 无
 */
void Hal_Motor_BldcPrepare(void);

/**
 * @description: 同步应用三相六步桥臂命令。
 * @detail: HAL/板级层负责逻辑 A/B/C 到物理通道的相序映射，以及 PWM/LOW/FLOAT 的电气实现。
 *          调用者仍负责门极 EN、定时器/MOE 初始化和硬件 Break。
 * @param {hal_motor_bldc_phase_mode_t} phase_a 逻辑 A 相状态
 * @param {hal_motor_bldc_phase_mode_t} phase_b 逻辑 B 相状态
 * @param {hal_motor_bldc_phase_mode_t} phase_c 逻辑 C 相状态
 * @param {float} duty PWM 相占空比 [0,1]；非法值按 0 处理
 * @return {void} 无
 */
void Hal_Motor_BldcApply(hal_motor_bldc_phase_mode_t phase_a,
                         hal_motor_bldc_phase_mode_t phase_b,
                         hal_motor_bldc_phase_mode_t phase_c, float duty);

/**
 * @description: 判断当前 ADC 帧是否位于 BLDC 有效的 PWM 导通采样窗口。
 * @detail: 中心对齐 PWM 顶部/底部各触发一次 ADC；仅向上计数侧的导通窗口样本用于 BEMF。
 * @return {bool} 当前帧可用于 BLDC BEMF 与换相状态机返回 true
 */
bool Hal_Motor_BldcBemfSampleReady(bool upward_counting);

/**
 * @description: 读取 BLDC 相电压 ADC 的 TIM2 触发计数。
 * @return {uint32_t} TIM2 CCR2 计数值
 */
uint32_t Hal_Motor_BldcGetSampleCount(void);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif /* HAL_MOTOR_BLDC_H */
