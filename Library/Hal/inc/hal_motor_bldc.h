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
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif /* HAL_MOTOR_BLDC_H */
