/*
 * @Description: FOC 中断集成：采样、保护、分频、控制调用与 PWM/耗时
 * @Author: lkc
 *
 * 学习要点：
 *   1) 本文件是平台胶水，不承载对齐/斜坡/电流 PI 算法细节。
 *   2) 算法在 motor_control / motor_current / 启动与无感模块。
 *   3) 分频前：Hal 采样帧 → Sense 换算 + 软件保护；分频后：Motor_Control_Step → 应用 PWM。
 *   4) 速度 PI 不在本中断；由 Motor_Task foc pid 写出 Iq，本拍只读。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "stm32f4xx.h"
#include "motor_param.h"
#include "motor_it.h"
#include "motor_fast_vf.h"
#include "motor_foc_isr.h"
#include "motor_detect.h"
#include "motor_detect_isr.h"
#include "motor_bldc.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: ADC-DMA 传输完成中断回调，FOC 控制链驱动入口
 * @detail: 按校准、BLDC、快速 V/F 与正常 FOC 的顺序分派，本函数不承载控制算法。
 * @param {void *} p 回调用户参数，未使用
 * @param {uint32_t} flags DMA 事件标志，未使用
 * @return {void} 无
 */
void Motor_DmaInt(void *p, uint32_t flags)
{
    (void)p;
    (void)flags;

    if (Motor_Detect_IsActive()) {
        Motor_DetectIsr_Step(&motor, (TIM1->CR1 & TIM_CR1_DIR) != 0U);
        return;
    }

    if (Motor_Bldc_IsSelected()) {
        Motor_Bldc_RunIsr(&motor);
        return;
    }

    /*
     * V/F 实机回归基线：严格保持 3e5d564 的执行顺序——先分频，命中控制拍后
     * 才采样并直接更新 PWM。这样不会让新增的编码器/霍尔/软件保护计算占满
     * 每个 30kHz ADC 中断，便于确认问题是否源自实时预算被破坏。
     * 该路径仅用于短时低压验证；硬件 Break 仍须有效。
     */
    if (motor.ol_enable && motor.ol_mode == MOTOR_OL_MODE_VF) {
        Motor_FastVf_RunIsr(&motor);
        return;
    }

    Motor_FocIsr_Step(&motor);
}

#ifdef __cplusplus
}
#endif
