/*
 * @Description: 快速 V/F 诊断路径的 ADC 中断包装
 * @Author: lkc
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_fast_vf.h"

#include <stddef.h>

#include "hal_motor.h"
#include "motor_foc_isr.h"
#include "motor_sense.h"
#include "motor_timing.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 执行快速 V/F 的一次 ADC 中断包装。
 * @detail: 保持历史顺序：先控制分频，再采样，母线有效时推进 V/F，最后记录耗时。
 * @param {MOTOR_PARAM_S *} motor 电机实例，不可为空
 * @return {void} 无
 */
void Motor_FastVf_RunIsr(MOTOR_PARAM_S *motor)
{
    uint32_t runtime_start;

    if (motor == NULL) {
        return;
    }

    runtime_start = Hal_Motor_TimeUs();
    if (!Motor_FocIsr_ShouldRun()) {
        Motor_Timing_RecordRuntime(runtime_start);
        return;
    }

    Motor_Sense_Update(motor);
    if (motor->v_bus >= 1.0f) {
        Motor_FastVf_Step(motor, Motor_DmaInt_GetControlFrequencyHz());
    }
    Motor_Timing_RecordRuntime(runtime_start);
}
