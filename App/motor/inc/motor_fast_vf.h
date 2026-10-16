/*
 * @Description: 快速 V/F 实机诊断路径
 * @Author: lkc
 */
#ifndef MOTOR_FAST_VF_H
#define MOTOR_FAST_VF_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 推进一次简化 V/F 诊断控制拍并提交电压。
 * @detail: 仅供历史实机基线验证，不代替正常 Motor_Control_Step 路径。
 *          在控制拍调用。频率或极对数无效时改走中点输出。motor 不可为空，违反约定触发 COMMON_ASSERT。
 * @param {MOTOR_PARAM_S *} motor 电机实例，不可为空
 * @param {float} control_hz 实际控制频率 (Hz)
 * @return {void} 无
 */
void Motor_FastVf_Step(MOTOR_PARAM_S *motor, float control_hz);

/**
 * @description: 执行快速 V/F 的一次 ADC 中断包装。
 * @detail: 保持历史顺序：先控制分频，再采样，母线有效时推进 V/F，最后记录耗时。
 *          仅在 ADC-DMA 中断、且未进入校准或 BLDC 时调用。
 * @param {MOTOR_PARAM_S *} motor 电机实例，不可为空
 * @return {void} 无
 */
void Motor_FastVf_RunIsr(MOTOR_PARAM_S *motor);

/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
