/*
 * @Description: BM75/HD75 已验证的 BLDC 默认启动配置
 * @Author: lkc
 */
#ifndef MOTOR_BLDC_DEFAULTS_H
#define MOTOR_BLDC_DEFAULTS_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#include "motor_bldc_sensorless.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 装入 BM75/HD75 已验证的六步与 BEMF 默认参数并初始化实例。
 * @detail: 不启动功率输出。换相输出回调接到板级 HAL。由任务调用，不在中断里调用。
 * @param {motor_bldc_six_step_t *} commutator 六步换相实例，不可为空
 * @param {motor_bldc_sensorless_t *} sensorless 无感控制实例，不可为空
 * @return {bool} 两个实例都初始化成功返回 true
 */
bool Motor_BldcDefaults_Init(motor_bldc_six_step_t *commutator, motor_bldc_sensorless_t *sensorless);

/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
