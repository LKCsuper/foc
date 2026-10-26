/* 默认单电机的传感器启动编排。 */
#ifndef MOTOR_SENSOR_H
#define MOTOR_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 初始化编码器、恢复电角校准，并按电机型号设置默认反馈源。
 * @detail: 在 Bsp_Init、MOTOR_Param_Init 后且控制任务启动前调用。
 *          保留编码器供诊断比较使用；霍尔 IO 仍由 BSP 初始化，霍尔表来自电机参数。
 *          不执行校准、不启动电机、不写 Flash；无感观测器仍由参数初始化流程建立。
 * @return {bool} 默认反馈源设置成功返回 true，不代表传感器已采样或校准有效。
 */
bool Motor_Sensor_Init(void);

#ifdef __cplusplus
}
#endif
#endif
