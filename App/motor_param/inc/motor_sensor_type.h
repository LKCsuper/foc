/*
 * @Description: 电机默认传感器类型。电机头文件选用，不表示本次调试是否临时改成开环。
 * @Author: lkc
 */
#ifndef MOTOR_SENSOR_TYPE_H
#define MOTOR_SENSOR_TYPE_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MOTOR_SENSOR_SENSORLESS (1U) /* 磁链观测器；接管前电角仍由 I/F 积分 */
#define MOTOR_SENSOR_ENCODER    (2U) /* MT6701 电角与转速 */
#define MOTOR_SENSOR_HALL       (3U) /* 霍尔表电角与转速 */
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
