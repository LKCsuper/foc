/* 默认单电机的传感器启动编排：编码器初始化、电角校准恢复与反馈源选择。 */
/* Includes ------------------------------------------------------------------*/
#include "motor_sensor.h"
#include "motor_encoder.h"
#include "motor.h"

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 初始化编码器、恢复电角校准并按电机型号设置默认反馈源。
 * @detail: 不执行校准、不启动电机、不写 Flash；霍尔表来自电机参数。
 * @return {bool} true=默认反馈源设置成功，不代表传感器已采样或校准有效。
 */
bool Motor_Sensor_Init(void)
{
    Motor_Encoder_Init();
    Motor_Encoder_ApplyCalibration(motor.encoder_offset_rad, motor.encoder_inverted, motor.encoder_calibrated);
    return Motor_ApplyProfileSensor() == MOTOR_OK;
}
