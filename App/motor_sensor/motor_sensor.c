#include "motor_sensor.h"
#include "motor_encoder.h"
#include "motor.h"

bool Motor_Sensor_Init(void)
{
    Motor_Encoder_Init();
    Motor_Encoder_ApplyCalibration(motor.encoder_offset_rad, motor.encoder_inverted, motor.encoder_calibrated);
    return Motor_ApplyProfileSensor() == MOTOR_OK;
}
