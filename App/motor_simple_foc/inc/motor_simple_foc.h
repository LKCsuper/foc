#ifndef __MOTOR_SIMPLE_FOC_H__
#define __MOTOR_SIMPLE_FOC_H__
/*
 * @Description: 简易 FOC 运动控制类型与全局状态声明
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/******************************************************************************/
/**
 *  Motiron control type
 */
typedef enum
{
	Type_torque,//!< Torque control
	Type_velocity,//!< Velocity motion control
	Type_angle,//!< Position/angle motion control
	Type_velocity_openloop,
	Type_angle_openloop
} MotionControlType;

/**
 *  Motiron control type
 */
typedef enum
{
	Type_voltage, //!< Torque control using voltage
	Type_dc_current, //!< Torque control using DC current (one current magnitude)
	Type_foc_current //!< torque control using dq currents
} TorqueControlType;

extern TorqueControlType torque_controller;
extern MotionControlType controller;
/******************************************************************************/
extern float shaft_angle;//!< current motor angle
extern float electrical_angle;
extern float shaft_velocity;
extern float current_sp;
extern float shaft_velocity_sp;
extern float shaft_angle_sp;
extern DQVoltage_s voltage;
extern DQCurrent_s current;

extern float sensor_offset;
extern float zero_electric_angle;
/******************************************************************************/
/**
 * @description: 获取当前机械轴角。
 * @return {float} 机械角 (rad)
 * @author: lkc
 */
float shaftAngle(void);
/**
 * @description: 获取当前机械轴角速度。
 * @return {float} 角速度 (rad/s)
 * @author: lkc
 */
float shaftVelocity(void);
/**
 * @description: 计算当前电角度（含传感器偏移与零位校准）。
 * @return {float} 电角度 (rad)
 * @author: lkc
 */
float electricalAngle(void);
/******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif
