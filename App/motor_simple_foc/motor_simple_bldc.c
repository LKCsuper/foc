/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-09-30 17:09:25
 * @LastEditors: lkc
 * @LastEditTime: 2023-09-30 22:31:34
 * @detail: 
 * 
 * 能旋转，但是实际跟vesc效果差太多
 * 
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
#define PWM_Period 5600
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/************************************************
main中调用的接口函数都在当前文件中
=================================================
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/119220638
创建日期：20210801
作    者：loop222 @郑州
************************************************/
/******************************************************************************/
float voltage_power_supply;
float voltage_limit;
int  pole_pairs;
unsigned long open_loop_timestamp;
float velocity_limit;
/******************************************************************************/
/******************************************************************************/
void move(float new_target)
{
	switch(controller)
	{
		case Type_velocity_openloop:
		{
			// velocity control in open loop
			shaft_velocity_sp = new_target;
			voltage.q = velocityOpenloop(shaft_velocity_sp); // returns the voltage that is set to the motor
			voltage.d = 0;
			break;
		}
		case Type_angle_openloop:
		{
			// angle control in open loop
			shaft_angle_sp = new_target;
			voltage.q = angleOpenloop(shaft_angle_sp); // returns the voltage that is set to the motor
			voltage.d = 0;
			break;
		}
	}
}
/******************************************************************************/
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	float Uout;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;
	
	if(Ud) // only if Ud and Uq set 
	{// _sqrt is an approx of sqrt (3-4% error)
		Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
	}
	else
	{// only Uq available - no need for atan2 and sqrt
		Uout = Uq / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + _PI_2);
	}
	if(Uout> 0.577)Uout= 0.577;
	if(Uout<-0.577)Uout= -0.577;
	
	sector = (angle_el / _PI_3) + 1;
	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
	T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;
	T0 = 1 - T1 - T2;
	
	// calculate the duty cycles(times)
	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	
	// TIM_SetCompare1(TIM2, Ta * PWM_Period);
	// TIM_SetCompare2(TIM2, Tb * PWM_Period);
	// TIM_SetCompare3(TIM2, Tc * PWM_Period);
	//TIMER_UPDATE_DUTY_M1(Ta * PWM_Period, Tb * PWM_Period, Tc * PWM_Period);

	//PRINTF("{%f,%f,%f}\r\n", Ta, Tb, Tc);

	TIMER_UPDATE_DUTY_M1(Ta * TIM1->ARR, Tb * TIM1->ARR, Tc * TIM1->ARR);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}
/******************************************************************************/
float velocityOpenloop(float target_velocity)
{
	unsigned long now_us;
	float Ts,Uq;

	now_us = TIM5->CNT; //_micros();
	if(now_us < open_loop_timestamp)
	{
		Ts = (float)(open_loop_timestamp - now_us)/9*1e-6;
	}
	else
	{
		Ts = (float)(0xFFFFFF - now_us + open_loop_timestamp)/9*1e-6;
	}
	open_loop_timestamp = now_us;  //save timestamp for next call
	// quick fix for strange cases (micros overflow)
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3; 
	
	target_velocity = 100;
	// calculate the necessary angle to achieve target velocity
	shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts); 
	
	PRINTF("{%f, %f, %f}\r\n", shaft_angle, target_velocity, Ts);

	Uq = voltage_limit;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
  	setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
	
	return Uq;
}
/******************************************************************************/
/**
 * @description: 角度开环 
 * @detail: 
 * @param {float} target_angle
 * @return {*}
 * @author: lkc
 */
float angleOpenloop(float target_angle)
{
	unsigned long now_us;
	float Ts,Uq;
	
	now_us = TIM5->CNT; //_micros();

	//PRINTF("%d\r\n", now_us);

	if(now_us < open_loop_timestamp)
	{
		Ts = (float)(open_loop_timestamp - now_us)/9*1e-6;
	}
	else
	{
		Ts = (float)(0xFFFFFF - now_us + open_loop_timestamp)/9*1e-6;
	}
		
	open_loop_timestamp = now_us;  //save timestamp for next call
	// quick fix for strange cases (micros overflow)
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3; 
	
	// calculate the necessary angle to move from current position towards target angle
  	// with maximal velocity (velocity_limit)
  	if(fabs( target_angle - shaft_angle ) > velocity_limit*Ts)
	{
		shaft_angle += _sign(target_angle - shaft_angle) * velocity_limit * Ts;
		//shaft_velocity = velocity_limit;
  	}
	else
	{
		shaft_angle = target_angle;
		//shaft_velocity = 0;
  	}

	Uq = voltage_limit;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
	setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
	
  	return Uq;
}
/******************************************************************************/

void Motor_Simple_BldcInit(void)
{
	voltage_power_supply=24;   			//V
	voltage_limit=1;           		//V，最大值需小于12/1.732=6.9。大功率航模电机设置的小一点0.5-1；小功率云台电机设置的大一点1-3
	velocity_limit=20;         			//rad/s angleOpenloop() use it
	controller=Type_velocity_openloop;  //Type_angle_openloop;  //
	pole_pairs=2;              			//极对数
}
#ifdef __cplusplus
}
#endif


