/*
 * @Description: 简易 FOC 数学工具函数与数据结构声明
 * @Author: lkc
 */
#ifndef __MOTOR_SIMPLE_UTILS_H__
#define __MOTOR_SIMPLE_UTILS_H__
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
// sign function
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ( (a) != (NOT_SET) )

// utility defines
#define _2_SQRT3 1.15470053838
#define _SQRT3 1.73205080757
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038
#define _PI_6 0.52359877559
/******************************************************************************/
// dq current structure 
typedef struct 
{
	float d;
	float q;
} DQCurrent_s;
// phase current structure 
typedef struct 
{
	float a;
	float b;
	float c;
} PhaseCurrent_s;
// dq voltage structs
typedef struct 
{
	float d;
	float q;
} DQVoltage_s;
/******************************************************************************/
/**
 * @description: 查表法快速近似 sin，输入角须在 [0, 2π]。
 * @param {float} a  输入角度 (rad)
 * @return {float} sin(a)
 * @author: lkc
 */
float _sin(float a);
/**
 * @description: 查表法快速近似 cos，输入角须在 [0, 2π]。
 * @param {float} a  输入角度 (rad)
 * @return {float} cos(a)
 * @author: lkc
 */
float _cos(float a);
/**
 * @description: 将弧度角归一化到 [0, 2π)。
 * @param {float} angle  输入角度 (rad)
 * @return {float} 归一化后的角度 (rad)
 * @author: lkc
 */
float _normalizeAngle(float angle);
/**
 * @description: 机械角换算为电角度。
 * @param {float} shaft_angle  机械角 (rad)
 * @param {int} pole_pairs     极对数
 * @return {float} 电角度 (rad)
 * @author: lkc
 */
float _electricalAngle(float shaft_angle, int pole_pairs);
/**
 * @description: 快速近似平方根。
 * @param {float} number  输入值
 * @return {float} sqrt(number) 的近似值
 * @author: lkc
 */
float _sqrtApprox(float number);
/******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif
