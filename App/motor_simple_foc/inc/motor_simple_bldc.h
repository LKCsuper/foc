/*
 * @Description: 简易 BLDC/FOC 开环拖动与 SVPWM 接口声明
 * @Author: lkc
 */
#ifndef __MOTOR_SIMLPE_BLDC_H__
#define __MOTOR_SIMLPE_BLDC_H__
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/**
 *  Direction structure
 */
typedef enum
{
    CW      = 1,  //clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   //not yet known or invalid state
} Direction;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern float voltage_power_supply;
extern float voltage_limit;
extern int  pole_pairs;
extern unsigned long open_loop_timestamp;
extern float velocity_limit;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/******************************************************************************/
/**
 * @description: 按当前控制模式更新开环目标并输出电压。
 * @param {float} new_target  速度 (rad/s) 或角度 (rad) 目标值
 * @return {*}
 * @author: lkc
 */
void move(float new_target);
/**
 * @description: 按 dq 电压与电角度计算 SVPWM 占空比并写入定时器。
 * @param {float} Uq       q 轴电压 (V)
 * @param {float} Ud       d 轴电压 (V)
 * @param {float} angle_el 电角度 (rad)
 * @return {*}
 * @author: lkc
 */
void setPhaseVoltage(float Uq, float Ud, float angle_el);
/**
 * @description: 初始化旧 SimpleFOC 学习实现的独立开环默认参数。
 * @detail: 仅供旧学习代码调用，不初始化主控制链的 motor；主固件使用 MOTOR_Param_Init。
 * @return {*}
 * @author: lkc
 */
void Motor_Simple_BldcInit(void);
/******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif
