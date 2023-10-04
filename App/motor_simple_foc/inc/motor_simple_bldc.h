/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-09-30 17:09:35
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-03 19:34:55
 * @detail: 
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
extern long sensor_direction;
extern float voltage_power_supply;
extern float voltage_limit;
extern float voltage_sensor_align;
extern int  pole_pairs;
extern unsigned long open_loop_timestamp;
extern float velocity_limit;
extern float current_limit;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/******************************************************************************/


/******************************************************************************/

/******************************************************************************/
void Motor_init(void);
void Motor_initFOC(void);
void loopFOC(void);
void move(float new_target);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
void Motor_Simple_BldcInit(void);
//void setPwm(float Ua, float Ub, float Uc);
/******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif
