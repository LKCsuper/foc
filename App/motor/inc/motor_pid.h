/*
 * @Description: 
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-02-19 17:13:24
 * @FilePath: \foc\App\motor\inc\utils.h
 */
#ifndef _MOTOR_PID_H
#define _MOTOR_PID_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Current_PID_Calc(real32_T ref_temp, real32_T fdb_temp, real32_T* out_temp, CURRENT_PID_DEF* current_pid_temp);
#ifdef __cplusplus
}
#endif

#endif
