/*
 * @Description: 
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-02-19 15:57:40
 * @FilePath: \foc\App\motor\inc\utils.h
 */
#ifndef _MOTOR_UTILS_H
#define _MOTOR_UTILS_H
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
uint8_t calculate_deadtime(float deadtime_ns, float core_clock_freq);
#ifdef __cplusplus
}
#endif

#endif
