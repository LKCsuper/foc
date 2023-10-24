/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-05 11:22:46
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-07 22:34:28
 * @detail: 
 */
#ifndef __MOTOR_HALL_H__
#define __MOTOR_HALL_H__
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define HALL_TIM_CLOCK              (u32)90000000
#define HALL_SAMPLE_FREQ            (u32)10000

#define PHASE_SHIFT_ANGLE           (float)(60.0f/360.0f*2.0f*PI)

#define HALL_ANGLE_FACTOR           (float)((float)HALL_TIM_CLOCK/(float)HALL_SAMPLE_FREQ*PI/3.0f)

#define HALL_SPEED_FACTOR           (float)((float)HALL_TIM_CLOCK/6.0f)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int utils_read_hall(bool is_second_motor, int samples);
int mcpwm_foc_hall_detect(float current, uint8_t *hall_table, bool *result);
void Motor_Hall_IRQHandler(void);
#ifdef __cplusplus
}
#endif
#endif
