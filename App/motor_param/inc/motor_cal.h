#ifndef __MOTOR_CAL_H__
#define __MOTOR_CAL_H__
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
void VESC_Calc_Foc_cc_kp_ki_gain(mc_configuration *mcconf, float tc);
#ifdef __cplusplus
}
#endif
#endif
