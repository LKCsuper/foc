/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-04 11:30:42
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-04 15:24:12
 * @detail: 
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 
 * @detail: 这个tc vesc默认是1000，暂定
 * @param {mc_configuration} *mcconf
 * @param {float} tc
 * @return {*}
 * @author: lkc
 */
void VESC_Calc_Foc_cc_kp_ki_gain(mc_configuration *mcconf, float tc) 
{
	/* 获取电阻 电感 磁链 */
    float r = mcconf->foc_motor_r;
	float l = mcconf->foc_motor_l;
	float lambda = mcconf->foc_motor_flux_linkage;  

	float bw = 1.0 / (tc * 1e-6);
	float kp = l * bw;
	float ki = r * bw;
	float gain = 1.0e-3 / SQ(lambda);
	//	float gain = (0.00001 / r) / SQ(lambda); // Old method

	mcconf->foc_current_kp = kp;
	mcconf->foc_current_ki = ki;
	mcconf->foc_observer_gain = gain * 1e6;
}


/**
 * @description: 
 * @detail: 
 * 
 * 
 * 
 * @return {*}
 * @author: lkc
 */
VOID MOTOR_Cal_Electric_KP_KI(VOID)
{

    return;
}
#ifdef __cplusplus
}
#endif


