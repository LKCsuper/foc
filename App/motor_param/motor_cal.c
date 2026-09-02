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
 * @description: 按 VESC 电流环时间常数整定 FOC 电流环 Kp/Ki 与观测器增益
 * @detail: tc 为电流环时间常数，vesc 默认 1000(us) 暂定。
 *          带宽 bw = 1/(tc*1e-6)，kp = L*bw，ki = R*bw。
 *          观测器增益按磁链平方折算：gain = 1e-3/λ²，写回时 *1e6。
 *          mcconf 需含 foc_motor_r/l/flux_linkage，结果写回
 *          foc_current_kp/ki 与 foc_observer_gain。
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
 * @description: 计算电机电气速度环 Kp/Ki 的预留接口
 * @detail: 当前为空实现（占位），待补充整定公式。
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


