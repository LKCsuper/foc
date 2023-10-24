/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-10-06 10:30:31
 * @FilePath: \foc\App\motor\utils.c
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Current_PID_Calc(real32_T ref_temp, real32_T fdb_temp, real32_T* out_temp, CURRENT_PID_DEF* current_pid_temp)
{
	real32_T error;
	real32_T temp;
	current_pid_temp->Err = ref_temp - fdb_temp;										/* 差值 */
	current_pid_temp->Cal_Out = current_pid_temp->P_Gain * current_pid_temp->Err + current_pid_temp->I_Sum;
	
    current_pid_temp->Max_Output = 5;

	if (current_pid_temp->Cal_Out > current_pid_temp->Max_Output) 
	{
		*out_temp = current_pid_temp->Max_Output;
	} 
	else if (current_pid_temp->Cal_Out < current_pid_temp->Min_Output) 
	{
		*out_temp = current_pid_temp->Min_Output;
	} 
	else 
	{
		*out_temp = current_pid_temp->Cal_Out;
	}

	current_pid_temp->I_Sum += ((*out_temp - current_pid_temp->Cal_Out) * current_pid_temp->B_Gain + current_pid_temp->I_Gain * current_pid_temp->Err) * 0.000067;
}

/**
 * @description: 电流环
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID MOTOR_Pid_Electric(VOID)
{
	volatile motor_state_t *state_m = &m_motor_1.m_motor_state;
	volatile mc_configuration *conf_now = m_motor_1.m_conf;
    float ki = conf_now->foc_current_ki;                /* 电流ki */
    float dt;                                           /* 时间差 */

    /* 最大占空比.fabsf取绝对值，最大的占空比其实就是自己设置的占空比 */
    float max_duty = fabsf(state_m->max_duty);

    /* d轴增益范围 */
	float d_gain_scale = 1.0;
	if (conf_now->foc_d_gain_scale_start < 0.99) {
        /* 0.5 / 0.6 */
		float max_mod_norm = fabsf(state_m->duty_now / max_duty);
		/* 如果最大占空比很小 */
        if (max_duty < 0.01) {
			max_mod_norm = 1.0;
		}
		if (max_mod_norm > conf_now->foc_d_gain_scale_start) {
			d_gain_scale = utils_map(max_mod_norm, conf_now->foc_d_gain_scale_start, 1.0,
					1.0, conf_now->foc_d_gain_scale_max_mod);
			if (d_gain_scale < conf_now->foc_d_gain_scale_max_mod) {
				d_gain_scale = conf_now->foc_d_gain_scale_max_mod;
			}
		}
	}

    /* 求误差，目标值与实际值的差值 */
	float Ierr_d = state_m->id_target - state_m->id;
	float Ierr_q = state_m->iq_target - state_m->iq;

	state_m->vd_int += Ierr_d * (ki * d_gain_scale * dt);
	state_m->vq_int += Ierr_q * (ki * dt);

	// Feedback (PI controller). No D action needed because the plant is a first order system (tf = 1/(Ls+R))
	state_m->vd = state_m->vd_int + Ierr_d * conf_now->foc_current_kp * d_gain_scale;
	state_m->vq = state_m->vq_int + Ierr_q * conf_now->foc_current_kp;

    return;
}
#ifdef __cplusplus
}
#endif
