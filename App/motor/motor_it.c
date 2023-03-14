/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-12 17:17:56
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

// 1. 采集电流
// 2. clarke变换得到α β正交电量
// 3. park变换得到正交电流量 id iq 其中iq与转矩有关 id与磁通有关 实际控制经常把id置为0
// 4. pid控制
// 5. 通过角度得到电机转过的角度
// 6. 进行逆变换park变换
// 7. 对va vb 进行逆clarke变换,得到实际需要的三相电压
#if 1
void Motor_DmaInt(void *p, uint32_t flags)
{
	/* 创建电机结构体 */
	MOTOR_PARAM_S *stMotorNow = &motor; 

	/* 采集电流,这里理论上应该减去偏差 */
	float ia = GET_CURRENT1() * FAC_CURRENT;
	float ib = GET_CURRENT2() * FAC_CURRENT;
	float ic = GET_CURRENT3() * FAC_CURRENT;
	float Ialpha = 0;
	float Ibeta = 0;
	float va = 0.0f;
	float vb = 0.0f;
	float vc = 0.0f;

	/* 这里是电压偏移*/
	va = (ADC_VOLTS(ADC_IND_SENS1) - 0.0f) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	vb = (ADC_VOLTS(ADC_IND_SENS3) - 0.0f) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	vc = (ADC_VOLTS(ADC_IND_SENS2) - 0.0f) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	
	/* 总线电压,根据电流计算 */
	UTILS_LP_FAST(stMotorNow->v_bus, GET_INPUT_VOLTAGE(), 0.1);

	/* 三相电流 */
	stMotorNow->ia = ia;
	stMotorNow->ib = ib;
	stMotorNow->ic = ic;

	stMotorNow->va = va;
	stMotorNow->vb = vb;
	stMotorNow->vc = vc;

	/* clarke 变换 */
	clarke_transform(ia, ib, ic, &Ialpha, &Ibeta);

	// motor_now->m_motor_state.i_alpha = ia;
	// motor_now->m_motor_state.i_beta = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;

	// float vd_tmp = c * motor_now->m_motor_state.v_alpha + s * motor_now->m_motor_state.v_beta;
	// float vq_tmp = c * motor_now->m_motor_state.v_beta  - s * motor_now->m_motor_state.v_alpha;
		
	stMotorNow->Ialpha = Ialpha;
	stMotorNow->Ibeta = Ibeta;

	/* park变换,这里很显然不是开环 */
//	park_transform();
	// svm();
	// uint32_t duty1, duty2, duty3, top;
	// top = TIM1->ARR;

	// foc_svm();

	// /* 设置占空比 */
	// TIMER_UPDATE_DUTY_M1();

	return;
}

#else
/**
 * @description: dma中断
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void Motor_DmaInt(void *p, uint32_t flags)
{
	(void)p;
	(void)flags;

	/*
		每N个ADC ISR请求运行一次FOC循环。这种方式的pwm频率是
		与FOC计算分离，因为它需要25个usec，所以不能工作
		在>40khz。要设置100kHz pwm FOC_CONTROL_LOOP_FREQ_DIVIDER可以设置为3
		因此它跳过2个ISR调用，并在第3个调用中执行控制循环。
	*/
	static int skip = 0;
	if (++skip == FOC_CONTROL_LOOP_FREQ_DIVIDER) {
		skip = 0;
	} else {
		return;
	}

	uint32_t t_start = timer_time_now();

	bool is_v7 = !(TIM1->CR1 & TIM_CR1_DIR);
	int norm_curr_ofs = 0;

#ifdef HW_HAS_DUAL_MOTORS
	bool is_second_motor = is_v7;
	norm_curr_ofs = is_second_motor ? 3 : 0;
	motor_all_state_t *motor_now = is_second_motor ? (motor_all_state_t*)&m_motor_2 : (motor_all_state_t*)&m_motor_1;
	motor_all_state_t *motor_other = is_second_motor ? (motor_all_state_t*)&m_motor_1 : (motor_all_state_t*)&m_motor_2;
	m_isr_motor = is_second_motor ? 2 : 1;
#ifdef HW_HAS_3_SHUNTS
	volatile TIM_TypeDef *tim = is_second_motor ? TIM8 : TIM1;
#endif
#else
	motor_all_state_t *motor_other = (motor_all_state_t*)&m_motor_1;
	motor_all_state_t *motor_now = (motor_all_state_t*)&m_motor_1;;
	m_isr_motor = 1;
#ifdef HW_HAS_3_SHUNTS
	volatile TIM_TypeDef *tim = TIM1;
#endif
#endif

	mc_configuration *conf_now = motor_now->m_conf;
	mc_configuration *conf_other = motor_other->m_conf;

	// Update modulation for V7 and collect current samples. This is used by the HFI.
	if (motor_other->m_duty_next_set) {
		motor_other->m_duty_next_set = false;
#ifdef HW_HAS_DUAL_MOTORS
		float curr0;
		float curr1;

		if (is_second_motor) {
			curr0 = ((float)GET_CURRENT1() - conf_other->foc_offsets_current[0]) * FAC_CURRENT;
			curr1 = ((float)GET_CURRENT2() - conf_other->foc_offsets_current[1]) * FAC_CURRENT;
			TIMER_UPDATE_DUTY_M1(motor_other->m_duty1_next, motor_other->m_duty2_next, motor_other->m_duty3_next);
		} else {
			curr0 = ((float)GET_CURRENT1_M2() - conf_other->foc_offsets_current[0]) * FAC_CURRENT;
			curr1 = ((float)GET_CURRENT2_M2() - conf_other->foc_offsets_current[1]) * FAC_CURRENT;
			TIMER_UPDATE_DUTY_M2(motor_other->m_duty1_next, motor_other->m_duty2_next, motor_other->m_duty3_next);
		}
#else
		float curr0 = ((float)GET_CURRENT1() - conf_other->foc_offsets_current[0]) * FAC_CURRENT;
		float curr1 = ((float)GET_CURRENT2() - conf_other->foc_offsets_current[1]) * FAC_CURRENT;

		TIMER_UPDATE_DUTY_M1(motor_other->m_duty1_next, motor_other->m_duty2_next, motor_other->m_duty3_next);
#ifdef HW_HAS_DUAL_PARALLEL
		TIMER_UPDATE_DUTY_M2(motor_other->m_duty1_next, motor_other->m_duty2_next, motor_other->m_duty3_next);
#endif
#endif

		motor_other->m_i_alpha_sample_next = curr0;
		motor_other->m_i_beta_sample_next = ONE_BY_SQRT3 * curr0 + TWO_BY_SQRT3 * curr1;
	}

#ifndef HW_HAS_DUAL_MOTORS
#ifdef HW_HAS_PHASE_SHUNTS
	if (!conf_now->foc_sample_v0_v7 && is_v7) {
		return;
	}
#else
	if (is_v7) {
		return;
	}
#endif
#endif

	// Reset the watchdog
	timeout_feed_WDT(THREAD_MCPWM);

#ifdef AD2S1205_SAMPLE_GPIO
	// force a position sample in the AD2S1205 resolver IC (falling edge)
	palClearPad(AD2S1205_SAMPLE_GPIO, AD2S1205_SAMPLE_PIN);
#endif

#ifdef HW_HAS_DUAL_MOTORS
	float curr0 = 0;
	float curr1 = 0;

	if (is_second_motor) {
		curr0 = GET_CURRENT1_M2();
		curr1 = GET_CURRENT2_M2();
	} else {
		curr0 = GET_CURRENT1();
		curr1 = GET_CURRENT2();
	}
#else
	float curr0 = GET_CURRENT1();
	float curr1 = GET_CURRENT2();
#ifdef HW_HAS_DUAL_PARALLEL
	curr0 += GET_CURRENT1_M2();
	curr1 += GET_CURRENT2_M2();
#endif
#endif

#ifdef HW_HAS_3_SHUNTS
#ifdef HW_HAS_DUAL_MOTORS
	float curr2 = is_second_motor ? GET_CURRENT3_M2() : GET_CURRENT3();
#else
	float curr2 = GET_CURRENT3();
#ifdef HW_HAS_DUAL_PARALLEL
	curr2 += GET_CURRENT3_M2();
#endif
#endif
#endif

	motor_now->m_currents_adc[0] = curr0;
	motor_now->m_currents_adc[1] = curr1;
#ifdef HW_HAS_3_SHUNTS
	motor_now->m_currents_adc[2] = curr2;
#else
	motor_now->m_currents_adc[2] = 0.0;
#endif

	curr0 -= conf_now->foc_offsets_current[0];
	curr1 -= conf_now->foc_offsets_current[1];
#ifdef HW_HAS_3_SHUNTS
	curr2 -= conf_now->foc_offsets_current[2];
	motor_now->m_curr_unbalance = curr0 + curr1 + curr2;
#endif

	ADC_curr_norm_value[0 + norm_curr_ofs] = curr0;
	ADC_curr_norm_value[1 + norm_curr_ofs] = curr1;
#ifdef HW_HAS_3_SHUNTS
	ADC_curr_norm_value[2 + norm_curr_ofs] = curr2;
#else
	ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
#endif

	// Use the best current samples depending on the modulation state.
#ifdef HW_HAS_3_SHUNTS
	if (conf_now->foc_sample_high_current) {
		// High current sampling mode. Choose the lower currents to derive the highest one
		// in order to be able to measure higher currents.
		const float i0_abs = fabsf(ADC_curr_norm_value[0 + norm_curr_ofs]);
		const float i1_abs = fabsf(ADC_curr_norm_value[1 + norm_curr_ofs]);
		const float i2_abs = fabsf(ADC_curr_norm_value[2 + norm_curr_ofs]);

		if (i0_abs > i1_abs && i0_abs > i2_abs) {
			ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
		} else if (i1_abs > i0_abs && i1_abs > i2_abs) {
			ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
		} else if (i2_abs > i0_abs && i2_abs > i1_abs) {
			ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
		}
	} else {
#ifdef HW_HAS_PHASE_SHUNTS
		if (is_v7) {
			if (tim->CCR1 > 500 && tim->CCR2 > 500) {
				// Use the same 2 shunts on low modulation, as that will avoid jumps in the current reading.
				// This is especially important when using HFI.
				ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
			} else {
				if (tim->CCR1 < tim->CCR2 && tim->CCR1 < tim->CCR3) {
					ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
				} else if (tim->CCR2 < tim->CCR1 && tim->CCR2 < tim->CCR3) {
					ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
				} else if (tim->CCR3 < tim->CCR1 && tim->CCR3 < tim->CCR2) {
					ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
				}
			}
		} else {
			if (tim->CCR1 < (tim->ARR - 500) && tim->CCR2 < (tim->ARR - 500)) {
				// Use the same 2 shunts on low modulation, as that will avoid jumps in the current reading.
				// This is especially important when using HFI.
				ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
			} else {
				if (tim->CCR1 > tim->CCR2 && tim->CCR1 > tim->CCR3) {
					ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
				} else if (tim->CCR2 > tim->CCR1 && tim->CCR2 > tim->CCR3) {
					ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
				} else if (tim->CCR3 > tim->CCR1 && tim->CCR3 > tim->CCR2) {
					ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
				}
			}
		}
#else
		if (tim->CCR1 < (tim->ARR - 500) && tim->CCR2 < (tim->ARR - 500)) {
			// Use the same 2 shunts on low modulation, as that will avoid jumps in the current reading.
			// This is especially important when using HFI.
			ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
		} else {
			if (tim->CCR1 > tim->CCR2 && tim->CCR1 > tim->CCR3) {
				ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
			} else if (tim->CCR2 > tim->CCR1 && tim->CCR2 > tim->CCR3) {
				ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
			} else if (tim->CCR3 > tim->CCR1 && tim->CCR3 > tim->CCR2) {
				ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
			}
		}
#endif
	}
#endif

	float ia = ADC_curr_norm_value[0 + norm_curr_ofs] * FAC_CURRENT;
	float ib = ADC_curr_norm_value[1 + norm_curr_ofs] * FAC_CURRENT;
//	float ic = -(ia + ib);

#ifdef HW_HAS_PHASE_SHUNTS
	float dt;
	if (conf_now->foc_sample_v0_v7) {
		dt = 1.0 / conf_now->foc_f_zv;
	} else {
		dt = 1.0 / (conf_now->foc_f_zv / 2.0);
	}
#else
	float dt = 1.0 / (conf_now->foc_f_zv / 2.0);
#endif

	// todo 如果频率太快的话，
	// This has to be done for the skip function to have any chance at working with the
	// observer and control loops.
	// TODO: Test this.
	dt *= (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;

	UTILS_LP_FAST(motor_now->m_motor_state.v_bus, GET_INPUT_VOLTAGE(), 0.1);

	volatile float enc_ang = 0;
	volatile bool encoder_is_being_used = false;

	if (virtual_motor_is_connected()) {
		if (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER ) {
			enc_ang = virtual_motor_get_angle_deg();
			encoder_is_being_used = true;
		}
	} else {
		if (encoder_is_configured()) {
			enc_ang = encoder_read_deg();
			encoder_is_being_used = true;
		}
	}

	if (encoder_is_being_used) {
		float phase_tmp = enc_ang;
		if (conf_now->foc_encoder_inverted) {
			phase_tmp = 360.0 - phase_tmp;
		}
		phase_tmp *= conf_now->foc_encoder_ratio;
		phase_tmp -= conf_now->foc_encoder_offset;
		utils_norm_angle((float*)&phase_tmp);
		motor_now->m_phase_now_encoder = DEG2RAD_f(phase_tmp);
	}

	if (motor_now->m_state == MC_STATE_RUNNING) {
		// Clarke transform assuming balanced currents
		motor_now->m_motor_state.i_alpha = ia;
		motor_now->m_motor_state.i_beta = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;

		motor_now->m_i_alpha_sample_with_offset = motor_now->m_motor_state.i_alpha;
		motor_now->m_i_beta_sample_with_offset = motor_now->m_motor_state.i_beta;

		if (motor_now->m_i_alpha_beta_has_offset) {
			motor_now->m_motor_state.i_alpha = 0.5 * (motor_now->m_motor_state.i_alpha + motor_now->m_i_alpha_sample_next);
			motor_now->m_motor_state.i_beta = 0.5 * (motor_now->m_motor_state.i_beta + motor_now->m_i_beta_sample_next);
			motor_now->m_i_alpha_beta_has_offset = false;
		}

		const float duty_now = motor_now->m_motor_state.duty_now;
		const float duty_abs = fabsf(duty_now);
		const float vq_now = motor_now->m_motor_state.vq;
		const float speed_fast_now = motor_now->m_pll_speed;

		float id_set_tmp = motor_now->m_id_set;
		float iq_set_tmp = motor_now->m_iq_set;
		motor_now->m_motor_state.max_duty = conf_now->l_max_duty;

		if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
			utils_truncate_number_abs(&iq_set_tmp, -conf_now->lo_current_min);
		}

		UTILS_LP_FAST(motor_now->m_duty_abs_filtered, duty_abs, 0.01);
		utils_truncate_number_abs((float*)&motor_now->m_duty_abs_filtered, 1.0);

		UTILS_LP_FAST(motor_now->m_duty_filtered, duty_now, 0.01);
		utils_truncate_number_abs((float*)&motor_now->m_duty_filtered, 1.0);

		float duty_set = motor_now->m_duty_cycle_set;
		bool control_duty = motor_now->m_control_mode == CONTROL_MODE_DUTY ||
				motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY ||
				motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY_PHASE;

		// Short all phases (duty=0) the moment the direction or modulation changes sign. That will avoid
		// active braking or changing direction. Keep all phases shorted (duty == 0) until the
		// braking current reaches the set or maximum value, then go back to current control
		// mode. Stay in duty=0 for at least 10 cycles to avoid jumping in and out of that mode rapidly
		// around the threshold.
		if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
			if ((SIGN(speed_fast_now) != SIGN(motor_now->m_br_speed_before) ||
					SIGN(vq_now) != SIGN(motor_now->m_br_vq_before) ||
					fabsf(motor_now->m_duty_filtered) < 0.001 || motor_now->m_br_no_duty_samples < 10) &&
					motor_now->m_motor_state.i_abs_filter < fabsf(iq_set_tmp)) {
				control_duty = true;
				duty_set = 0.0;
				motor_now->m_br_no_duty_samples = 0;
			} else if (motor_now->m_br_no_duty_samples < 10) {
				control_duty = true;
				duty_set = 0.0;
				motor_now->m_br_no_duty_samples++;
			}
		} else {
			motor_now->m_br_no_duty_samples = 0;
		}

		motor_now->m_br_speed_before = speed_fast_now;
		motor_now->m_br_vq_before = vq_now;

		// Brake when set ERPM is below min ERPM
		if (motor_now->m_control_mode == CONTROL_MODE_SPEED &&
				fabsf(motor_now->m_speed_pid_set_rpm) < conf_now->s_pid_min_erpm) {
			control_duty = true;
			duty_set = 0.0;
		}

		// Reset integrator when leaving duty cycle mode, as the windup protection is not too fast. Making
		// better windup protection is probably better, but not easy.
		if (!control_duty && motor_now->m_was_control_duty) {
			motor_now->m_motor_state.vq_int = motor_now->m_motor_state.vq;
			if (conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_BEMF ||
					conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF) {
				motor_now->m_motor_state.vq_int -= motor_now->m_motor_state.speed_rad_s * conf_now->foc_motor_flux_linkage;
			}
		}
		motor_now->m_was_control_duty = control_duty;

		if (!control_duty) {
			motor_now->m_duty_i_term = motor_now->m_motor_state.iq / conf_now->lo_current_max;
		}

		if (control_duty) {
			float scale = 1.0 / motor_now->m_motor_state.v_bus;

			// Duty cycle control
			if (fabsf(duty_set) < (duty_abs - (scale * conf_now->foc_motor_r * conf_now->lo_current_max)) ||
					(SIGN(motor_now->m_motor_state.vq) * motor_now->m_motor_state.iq) < conf_now->lo_current_min) {
				// Truncating the duty cycle here would be dangerous, so run a PID controller.

				// Reset the integrator in duty mode to not increase the duty if the load suddenly changes. In braking
				// mode this would cause a discontinuity, so there we want to keep the value of the integrator.
				if (motor_now->m_control_mode == CONTROL_MODE_DUTY) {
					if (duty_now > 0.0) {
						if (motor_now->m_duty_i_term > 0.0) {
							motor_now->m_duty_i_term = 0.0;
						}
					} else {
						if (motor_now->m_duty_i_term < 0.0) {
							motor_now->m_duty_i_term = 0.0;
						}
					}
				}

				// Compute error
				float error = duty_set - motor_now->m_motor_state.duty_now;

				// Compute parameters
				float p_term = error * conf_now->foc_duty_dowmramp_kp * scale;
				motor_now->m_duty_i_term += error * (conf_now->foc_duty_dowmramp_ki * dt) * scale;

				// I-term wind-up protection
				utils_truncate_number((float*)&motor_now->m_duty_i_term, -1.0, 1.0);

				// Calculate output
				float output = p_term + motor_now->m_duty_i_term;
				utils_truncate_number(&output, -1.0, 1.0);
				iq_set_tmp = output * conf_now->lo_current_max;
			} else {
				// If the duty cycle is less than or equal to the set duty cycle just limit
				// the modulation and use the maximum allowed current.
				motor_now->m_duty_i_term = motor_now->m_motor_state.iq / conf_now->lo_current_max;
				motor_now->m_motor_state.max_duty = duty_set;
				if (duty_set > 0.0) {
					iq_set_tmp = conf_now->lo_current_max;
				} else {
					iq_set_tmp = -conf_now->lo_current_max;
				}
			}
		} else if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
			// Braking
			iq_set_tmp = -SIGN(speed_fast_now) * fabsf(iq_set_tmp);
		}

		// Set motor phase
		{
			if (!motor_now->m_phase_override) {
				foc_observer_update(motor_now->m_motor_state.v_alpha, motor_now->m_motor_state.v_beta,
						motor_now->m_motor_state.i_alpha, motor_now->m_motor_state.i_beta,
						dt, &(motor_now->m_observer_state), &motor_now->m_phase_now_observer, motor_now);

				// Compensate from the phase lag caused by the switching frequency. This is important for motors
				// that run on high ERPM compared to the switching frequency.
				motor_now->m_phase_now_observer += motor_now->m_pll_speed * dt * (0.5 + conf_now->foc_observer_offset);
				utils_norm_angle_rad((float*)&motor_now->m_phase_now_observer);
			}

			switch (conf_now->foc_sensor_mode) {
			case FOC_SENSOR_MODE_ENCODER:
				if (encoder_index_found() || virtual_motor_is_connected()) {
					motor_now->m_motor_state.phase = foc_correct_encoder(
							motor_now->m_phase_now_observer,
							motor_now->m_phase_now_encoder,
							motor_now->m_speed_est_fast,
							conf_now->foc_sl_erpm,
							motor_now);
				} else {
					// Rotate the motor in open loop if the index isn't found.
					motor_now->m_motor_state.phase = motor_now->m_phase_now_encoder_no_index;
				}

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;
			case FOC_SENSOR_MODE_HALL:
				motor_now->m_phase_now_observer = foc_correct_hall(motor_now->m_phase_now_observer, dt, motor_now,
						utils_read_hall(motor_now != &m_motor_1, conf_now->m_hall_extra_samples));
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;
			case FOC_SENSOR_MODE_SENSORLESS:
				if (motor_now->m_phase_observer_override) {
					motor_now->m_motor_state.phase = motor_now->m_phase_now_observer_override;
					motor_now->m_observer_state.x1 = motor_now->m_observer_x1_override;
					motor_now->m_observer_state.x2 = motor_now->m_observer_x2_override;
					iq_set_tmp += conf_now->foc_sl_openloop_boost_q * SIGN(iq_set_tmp);
					if (conf_now->foc_sl_openloop_max_q > conf_now->cc_min_current) {
						utils_truncate_number_abs(&iq_set_tmp, conf_now->foc_sl_openloop_max_q);
					}
				} else {
					motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
				}

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;

			case FOC_SENSOR_MODE_HFI_START:
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;

				if (motor_now->m_phase_observer_override) {
					motor_now->m_hfi.est_done_cnt = 0;
					motor_now->m_hfi.flip_cnt = 0;

					motor_now->m_min_rpm_hyst_timer = 0.0;
					motor_now->m_min_rpm_timer = 0.0;
					motor_now->m_phase_observer_override = false;
				}

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;

			case FOC_SENSOR_MODE_HFI:
			case FOC_SENSOR_MODE_HFI_V2:
			case FOC_SENSOR_MODE_HFI_V3:
			case FOC_SENSOR_MODE_HFI_V4:
			case FOC_SENSOR_MODE_HFI_V5:
				if (fabsf(RADPS2RPM_f(motor_now->m_speed_est_fast)) > conf_now->foc_sl_erpm_hfi) {
					motor_now->m_hfi.observer_zero_time = 0;
				} else {
					motor_now->m_hfi.observer_zero_time += dt;
				}

				if (motor_now->m_hfi.observer_zero_time < conf_now->foc_hfi_obs_ovr_sec) {
					motor_now->m_hfi.angle = motor_now->m_phase_now_observer;
					motor_now->m_hfi.double_integrator = -motor_now->m_speed_est_fast;
				}

				motor_now->m_motor_state.phase = foc_correct_encoder(
						motor_now->m_phase_now_observer,
						motor_now->m_hfi.angle,
						motor_now->m_speed_est_fast,
						conf_now->foc_sl_erpm_hfi,
						motor_now);

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;
			}

			if (motor_now->m_control_mode == CONTROL_MODE_HANDBRAKE) {
				// Force the phase to 0 in handbrake mode so that the current simply locks the rotor.
				motor_now->m_motor_state.phase = 0.0;
			} else if (motor_now->m_control_mode == CONTROL_MODE_OPENLOOP ||
					motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY) {
				motor_now->m_openloop_angle += dt * motor_now->m_openloop_speed;
				utils_norm_angle_rad((float*)&motor_now->m_openloop_angle);
				motor_now->m_motor_state.phase = motor_now->m_openloop_angle;
			} else if (motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_PHASE ||
					motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY_PHASE) {
				motor_now->m_motor_state.phase = motor_now->m_openloop_phase;
			}

			if (motor_now->m_phase_override) {
				motor_now->m_motor_state.phase = motor_now->m_phase_now_override;
			}

			utils_fast_sincos_better(motor_now->m_motor_state.phase,
					(float*)&motor_now->m_motor_state.phase_sin,
					(float*)&motor_now->m_motor_state.phase_cos);
		}

		// Apply MTPA. See: https://github.com/vedderb/bldc/pull/179
		const float ld_lq_diff = conf_now->foc_motor_ld_lq_diff;
		if (conf_now->foc_mtpa_mode != MTPA_MODE_OFF && ld_lq_diff != 0.0) {
			const float lambda = conf_now->foc_motor_flux_linkage;

			float iq_ref = iq_set_tmp;
			if (conf_now->foc_mtpa_mode == MTPA_MODE_IQ_MEASURED) {
				iq_ref = utils_min_abs(iq_set_tmp, motor_now->m_motor_state.iq_filter);
			}

			id_set_tmp = (lambda - sqrtf(SQ(lambda) + 8.0 * SQ(ld_lq_diff * iq_ref))) / (4.0 * ld_lq_diff);
			iq_set_tmp = SIGN(iq_set_tmp) * sqrtf(SQ(iq_set_tmp) - SQ(id_set_tmp));
		}

		const float mod_q = motor_now->m_motor_state.mod_q_filter;

		// Running FW from the 1 khz timer seems fast enough.
//		run_fw(motor_now, dt);
		id_set_tmp -= motor_now->m_i_fw_set;
		iq_set_tmp -= SIGN(mod_q) * motor_now->m_i_fw_set * conf_now->foc_fw_q_current_factor;

		// Apply current limits
		// TODO: Consider D axis current for the input current as well.
		if (mod_q > 0.001) {
			utils_truncate_number(&iq_set_tmp, conf_now->lo_in_current_min / mod_q, conf_now->lo_in_current_max / mod_q);
		} else if (mod_q < -0.001) {
			utils_truncate_number(&iq_set_tmp, conf_now->lo_in_current_max / mod_q, conf_now->lo_in_current_min / mod_q);
		}

		if (mod_q > 0.0) {
			utils_truncate_number(&iq_set_tmp, conf_now->lo_current_min, conf_now->lo_current_max);
		} else {
			utils_truncate_number(&iq_set_tmp, -conf_now->lo_current_max, -conf_now->lo_current_min);
		}

		float current_max_abs = fabsf(utils_max_abs(conf_now->lo_current_max, conf_now->lo_current_min));
		utils_truncate_number_abs(&id_set_tmp, current_max_abs);
		utils_truncate_number_abs(&iq_set_tmp, sqrtf(SQ(current_max_abs) - SQ(id_set_tmp)));

		motor_now->m_motor_state.id_target = id_set_tmp;
		motor_now->m_motor_state.iq_target = iq_set_tmp;

		control_current(motor_now, dt);
	} else {
		// Motor is not running

		// The current is 0 when the motor is undriven
		motor_now->m_motor_state.i_alpha = 0.0;
		motor_now->m_motor_state.i_beta = 0.0;
		motor_now->m_motor_state.id = 0.0;
		motor_now->m_motor_state.iq = 0.0;
		motor_now->m_motor_state.id_filter = 0.0;
		motor_now->m_motor_state.iq_filter = 0.0;
#ifdef HW_HAS_INPUT_CURRENT_SENSOR
		GET_INPUT_CURRENT_OFFSET(); // TODO: should this be done here?
#endif
		motor_now->m_motor_state.i_bus = 0.0;
		motor_now->m_motor_state.i_abs = 0.0;
		motor_now->m_motor_state.i_abs_filter = 0.0;

		// Track back emf
		update_valpha_vbeta(motor_now, 0.0, 0.0);

		// Run observer
		foc_observer_update(motor_now->m_motor_state.v_alpha, motor_now->m_motor_state.v_beta,
						motor_now->m_motor_state.i_alpha, motor_now->m_motor_state.i_beta,
						dt, &(motor_now->m_observer_state), 0, motor_now);
		motor_now->m_phase_now_observer = utils_fast_atan2(motor_now->m_x2_prev + motor_now->m_observer_state.x2,
														   motor_now->m_x1_prev + motor_now->m_observer_state.x1);

		// The observer phase offset has to be added here as well, with 0.5 switching cycles offset
		// compared to when running. Otherwise going from undriven to driven causes a current
		// spike.
		motor_now->m_phase_now_observer += motor_now->m_pll_speed * dt * conf_now->foc_observer_offset;
		utils_norm_angle_rad((float*)&motor_now->m_phase_now_observer);

		motor_now->m_x1_prev = motor_now->m_observer_state.x1;
		motor_now->m_x2_prev = motor_now->m_observer_state.x2;

		// Set motor phase
		{
			switch (conf_now->foc_sensor_mode) {
			case FOC_SENSOR_MODE_ENCODER:
				motor_now->m_motor_state.phase = foc_correct_encoder(
						motor_now->m_phase_now_observer,
						motor_now->m_phase_now_encoder,
						motor_now->m_speed_est_fast,
						conf_now->foc_sl_erpm,
						motor_now);
				break;
			case FOC_SENSOR_MODE_HALL:
				motor_now->m_phase_now_observer = foc_correct_hall(motor_now->m_phase_now_observer, dt, motor_now,
						utils_read_hall(motor_now != &m_motor_1, conf_now->m_hall_extra_samples));
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
				break;
			case FOC_SENSOR_MODE_SENSORLESS:
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
				break;
			case FOC_SENSOR_MODE_HFI:
			case FOC_SENSOR_MODE_HFI_V2:
			case FOC_SENSOR_MODE_HFI_V3:
			case FOC_SENSOR_MODE_HFI_V4:
			case FOC_SENSOR_MODE_HFI_V5:
			case FOC_SENSOR_MODE_HFI_START:{
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
				if (fabsf(RADPS2RPM_f(motor_now->m_pll_speed)) < (conf_now->foc_sl_erpm_hfi * 1.1)) {
					motor_now->m_hfi.est_done_cnt = 0;
					motor_now->m_hfi.flip_cnt = 0;
				}
			} break;

			}

			utils_fast_sincos_better(motor_now->m_motor_state.phase,
					(float*)&motor_now->m_motor_state.phase_sin,
					(float*)&motor_now->m_motor_state.phase_cos);
		}

		// HFI Restore
#ifdef HW_HAS_DUAL_MOTORS
		if (is_second_motor) {
			CURRENT_FILTER_ON_M2();
		} else {
			CURRENT_FILTER_ON();
		}
#else
		CURRENT_FILTER_ON();
#endif

		motor_now->m_hfi.ind = 0;
		motor_now->m_hfi.ready = false;
		motor_now->m_hfi.double_integrator = 0.0;
		motor_now->m_hfi.is_samp_n = false;
		motor_now->m_hfi.prev_sample = 0.0;
		motor_now->m_hfi.angle = motor_now->m_motor_state.phase;

		float s = motor_now->m_motor_state.phase_sin;
		float c = motor_now->m_motor_state.phase_cos;

		// Park transform
		float vd_tmp = c * motor_now->m_motor_state.v_alpha + s * motor_now->m_motor_state.v_beta;
		float vq_tmp = c * motor_now->m_motor_state.v_beta  - s * motor_now->m_motor_state.v_alpha;

		UTILS_NAN_ZERO(motor_now->m_motor_state.vd);
		UTILS_NAN_ZERO(motor_now->m_motor_state.vq);

		UTILS_LP_FAST(motor_now->m_motor_state.vd, vd_tmp, 0.2);
		UTILS_LP_FAST(motor_now->m_motor_state.vq, vq_tmp, 0.2);

		// Set the current controller integrator to the BEMF voltage to avoid
		// a current spike when the motor is driven again. Notice that we have
		// to take decoupling into account.
		motor_now->m_motor_state.vd_int = motor_now->m_motor_state.vd;
		motor_now->m_motor_state.vq_int = motor_now->m_motor_state.vq;

		if (conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_BEMF ||
				conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF) {
			motor_now->m_motor_state.vq_int -= motor_now->m_motor_state.speed_rad_s * conf_now->foc_motor_flux_linkage;
		}

		// Update corresponding modulation
		/* voltage_normalize = 1/(2/3*V_bus) */
		const float voltage_normalize = 1.5 / motor_now->m_motor_state.v_bus;

		motor_now->m_motor_state.mod_d = motor_now->m_motor_state.vd * voltage_normalize;
		motor_now->m_motor_state.mod_q = motor_now->m_motor_state.vq * voltage_normalize;
		UTILS_NAN_ZERO(motor_now->m_motor_state.mod_q_filter);
		UTILS_LP_FAST(motor_now->m_motor_state.mod_q_filter, motor_now->m_motor_state.mod_q, 0.2);
		utils_truncate_number_abs((float*)&motor_now->m_motor_state.mod_q_filter, 1.0);
	}

	// Calculate duty cycle
	motor_now->m_motor_state.duty_now = SIGN(motor_now->m_motor_state.vq) *
			NORM2_f(motor_now->m_motor_state.mod_d, motor_now->m_motor_state.mod_q) * TWO_BY_SQRT3;

	float phase_for_speed_est = 0.0;
	switch (conf_now->foc_speed_soure) {
	case SPEED_SRC_CORRECTED:
		phase_for_speed_est = motor_now->m_motor_state.phase;
		break;
	case SPEED_SRC_OBSERVER:
		phase_for_speed_est = motor_now->m_phase_now_observer;
		break;
	};

	// Run PLL for speed estimation
	foc_pll_run(phase_for_speed_est, dt, &motor_now->m_pll_phase, &motor_now->m_pll_speed, conf_now);
	motor_now->m_motor_state.speed_rad_s = motor_now->m_pll_speed;

	// Low latency speed estimation, for e.g. HFI and speed control.
	{
		float diff = utils_angle_difference_rad(phase_for_speed_est, motor_now->m_phase_before_speed_est);
		utils_truncate_number(&diff, -M_PI / 3.0, M_PI / 3.0);

		UTILS_LP_FAST(motor_now->m_speed_est_fast, diff / dt, 0.01);
		UTILS_NAN_ZERO(motor_now->m_speed_est_fast);

		UTILS_LP_FAST(motor_now->m_speed_est_faster, diff / dt, 0.2);
		UTILS_NAN_ZERO(motor_now->m_speed_est_faster);

		// pll wind-up protection
		utils_truncate_number_abs((float*)&motor_now->m_pll_speed, fabsf(motor_now->m_speed_est_fast) * 3.0);

		motor_now->m_phase_before_speed_est = phase_for_speed_est;
	}

	// Update tachometer (resolution = 60 deg as for BLDC)
	float ph_tmp = motor_now->m_motor_state.phase;
	utils_norm_angle_rad(&ph_tmp);
	int step = (int)floorf((ph_tmp + M_PI) / (2.0 * M_PI) * 6.0);
	utils_truncate_number_int(&step, 0, 5);
	int diff = step - motor_now->m_tacho_step_last;
	motor_now->m_tacho_step_last = step;

	if (diff > 3) {
		diff -= 6;
	} else if (diff < -2) {
		diff += 6;
	}

	motor_now->m_tachometer += diff;
	motor_now->m_tachometer_abs += abs(diff);

	// Track position control angle
	float angle_now = 0.0;
	if (encoder_is_configured()) {
		if (conf_now->m_sensor_port_mode == SENSOR_PORT_MODE_TS5700N8501_MULTITURN) {
			angle_now = encoder_read_deg_multiturn();
		} else {
			angle_now = enc_ang;
		}
	} else {
		angle_now = RAD2DEG_f(motor_now->m_motor_state.phase);
	}

	utils_norm_angle(&angle_now);

	if (conf_now->p_pid_ang_div > 0.98 && conf_now->p_pid_ang_div < 1.02) {
		motor_now->m_pos_pid_now = angle_now;
	} else {
		if (angle_now < 90.0 && motor_now->m_pid_div_angle_last > 270.0) {
			motor_now->m_pid_div_angle_accumulator += 360.0 / conf_now->p_pid_ang_div;
			utils_norm_angle((float*)&motor_now->m_pid_div_angle_accumulator);
		} else if (angle_now > 270.0 && motor_now->m_pid_div_angle_last < 90.0) {
			motor_now->m_pid_div_angle_accumulator -= 360.0 / conf_now->p_pid_ang_div;
			utils_norm_angle((float*)&motor_now->m_pid_div_angle_accumulator);
		}

		motor_now->m_pid_div_angle_last = angle_now;

		motor_now->m_pos_pid_now = motor_now->m_pid_div_angle_accumulator + angle_now / conf_now->p_pid_ang_div;
		utils_norm_angle((float*)&motor_now->m_pos_pid_now);
	}

#ifdef AD2S1205_SAMPLE_GPIO
	// Release sample in the AD2S1205 resolver IC.
	palSetPad(AD2S1205_SAMPLE_GPIO, AD2S1205_SAMPLE_PIN);
#endif

#ifdef HW_HAS_DUAL_MOTORS
	mc_interface_mc_timer_isr(is_second_motor);
#else
	mc_interface_mc_timer_isr(false);
#endif

	m_isr_motor = 0;
	m_last_adc_isr_duration = timer_seconds_elapsed_since(t_start);


    return;
}
#endif

#ifdef __cplusplus
}
#endif
