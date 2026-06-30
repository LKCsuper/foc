// vesc 核心中断
void mcpwm_foc_adc_int_handler(void *p, uint32_t flags) {
	(void)p;
	(void)flags;

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
}

// Private functions

static void timer_update(motor_all_state_t *motor, float dt) {
	foc_run_fw(motor, dt);

	const mc_configuration *conf_now = motor->m_conf;

	// Calculate temperature-compensated parameters here
	if (mc_interface_temp_motor_filtered() > -30.0) {
		float comp_fact = 1.0 + 0.00386 * (mc_interface_temp_motor_filtered() - conf_now->foc_temp_comp_base_temp);
		motor->m_res_temp_comp = conf_now->foc_motor_r * comp_fact;
		motor->m_current_ki_temp_comp = conf_now->foc_current_ki * comp_fact;
	} else {
		motor->m_res_temp_comp = conf_now->foc_motor_r;
		motor->m_current_ki_temp_comp = conf_now->foc_current_ki;
	}

	// Check if it is time to stop the modulation. Notice that modulation is kept on as long as there is
	// field weakening current.
	utils_sys_lock_cnt();
	utils_step_towards((float*)&motor->m_current_off_delay, 0.0, dt);
	if (!motor->m_phase_override && motor->m_state == MC_STATE_RUNNING &&
			(motor->m_control_mode == CONTROL_MODE_CURRENT ||
					motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE ||
					motor->m_control_mode == CONTROL_MODE_HANDBRAKE ||
					motor->m_control_mode == CONTROL_MODE_OPENLOOP ||
					motor->m_control_mode == CONTROL_MODE_OPENLOOP_PHASE)) {

		// This is required to allow releasing the motor when cc_min_current is 0
		float min_current = conf_now->cc_min_current;
		if (min_current < 0.001 && get_motor_now()->m_motor_released) {
			min_current = 0.001;
		}

		if (fabsf(motor->m_iq_set) < min_current &&
				fabsf(motor->m_id_set) < min_current &&
				motor->m_i_fw_set < min_current &&
				motor->m_current_off_delay < dt) {
			motor->m_control_mode = CONTROL_MODE_NONE;
			motor->m_state = MC_STATE_OFF;
			stop_pwm_hw(motor);
		}
	}
	utils_sys_unlock_cnt();

	// Use this to study the openloop timers under experiment plot
#if 0
	{
		static bool plot_started = false;
		static int plot_div = 0;
		static float plot_int = 0.0;
		static int get_fw_version_cnt = 0;

		if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
			get_fw_version_cnt = commands_get_fw_version_sent_cnt();
			plot_started = false;
		}

		plot_div++;
		if (plot_div >= 10) {
			plot_div = 0;
			if (!plot_started) {
				plot_started = true;
				commands_init_plot("Time", "Val");
				commands_plot_add_graph("m_min_rpm_timer");
				commands_plot_add_graph("m_min_rpm_hyst_timer");
			}

			commands_plot_set_graph(0);
			commands_send_plot_points(plot_int, motor->m_min_rpm_timer);
			commands_plot_set_graph(1);
			commands_send_plot_points(plot_int, motor->m_min_rpm_hyst_timer);
			plot_int++;
		}
	}
#endif

	// Use this to study the observer state in a XY-plot
#if 0
	{
		static bool plot_started = false;
		static int plot_div = 0;
		static int get_fw_version_cnt = 0;

		if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
			get_fw_version_cnt = commands_get_fw_version_sent_cnt();
			plot_started = false;
		}

		plot_div++;
		if (plot_div >= 10) {
			plot_div = 0;
			if (!plot_started) {
				plot_started = true;
				commands_init_plot("X1", "X2");
				commands_plot_add_graph("Observer");
				commands_plot_add_graph("Observer Mag");
			}

			commands_plot_set_graph(0);
			commands_send_plot_points(m_motor_1.m_observer_x1, m_motor_1.m_observer_x2);
			float mag = NORM2_f(m_motor_1.m_observer_x1, m_motor_1.m_observer_x2);
			commands_plot_set_graph(1);
			commands_send_plot_points(0.0, mag);
		}
	}
#endif

	float t_lock = conf_now->foc_sl_openloop_time_lock;
	float t_ramp = conf_now->foc_sl_openloop_time_ramp;
	float t_const = conf_now->foc_sl_openloop_time;

	float openloop_current = fabsf(motor->m_motor_state.iq_filter);
	openloop_current += conf_now->foc_sl_openloop_boost_q;
	if (conf_now->foc_sl_openloop_max_q > 0.0) {
		utils_truncate_number(&openloop_current, 0.0, conf_now->foc_sl_openloop_max_q);
	}

	float openloop_rpm_max = utils_map(openloop_current,
			0.0, conf_now->l_current_max,
			conf_now->foc_openloop_rpm_low * conf_now->foc_openloop_rpm,
			conf_now->foc_openloop_rpm);

	utils_truncate_number_abs(&openloop_rpm_max, conf_now->foc_openloop_rpm);

	float openloop_rpm = openloop_rpm_max;
	if (conf_now->foc_sensor_mode != FOC_SENSOR_MODE_ENCODER) {
		float time_fwd = t_lock + t_ramp + t_const - motor->m_min_rpm_timer;
		if (time_fwd < t_lock) {
			openloop_rpm = 0.0;
		} else if (time_fwd < (t_lock + t_ramp)) {
			openloop_rpm = utils_map(time_fwd, t_lock,
					t_lock + t_ramp, 0.0, openloop_rpm);
		}
	}

	utils_truncate_number_abs(&openloop_rpm, openloop_rpm_max);

	float add_min_speed = 0.0;
	if (motor->m_motor_state.duty_now > 0.0) {
		add_min_speed = RPM2RADPS_f(openloop_rpm) * dt;
	} else {
		add_min_speed = -RPM2RADPS_f(openloop_rpm) * dt;
	}

	// Open loop encoder angle for when the index is not found
	motor->m_phase_now_encoder_no_index += add_min_speed;
	utils_norm_angle_rad((float*)&motor->m_phase_now_encoder_no_index);

	if (fabsf(motor->m_pll_speed) < RPM2RADPS_f(openloop_rpm_max) &&
			motor->m_min_rpm_hyst_timer < conf_now->foc_sl_openloop_hyst) {
		motor->m_min_rpm_hyst_timer += dt;
	} else if (motor->m_min_rpm_hyst_timer > 0.0) {
		motor->m_min_rpm_hyst_timer -= dt;
	}

	// Don't use this in brake mode.
	if (motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE ||
			(motor->m_state == MC_STATE_RUNNING && fabsf(motor->m_motor_state.duty_now) < 0.001)) {
		motor->m_min_rpm_hyst_timer = 0.0;
		motor->m_min_rpm_timer = 0.0;
		motor->m_phase_observer_override = false;
	}

	bool started_now = false;
	if (motor->m_min_rpm_hyst_timer >= conf_now->foc_sl_openloop_hyst &&
			motor->m_min_rpm_timer <= 0.0001) {
		motor->m_min_rpm_timer = t_lock + t_ramp + t_const;
		started_now = true;
	}

	if (motor->m_state != MC_STATE_RUNNING) {
		motor->m_min_rpm_timer = 0.0;
	}

	if (motor->m_min_rpm_timer > 0.0) {
		motor->m_phase_now_observer_override += add_min_speed;

		// When the motor gets stuck it tends to be 90 degrees off, so start the open loop
		// sequence by correcting with 60 degrees.
		if (started_now) {
			if (motor->m_motor_state.duty_now > 0.0) {
				motor->m_phase_now_observer_override += M_PI / 3.0;
			} else {
				motor->m_phase_now_observer_override -= M_PI / 3.0;
			}
		}

		utils_norm_angle_rad((float*)&motor->m_phase_now_observer_override);
		motor->m_phase_observer_override = true;
		motor->m_min_rpm_timer -= dt;
		motor->m_min_rpm_hyst_timer = 0.0;

		// Set observer state to help it start tracking when leaving open loop.
		float s, c;
		utils_fast_sincos_better(motor->m_phase_now_observer_override + SIGN(motor->m_motor_state.duty_now) * M_PI / 4.0, &s, &c);
		motor->m_observer_x1_override = c * conf_now->foc_motor_flux_linkage;
		motor->m_observer_x2_override = s * conf_now->foc_motor_flux_linkage;
	} else {
		motor->m_phase_now_observer_override = motor->m_phase_now_observer;
		motor->m_phase_observer_override = false;
	}

	// Samples
	if (motor->m_state == MC_STATE_RUNNING) {
		const volatile float vd_tmp = motor->m_motor_state.vd;
		const volatile float vq_tmp = motor->m_motor_state.vq;
		const volatile float id_tmp = motor->m_motor_state.id;
		const volatile float iq_tmp = motor->m_motor_state.iq;

		motor->m_samples.avg_current_tot += NORM2_f(id_tmp, iq_tmp);
		motor->m_samples.avg_voltage_tot += NORM2_f(vd_tmp, vq_tmp);
		motor->m_samples.sample_num++;
	}

	// Observer gain scaling, based on bus voltage and duty cycle
	float gamma_tmp = utils_map(fabsf(motor->m_motor_state.duty_now),
								0.0, 40.0 / motor->m_motor_state.v_bus,
								0, conf_now->foc_observer_gain);
	if (gamma_tmp < (conf_now->foc_observer_gain_slow * conf_now->foc_observer_gain)) {
		gamma_tmp = conf_now->foc_observer_gain_slow * conf_now->foc_observer_gain;
	}

	// 4.0 scaling is kind of arbitrary, but it should make configs from old VESC Tools more likely to work.
	motor->m_gamma_now = gamma_tmp * 4.0;

	// Run resistance observer
	// See "An adaptive flux observer for the permanent magnet synchronous motor"
	// https://doi.org/10.1002/acs.2587
	{
		float res_est_gain = 0.00002;
		float i_abs_sq = SQ(motor->m_motor_state.i_abs);
		motor->m_r_est = motor->m_r_est_state - 0.5 * res_est_gain * conf_now->foc_motor_l * i_abs_sq;
		float res_dot = -res_est_gain * (motor->m_r_est * i_abs_sq + motor->m_speed_est_fast *
				(motor->m_motor_state.i_beta * motor->m_observer_state.x1 - motor->m_motor_state.i_alpha * motor->m_observer_state.x2) -
				(motor->m_motor_state.i_alpha * motor->m_motor_state.v_alpha + motor->m_motor_state.i_beta * motor->m_motor_state.v_beta));
		motor->m_r_est_state += res_dot * dt;

		utils_truncate_number((float*)&motor->m_r_est_state, conf_now->foc_motor_r * 0.25, conf_now->foc_motor_r * 3.0);
	}
}


/**
 * Run the current control loop.
 *
 * @param state_m
 * The motor state.
 *
 * Parameters that shall be set before calling this function:
 * id_target
 * iq_target
 * max_duty
 * phase
 * i_alpha
 * i_beta
 * v_bus
 * speed_rad_s
 *
 * Parameters that will be updated in this function:
 * i_bus
 * i_abs
 * i_abs_filter
 * v_alpha
 * v_beta
 * mod_d
 * mod_q
 * id
 * iq
 * id_filter
 * iq_filter
 * vd
 * vq
 * vd_int
 * vq_int
 * svm_sector
 *
 * @param dt
 * The time step in seconds.
 */
static void control_current(motor_all_state_t *motor, float dt) {
	volatile motor_state_t *state_m = &motor->m_motor_state;
	volatile mc_configuration *conf_now = motor->m_conf;

	float s = state_m->phase_sin;
	float c = state_m->phase_cos;

	float abs_rpm = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));

	bool do_hfi = (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI ||
			conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2 ||
			conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V3 ||
			conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4 ||
			conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V5 ||
			(conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_START &&
					motor->m_control_mode != CONTROL_MODE_CURRENT_BRAKE &&
					fabsf(state_m->iq_target) > conf_now->cc_min_current)) &&
							!motor->m_phase_override &&
							abs_rpm < (conf_now->foc_sl_erpm_hfi * (motor->m_cc_was_hfi ? 1.8 : 1.5));

	bool hfi_est_done = motor->m_hfi.est_done_cnt >= conf_now->foc_hfi_start_samples;

	// Only allow Q axis current after the HFI ambiguity is resolved. This causes
	// a short delay when starting.
	if (do_hfi && !hfi_est_done) {
		state_m->iq_target = 0;
	} else if (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_START) {
		do_hfi = false;
	}

	motor->m_cc_was_hfi = do_hfi;

	float max_duty = fabsf(state_m->max_duty);
	utils_truncate_number(&max_duty, 0.0, conf_now->l_max_duty);

	// Park transform: transforms the currents from stator to the rotor reference frame
	state_m->id = c * state_m->i_alpha + s * state_m->i_beta;
	state_m->iq = c * state_m->i_beta  - s * state_m->i_alpha;

	// Low passed currents are used for less time critical parts, not for the feedback
	UTILS_LP_FAST(state_m->id_filter, state_m->id, conf_now->foc_current_filter_const);
	UTILS_LP_FAST(state_m->iq_filter, state_m->iq, conf_now->foc_current_filter_const);

	float d_gain_scale = 1.0;
	if (conf_now->foc_d_gain_scale_start < 0.99) {
		float max_mod_norm = fabsf(state_m->duty_now / max_duty);
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

	float Ierr_d = state_m->id_target - state_m->id;
	float Ierr_q = state_m->iq_target - state_m->iq;

	float ki = conf_now->foc_current_ki;
	if (conf_now->foc_temp_comp) {
		ki = motor->m_current_ki_temp_comp;
	}

	state_m->vd_int += Ierr_d * (ki * d_gain_scale * dt);
	state_m->vq_int += Ierr_q * (ki * dt);

	// Feedback (PI controller). No D action needed because the plant is a first order system (tf = 1/(Ls+R))
	state_m->vd = state_m->vd_int + Ierr_d * conf_now->foc_current_kp * d_gain_scale;
	state_m->vq = state_m->vq_int + Ierr_q * conf_now->foc_current_kp;

	// Decoupling. Using feedforward this compensates for the fact that the equations of a PMSM
	// are not really decoupled (the d axis current has impact on q axis voltage and visa-versa):
	//      Resistance  Inductance   Cross terms   Back-EMF   (see www.mathworks.com/help/physmod/sps/ref/pmsm.html)
	// vd = Rs*id   +   Ld*did/dt −  ωe*iq*Lq
	// vq = Rs*iq   +   Lq*diq/dt +  ωe*id*Ld     + ωe*ψm
	float dec_vd = 0.0;
	float dec_vq = 0.0;
	float dec_bemf = 0.0;

	if (motor->m_control_mode < CONTROL_MODE_HANDBRAKE && conf_now->foc_cc_decoupling != FOC_CC_DECOUPLING_DISABLED) {
		switch (conf_now->foc_cc_decoupling) {
		case FOC_CC_DECOUPLING_CROSS:
			dec_vd = state_m->iq_filter * motor->m_speed_est_fast * motor->p_lq; // m_speed_est_fast is ωe in [rad/s]
			dec_vq = state_m->id_filter * motor->m_speed_est_fast * motor->p_ld;
			break;

		case FOC_CC_DECOUPLING_BEMF:
			dec_bemf = motor->m_speed_est_fast * conf_now->foc_motor_flux_linkage;
			break;

		case FOC_CC_DECOUPLING_CROSS_BEMF:
			dec_vd = state_m->iq_filter * motor->m_speed_est_fast * motor->p_lq;
			dec_vq = state_m->id_filter * motor->m_speed_est_fast * motor->p_ld;
			dec_bemf = motor->m_speed_est_fast * conf_now->foc_motor_flux_linkage;
			break;

		default:
			break;
		}
	}

	state_m->vd -= dec_vd; //Negative sign as in the PMSM equations
	state_m->vq += dec_vq + dec_bemf;

	// Calculate the max length of the voltage space vector without overmodulation.
	// Is simply 1/sqrt(3) * v_bus. See https://microchipdeveloper.com/mct5001:start. Adds margin with max_duty.
	float max_v_mag = ONE_BY_SQRT3 * max_duty * state_m->v_bus;

	// Saturation and anti-windup. Notice that the d-axis has priority as it controls field
	// weakening and the efficiency.
	float vd_presat = state_m->vd;
	utils_truncate_number_abs((float*)&state_m->vd, max_v_mag);
	state_m->vd_int += (state_m->vd - vd_presat);

	float max_vq = sqrtf(SQ(max_v_mag) - SQ(state_m->vd));
	float vq_presat = state_m->vq;
	utils_truncate_number_abs((float*)&state_m->vq, max_vq);
	state_m->vq_int += (state_m->vq - vq_presat);

	utils_saturate_vector_2d((float*)&state_m->vd, (float*)&state_m->vq, max_v_mag);

	// mod_d and mod_q are normalized such that 1 corresponds to the max possible voltage:
	//    voltage_normalize = 1/(2/3*V_bus)
	// This includes overmodulation and therefore cannot be made in any direction.
	// Note that this scaling is different from max_v_mag, which is without over modulation.
	const float voltage_normalize = 1.5 / state_m->v_bus;
	state_m->mod_d = state_m->vd * voltage_normalize;
	state_m->mod_q = state_m->vq * voltage_normalize;
	UTILS_NAN_ZERO(state_m->mod_q_filter);
	UTILS_LP_FAST(state_m->mod_q_filter, state_m->mod_q, 0.2);

	// TODO: Have a look at this?
#ifdef HW_HAS_INPUT_CURRENT_SENSOR
	state_m->i_bus = GET_INPUT_CURRENT();
#else
	state_m->i_bus = state_m->mod_alpha_measured * state_m->i_alpha + state_m->mod_beta_measured * state_m->i_beta;
	// TODO: Also calculate motor power based on v_alpha, v_beta, i_alpha and i_beta. This is much more accurate
	// with phase filters than using the modulation and bus current.
#endif
	state_m->i_abs = NORM2_f(state_m->id, state_m->iq);
	state_m->i_abs_filter = NORM2_f(state_m->id_filter, state_m->iq_filter);

	// Inverse Park transform: transforms the (normalized) voltages from the rotor reference frame to the stator frame
	state_m->mod_alpha_raw = c * state_m->mod_d - s * state_m->mod_q;
	state_m->mod_beta_raw  = c * state_m->mod_q + s * state_m->mod_d;

	update_valpha_vbeta(motor, state_m->mod_alpha_raw, state_m->mod_beta_raw);

	// Dead time compensated values for vd and vq. Note that these are not used to control the switching times.
	state_m->vd = c * motor->m_motor_state.v_alpha + s * motor->m_motor_state.v_beta;
	state_m->vq = c * motor->m_motor_state.v_beta  - s * motor->m_motor_state.v_alpha;

	// HFI
	if (do_hfi) {
#ifdef HW_HAS_DUAL_MOTORS
		if (motor == &m_motor_2) {
			CURRENT_FILTER_OFF_M2();
		} else {
			CURRENT_FILTER_OFF();
		}
#else
		CURRENT_FILTER_OFF();
#endif

		float mod_alpha_v7 = state_m->mod_alpha_raw;
		float mod_beta_v7 = state_m->mod_beta_raw;

#ifdef HW_HAS_PHASE_SHUNTS
		float mod_alpha_v0 = state_m->mod_alpha_raw;
		float mod_beta_v0 = state_m->mod_beta_raw;
#endif

		float hfi_voltage;
		if (motor->m_hfi.est_done_cnt < conf_now->foc_hfi_start_samples) {
			hfi_voltage = conf_now->foc_hfi_voltage_start;
		} else {
			hfi_voltage = utils_map(fabsf(state_m->iq), -0.01, conf_now->l_current_max,
					conf_now->foc_hfi_voltage_run, conf_now->foc_hfi_voltage_max);
		}

		utils_truncate_number_abs(&hfi_voltage, state_m->v_bus * (1.0 - fabsf(state_m->duty_now)) * SQRT3_BY_2 * (2.0 / 3.0) * 0.95);

		if ((conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V5) && hfi_est_done) {
			if (motor->m_hfi.is_samp_n) {
				float sample_now = c * motor->m_i_beta_sample_with_offset - s * motor->m_i_alpha_sample_with_offset;
				float di = (motor->m_hfi.prev_sample - sample_now);

				if (!motor->m_using_encoder) {
					motor->m_hfi.double_integrator = -motor->m_speed_est_fast;
					motor->m_hfi.angle = motor->m_phase_now_observer;
				} else {
					float hfi_dt = dt * 2.0;
#ifdef HW_HAS_PHASE_SHUNTS
					if (!conf_now->foc_sample_v0_v7 && conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4) {
						hfi_dt = dt;
					}
#endif
					foc_hfi_adjust_angle(
							(di * conf_now->foc_f_zv) / (hfi_voltage * motor->p_inv_ld_lq),
							motor, hfi_dt
					);
				}

#ifdef HW_HAS_PHASE_SHUNTS
				if (conf_now->foc_sample_v0_v7 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V5) {
					mod_alpha_v7 -= hfi_voltage * c * voltage_normalize;
					mod_beta_v7 -= hfi_voltage * s * voltage_normalize;
				} else {
					motor->m_hfi.prev_sample = c * motor->m_i_beta_sample_next - s * motor->m_i_alpha_sample_next;

					mod_alpha_v0 -= hfi_voltage * c * voltage_normalize;
					mod_beta_v0 -= hfi_voltage * s * voltage_normalize;

					mod_alpha_v7 += hfi_voltage * c * voltage_normalize;
					mod_beta_v7 += hfi_voltage * s * voltage_normalize;

					motor->m_hfi.is_samp_n = !motor->m_hfi.is_samp_n;
					motor->m_i_alpha_beta_has_offset = true;
				}
#else
				mod_alpha_v7 -= hfi_voltage * c * voltage_normalize;
				mod_beta_v7 -= hfi_voltage * s * voltage_normalize;
#endif
			} else {
				motor->m_hfi.prev_sample = c * state_m->i_beta - s * state_m->i_alpha;
				mod_alpha_v7 += hfi_voltage * c * voltage_normalize;
				mod_beta_v7  += hfi_voltage * s * voltage_normalize;
			}
		} else if ((conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V3) && hfi_est_done) {
			if (motor->m_hfi.is_samp_n) {
				if (fabsf(state_m->iq_target) > conf_now->foc_hfi_hyst) {
					motor->m_hfi.sign_last_sample = SIGN(state_m->iq_target);
				}

				float sample_now = motor->m_hfi.cos_last * motor->m_i_alpha_sample_with_offset +
						motor->m_hfi.sin_last * motor->m_i_beta_sample_with_offset;
				float di = (sample_now - motor->m_hfi.prev_sample);

				if (!motor->m_using_encoder) {
					motor->m_hfi.double_integrator = -motor->m_speed_est_fast;
					motor->m_hfi.angle = motor->m_phase_now_observer;
				} else {
					float hfi_dt = dt * 2.0;
#ifdef HW_HAS_PHASE_SHUNTS
					if (!conf_now->foc_sample_v0_v7 && conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2) {
						hfi_dt = dt;
					}
#endif
					foc_hfi_adjust_angle(
							motor->m_hfi.sign_last_sample * ((conf_now->foc_f_zv * di) /
									hfi_voltage - motor->p_v2_v3_inv_avg_half) / motor->p_inv_ld_lq,
							motor, hfi_dt
					);
				}

				// Use precomputed rotation matrix
				if (motor->m_hfi.sign_last_sample > 0) {
					// +45 Degrees
					motor->m_hfi.sin_last = ONE_BY_SQRT2 * (c + s);
					motor->m_hfi.cos_last = ONE_BY_SQRT2 * (c - s);
				} else {
					// -45 Degrees
					motor->m_hfi.sin_last = ONE_BY_SQRT2 * (-c + s);
					motor->m_hfi.cos_last = ONE_BY_SQRT2 * (c + s);
				}

#ifdef HW_HAS_PHASE_SHUNTS
				if (conf_now->foc_sample_v0_v7 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V3) {
					mod_alpha_v7 += hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
					mod_beta_v7 += hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;
				} else {
					motor->m_hfi.prev_sample = motor->m_hfi.cos_last * motor->m_i_alpha_sample_next +
							motor->m_hfi.sin_last * motor->m_i_beta_sample_next;

					mod_alpha_v0 += hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
					mod_beta_v0 += hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;

					mod_alpha_v7 -= hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
					mod_beta_v7 -= hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;

					motor->m_hfi.is_samp_n = !motor->m_hfi.is_samp_n;
					motor->m_i_alpha_beta_has_offset = true;
				}
#else
				mod_alpha_v7 += hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
				mod_beta_v7 += hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;
#endif
			} else {
				motor->m_hfi.prev_sample = motor->m_hfi.cos_last * state_m->i_alpha + motor->m_hfi.sin_last * state_m->i_beta;
				mod_alpha_v7 -= hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
				mod_beta_v7  -= hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;
			}
		} else {
			if (motor->m_hfi.is_samp_n) {
				float sample_now = (utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_alpha -
						utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_beta);
				float di = (sample_now - motor->m_hfi.prev_sample);

				motor->m_hfi.buffer_current[motor->m_hfi.ind] = di;

				if (di > 0.01) {
					motor->m_hfi.buffer[motor->m_hfi.ind] = hfi_voltage / (conf_now->foc_f_zv * di);
				}

				motor->m_hfi.ind++;
				if (motor->m_hfi.ind == motor->m_hfi.samples) {
					motor->m_hfi.ind = 0;
					motor->m_hfi.ready = true;
				}

				mod_alpha_v7 += hfi_voltage * utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
				mod_beta_v7  -= hfi_voltage * utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
			} else {
				motor->m_hfi.prev_sample = utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_alpha -
						utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_beta;

				mod_alpha_v7 -= hfi_voltage * utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
				mod_beta_v7  += hfi_voltage * utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
			}
		}

		utils_saturate_vector_2d(&mod_alpha_v7, &mod_beta_v7, SQRT3_BY_2 * 0.95);
		motor->m_hfi.is_samp_n = !motor->m_hfi.is_samp_n;

		if (conf_now->foc_sample_v0_v7) {
			state_m->mod_alpha_raw = mod_alpha_v7;
			state_m->mod_beta_raw = mod_beta_v7;
		} else {
#ifdef HW_HAS_PHASE_SHUNTS
			if (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4) {
				utils_saturate_vector_2d(&mod_alpha_v0, &mod_beta_v0, SQRT3_BY_2 * 0.95);
				state_m->mod_alpha_raw = mod_alpha_v0;
				state_m->mod_beta_raw = mod_beta_v0;
			}
#endif

			// Delay adding the HFI voltage when not sampling in both 0 vectors, as it will cancel
			// itself with the opposite pulse from the previous HFI sample. This makes more sense
			// when drawing the SVM waveform.
			foc_svm(mod_alpha_v7, mod_beta_v7, TIM1->ARR,
				(uint32_t*)&motor->m_duty1_next,
				(uint32_t*)&motor->m_duty2_next,
				(uint32_t*)&motor->m_duty3_next,
				(uint32_t*)&state_m->svm_sector);
			motor->m_duty_next_set = true;
		}
	} else {
#ifdef HW_HAS_DUAL_MOTORS
		if (motor == &m_motor_2) {
			CURRENT_FILTER_ON_M2();
		} else {
			CURRENT_FILTER_ON();
		}
#else
		CURRENT_FILTER_ON();
#endif
		motor->m_hfi.ind = 0;
		motor->m_hfi.ready = false;
		motor->m_hfi.is_samp_n = false;
		motor->m_hfi.prev_sample = 0.0;
		motor->m_hfi.double_integrator = 0.0;
	}

	// Set output (HW Dependent)
	uint32_t duty1, duty2, duty3, top;
	top = TIM1->ARR;

	// Calculate the duty cycles for all the phases. This also injects a zero modulation signal to
	// be able to fully utilize the bus voltage. See https://microchipdeveloper.com/mct5001:start
	foc_svm(state_m->mod_alpha_raw, state_m->mod_beta_raw, top, &duty1, &duty2, &duty3, (uint32_t*)&state_m->svm_sector);

	if (motor == &m_motor_1) {
		TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3);
#ifdef HW_HAS_DUAL_PARALLEL
		TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3);
#endif
	} else {
#ifndef HW_HAS_DUAL_PARALLEL
		TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3);
#endif
	}

	// do not allow to turn on PWM outputs if virtual motor is used
	if(virtual_motor_is_connected() == false) {
		if (!motor->m_output_on) {
			start_pwm_hw(motor);
		}
	}
}