/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-02 22:16:11
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-02 22:28:31
 * @detail: 
 * 
 * 使用
 * foc_correct_hall(motor_now->m_phase_now_observer, dt, motor_now, utils_read_hall(motor_now != &m_motor_1, conf_now->m_hall_extra_samples));
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
float foc_correct_hall(float angle, float dt, motor_all_state_t *motor, int hall_val) 
{
	mc_configuration *conf_now = motor->m_conf;
	motor->m_hall_dt_diff_now += dt;

	float rad_per_sec = (M_PI / 3.0) / motor->m_hall_dt_diff_last;
	float rpm_abs_fast = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));
	float rpm_abs_hall = fabsf(RADPS2RPM_f(rad_per_sec));

	// Hysteresis 5 % of total speed
	float hyst = conf_now->foc_sl_erpm * 0.1;
	if (motor->m_using_hall) {
		if (fminf(rpm_abs_fast, rpm_abs_hall) > (conf_now->foc_sl_erpm + hyst)) {
			motor->m_using_hall = false;
		}
	} else {
		if (rpm_abs_fast < (conf_now->foc_sl_erpm - hyst)) {
			motor->m_using_hall = true;
		}
	}

	int ang_hall_int = conf_now->foc_hall_table[hall_val];

	// Only override the observer if the hall sensor value is valid.
	if (ang_hall_int < 201) {
		// Scale to the circle and convert to radians
		float ang_hall_now = ((float)ang_hall_int / 200.0) * 2 * M_PI;

		if (motor->m_ang_hall_int_prev < 0) {
			// Previous angle not valid
			motor->m_ang_hall_int_prev = ang_hall_int;
			motor->m_ang_hall = ang_hall_now;
		} else if (ang_hall_int != motor->m_ang_hall_int_prev) {
			int diff = ang_hall_int - motor->m_ang_hall_int_prev;
			if (diff > 100) {
				diff -= 200;
			} else if (diff < -100) {
				diff += 200;
			}

			// This is only valid if the direction did not just change. If it did, we use the
			// last speed together with the sign right now.
			if (SIGN(diff) == SIGN(motor->m_hall_dt_diff_last)) {
				if (diff > 0) {
					motor->m_hall_dt_diff_last = motor->m_hall_dt_diff_now;
				} else {
					motor->m_hall_dt_diff_last = -motor->m_hall_dt_diff_now;
				}
			} else {
				motor->m_hall_dt_diff_last = -motor->m_hall_dt_diff_last;
			}

			motor->m_hall_dt_diff_now = 0.0;

			// A transition was just made. The angle is in the middle of the new and old angle.
			int ang_avg = motor->m_ang_hall_int_prev + diff / 2;
			ang_avg %= 200;

			// Scale to the circle and convert to radians
			motor->m_ang_hall = ((float)ang_avg / 200.0) * 2 * M_PI;
		}

		motor->m_ang_hall_int_prev = ang_hall_int;

		if (RADPS2RPM_f((M_PI / 3.0) /
				fmaxf(fabsf(motor->m_hall_dt_diff_now),
						fabsf(motor->m_hall_dt_diff_last))) < conf_now->foc_hall_interp_erpm) {
			// Don't interpolate on very low speed, just use the closest hall sensor. The reason is that we might
			// get stuck at 60 degrees off if a direction change happens between two steps.
			motor->m_ang_hall = ang_hall_now;
		} else {
			// Interpolate
			float diff = utils_angle_difference_rad(motor->m_ang_hall, ang_hall_now);
			if (fabsf(diff) < ((2.0 * M_PI) / 12.0)) {
				// Do interpolation
				motor->m_ang_hall += rad_per_sec * dt;
			} else {
				// We are too far away with the interpolation
				motor->m_ang_hall -= diff / 100.0;
			}
		}

		// Limit hall sensor rate of change. This will reduce current spikes in the current controllers when the angle estimation
		// changes fast.
		float angle_step = (fmaxf(rpm_abs_hall, conf_now->foc_hall_interp_erpm) / 60.0) * 2.0 * M_PI * dt * 1.5;
		float angle_diff = utils_angle_difference_rad(motor->m_ang_hall, motor->m_ang_hall_rate_limited);
		if (fabsf(angle_diff) < angle_step) {
			motor->m_ang_hall_rate_limited = motor->m_ang_hall;
		} else {
			motor->m_ang_hall_rate_limited += angle_step * SIGN(angle_diff);
		}

		utils_norm_angle_rad((float*)&motor->m_ang_hall_rate_limited);
		utils_norm_angle_rad((float*)&motor->m_ang_hall);

		if (motor->m_using_hall) {
			angle = motor->m_ang_hall_rate_limited;
		}
	} else {
		// Invalid hall reading. Don't update angle.
		motor->m_ang_hall_int_prev = -1;

		// Also allow open loop in order to behave like normal sensorless
		// operation. Then the motor works even if the hall sensor cable
		// gets disconnected (when the sensor spacing is 120 degrees).
		if (motor->m_phase_observer_override && motor->m_state == MC_STATE_RUNNING) {
			angle = motor->m_phase_now_observer_override;
		}
	}

	return angle;
}

/**
 * Read hall sensors
 * 读取霍尔传感器
 * @param is_second_motor
 * Use hall sensor port for second motor on dual motor hardware.
 *
 * @param samples
 * is_second_motor 是否是二电机
 * samples MCCONF_M_HALL_EXTRA_SAMPLES 采样过滤， 默认采样一次
 * The number of extra samples to read and filter over. If this
 * is 0, only one sample will be used.
 *
 * @return
 * 三个霍尔传感器的状态
 * The state of the three hall sensors.
 */
int utils_read_hall(bool is_second_motor, int samples) {
	samples = 1 + 2 * samples;

	int h1 = 0, h2 = 0, h3 = 0;
	int tres = samples / 2;

	if (is_second_motor) {
		while (samples--) {
			h1 += READ_HALL1_2();
			h2 += READ_HALL2_2();
			h3 += READ_HALL3_2();
		}
	} else {
		while (samples--) {
			h1 += READ_HALL1();
			h2 += READ_HALL2();
			h3 += READ_HALL3();
		}
	}

	return (h1 > tres) | ((h2 > tres) << 1) | ((h3 > tres) << 2);
}

/**
 * 检测霍尔
 * Run the motor in open loop and figure out at which angles the hall sensors are.
 *
 * @param current
 * Current to use.
 *
 * @param hall_table
 * Table to store the result to.
 *
 * @result
 * true: Success
 * false: Something went wrong
 *
 * @return
 * The fault code
 */
int mcpwm_foc_hall_detect(float current, uint8_t *hall_table, bool *result) {
	volatile motor_all_state_t *motor = get_motor_now();
	int fault = FAULT_CODE_NONE;
	mc_interface_lock();

	motor->m_phase_override = true;
	motor->m_id_set = 0.0;
	motor->m_iq_set = 0.0;
	motor->m_control_mode = CONTROL_MODE_CURRENT;
	motor->m_motor_released = false;
	motor->m_state = MC_STATE_RUNNING;

	// MTPA overrides id target
	MTPA_MODE mtpa_old = motor->m_conf->foc_mtpa_mode;
	motor->m_conf->foc_mtpa_mode = MTPA_MODE_OFF;

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	// Lock the motor
	motor->m_phase_now_override = 0;

	*result = false;

	for (int i = 0;i < 1000;i++) {
		motor->m_id_set = (float)i * current / 1000.0;
		fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			goto exit_hall_detect;
		}
		chThdSleepMilliseconds(1);
	}

	float sin_hall[8];
	float cos_hall[8];
	int hall_iterations[8];
	memset(sin_hall, 0, sizeof(sin_hall));
	memset(cos_hall, 0, sizeof(cos_hall));
	memset(hall_iterations, 0, sizeof(hall_iterations));

	// Forwards
	for (int i = 0;i < 3;i++) {
		for (int j = 0;j < 360;j++) {
			motor->m_phase_now_override = DEG2RAD_f(j);
			fault = mc_interface_get_fault();
			if (fault != FAULT_CODE_NONE) {
				goto exit_hall_detect;
			}
			chThdSleepMilliseconds(5);

			int hall = utils_read_hall(motor != &m_motor_1, motor->m_conf->m_hall_extra_samples);
			float s, c;
			sincosf(motor->m_phase_now_override, &s, &c);
			sin_hall[hall] += s;
			cos_hall[hall] += c;
			hall_iterations[hall]++;
		}
	}

	// Reverse
	for (int i = 0;i < 3;i++) {
		for (int j = 360;j >= 0;j--) {
			motor->m_phase_now_override = DEG2RAD_f(j);
			fault = mc_interface_get_fault();
			if (fault != FAULT_CODE_NONE) {
				goto exit_hall_detect;
			}
			chThdSleepMilliseconds(5);

			int hall = utils_read_hall(motor != &m_motor_1, motor->m_conf->m_hall_extra_samples);
			float s, c;
			sincosf(motor->m_phase_now_override, &s, &c);
			sin_hall[hall] += s;
			cos_hall[hall] += c;
			hall_iterations[hall]++;
		}
	}

	int fails = 0;
	for(int i = 0;i < 8;i++) {
		if (hall_iterations[i] > 30) {
			float ang = RAD2DEG_f(atan2f(sin_hall[i], cos_hall[i]));
			utils_norm_angle(&ang);
			hall_table[i] = (uint8_t)(ang * 200.0 / 360.0);
		} else {
			hall_table[i] = 255;
			fails++;
		}
	}
	*result = (fails == 2);

	exit_hall_detect:
	motor->m_id_set = 0.0;
	motor->m_iq_set = 0.0;
	motor->m_phase_override = false;
	motor->m_control_mode = CONTROL_MODE_NONE;
	motor->m_state = MC_STATE_OFF;
	stop_pwm_hw((motor_all_state_t*)motor);
	motor->m_conf->foc_mtpa_mode = mtpa_old;
	timeout_configure(tout, tout_c, tout_ksw);
	mc_interface_unlock();

	return fault;
}
#ifdef __cplusplus
}
#endif