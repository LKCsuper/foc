/*
 * @Description: 
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-03-04 17:45:55
 * @FilePath: \foc\App\motor\inc\utils.h
 */
#ifndef _MOTOR_PARAM_H
#define _MOTOR_PARAM_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

typedef struct {
	float va;
	float vb;
	float vc;
	float v_mag_filter;
	float mod_alpha_filter;
	float mod_beta_filter;
	float mod_alpha_measured;
	float mod_beta_measured;
	float mod_alpha_raw;
	float mod_beta_raw;
	float id_target;
	float iq_target;
	float max_duty;
	float duty_now;
	float phase;
	float phase_cos;
	float phase_sin;
	float i_alpha;
	float i_beta;
	float i_abs;
	float i_abs_filter;
	float i_bus;
	float v_bus;
	float v_alpha;
	float v_beta;
	float mod_d;
	float mod_q;
	float mod_q_filter;
	float id;
	float iq;
	float id_filter;
	float iq_filter;
	float vd;
	float vq;
	float vd_int;
	float vq_int;
	float speed_rad_s;
	uint32_t svm_sector;
	bool is_using_phase_filters;
} motor_state_t;

typedef struct {
	float x1;
	float x2;
	float lambda_est;
	float i_alpha_last;
	float i_beta_last;
} observer_state;

typedef struct {
	int sample_num;
	float avg_current_tot;
	float avg_voltage_tot;
} mc_sample_t;

typedef struct {
	void(*fft_bin0_func)(float*, float*, float*);
	void(*fft_bin1_func)(float*, float*, float*);
	void(*fft_bin2_func)(float*, float*, float*);

	int samples;
	int table_fact;
	float buffer[32];
	float buffer_current[32];
	bool ready;
	int ind;
	bool is_samp_n;
	float sign_last_sample;
	float cos_last, sin_last;
	float prev_sample;
	float angle;
	float double_integrator;
	int est_done_cnt;
	float observer_zero_time;
	int flip_cnt;
} hfi_state_t;

typedef struct {
	mc_configuration *m_conf;
	mc_state m_state;
	mc_control_mode m_control_mode;
	motor_state_t m_motor_state;
	float m_curr_unbalance;
	float m_currents_adc[3];
	bool m_phase_override;
	float m_phase_now_override;
	float m_duty_cycle_set;
	float m_id_set;
	float m_iq_set;
	float m_i_fw_set;
	float m_current_off_delay;
	float m_openloop_speed;
	float m_openloop_phase;
	bool m_output_on;
	float m_pos_pid_set;
	float m_speed_pid_set_rpm;
	float m_speed_command_rpm;
	float m_phase_now_observer;
	float m_phase_now_observer_override;
	float m_observer_x1_override;
	float m_observer_x2_override;
	bool m_phase_observer_override;
	float m_phase_now_encoder;
	float m_phase_now_encoder_no_index;
	observer_state m_observer_state;
	float m_pll_phase;
	float m_pll_speed;
	mc_sample_t m_samples;
	int m_tachometer;
	int m_tachometer_abs;
	float m_pos_pid_now;
	float m_gamma_now;
	bool m_using_encoder;
	float m_speed_est_fast;
	float m_speed_est_faster;
	int m_duty1_next, m_duty2_next, m_duty3_next;
	bool m_duty_next_set;
	float m_i_alpha_sample_next;
	float m_i_beta_sample_next;
	float m_i_alpha_sample_with_offset;
	float m_i_beta_sample_with_offset;
	float m_i_alpha_beta_has_offset;
	hfi_state_t m_hfi;
	int m_hfi_plot_en;
	float m_hfi_plot_sample;

	// For braking
	float m_br_speed_before;
	float m_br_vq_before;
	int m_br_no_duty_samples;

	float m_duty_abs_filtered;
	float m_duty_filtered;
	bool m_was_control_duty;
	float m_duty_i_term;
	float m_openloop_angle;
	float m_x1_prev;
	float m_x2_prev;
	float m_phase_before_speed_est;
	int m_tacho_step_last;
	float m_pid_div_angle_last;
	float m_pid_div_angle_accumulator;
	float m_min_rpm_hyst_timer;
	float m_min_rpm_timer;
	bool m_cc_was_hfi;
	float m_pos_i_term;
	float m_pos_prev_error;
	float m_pos_dt_int;
	float m_pos_prev_proc;
	float m_pos_dt_int_proc;
	float m_pos_d_filter;
	float m_pos_d_filter_proc;
	float m_speed_i_term;
	float m_speed_prev_error;
	float m_speed_d_filter;
	int m_ang_hall_int_prev;
	bool m_using_hall;
	float m_ang_hall;
	float m_ang_hall_rate_limited;
	float m_hall_dt_diff_last;
	float m_hall_dt_diff_now;
	bool m_motor_released;

	// Resistance observer
	float m_r_est;
	float m_r_est_state;

	// Temperature-compensated parameters
	float m_res_temp_comp;
	float m_current_ki_temp_comp;

	// Pre-calculated values
	float p_lq;
	float p_ld;
	float p_inv_ld_lq; // (1.0/lq - 1.0/ld)
	float p_v2_v3_inv_avg_half; // (0.5/ld + 0.5/lq)
} motor_all_state_t;

typedef struct motor_param
{
	float ia;
	float ib;
	float ic;
    
}MOTOR_PARAM_S;

/* Private variables ---------------------------------------------------------*/
extern motor_all_state_t motor1;
extern motor_all_state_t motor2;

extern MOTOR_PARAM_S motor;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
