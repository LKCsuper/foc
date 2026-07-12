/*
 * @Description: 
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-10-09 23:00:11
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
//#pragma pack (1)
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
	bool m_phase_override;					/* 这个为true就是纯角度 */
	float m_phase_now_override;				/* 这个目前看来就是纯角度控制 */
	float m_duty_cycle_set;
	float m_id_set;							/* 电流环 设置d轴值 */
	float m_iq_set;							/* 电流环 设置q轴值*/
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

typedef struct
{
  real32_T P_Gain;
  real32_T I_Gain;
  real32_T D_Gain;
  real32_T B_Gain;
  real32_T Max_Output;
  real32_T Min_Output;
  real32_T I_Sum;
  real32_T Cal_Out;
  real32_T Err;				/* 差值 */
}CURRENT_PID_DEF;

typedef struct motor_param
{
	float currentAdc[3];
	float voltAdc[3];

	float ia;						/* 检测到的电流a */
	float ib;						/* 检测到的电流b */
	float ic;						/* 检测到的电流c */

	float va;						/* 检测到的电压a */
	float vb;                       /* 检测到的电压b */
	float vc;                       /* 检测到的电压c */

	float v_bus;					/* 总线电压 */

	float Ialpha;
	float Ibeta;

	float Id;						/* park 变换 */
	float Iq;						/* park 变换 */

	float theta;					/* 角度 */

	/* 电流环,设置 */
	float e_id_set;
	float e_iq_set;
	float e_err_ki_sum;				/* 积分累加 */
    float e_ki_windup;				/* 积分抗饱和 */
	float e_iq_ki;					/* q轴电流环ki */
	float e_iq_kp;					/* q轴电流环kp */
    float e_id_ki;					/* d轴电流环ki */
	float e_id_kp;					/* d轴电流环kp */
    float e_iq_limit;                /* q轴限制 */
    float e_id_limit;                /* d轴限制 */
	
	CURRENT_PID_DEF e_iq_pid;

	float e_iq_out;					/* q轴实际输出 */
	float e_id_out;					/* d轴实际输出 */

	float modD;
	float modQ;

	float modAlpha;
	float modBeta;

	ULONG duty1;
	ULONG duty2;
	ULONG duty3;

	uint8_t curHallABC;
	uint8_t lastHallABC;
	uint8_t hallABC[6];

	/* conf */
	float currentOffset[3];			/* 电流偏移 */
	float voltOffset[3];			/* 电压偏移 */
	float voltundrivenOffset[3]; 	/* 电压无负载偏移 */

	/* 开环控制：设 ol_target_rpm 即可，其余自动计算 */
	bool ol_enable;					/* open-loop enable */
#define MOTOR_OL_MODE_VF   (0)  /* V/F 电压开环 */
#define MOTOR_OL_MODE_IF   (1)  /* 角度开环 + 电流闭环 (I/F) */
	int  ol_mode;					/* MOTOR_OL_MODE_VF 或 MOTOR_OL_MODE_IF */
	float ol_target_rpm;			/* 目标转速 RPM */
	float ol_max_rpm;				/* max target speed RPM */
	float ol_current_rpm;			/* 当前斜坡转速 RPM */
	float ol_v_boost;				/* V/F 启动补偿 V（默认 2V） */
	float ol_kv_f;					/* V/F 斜率 V/Hz */
	float ol_ramp_rpm_s;			/* 软启动速率 RPM/s */
	float ol_vq_limit;				/* Vq 上限 V */
	float ol_theta;					/* 开环电角度 rad */
	float ol_foc_freq;				/* ADC-DMA 基准频率 Hz（默认 30000，实际 FOC 还要除以分频系数） */
	float ol_pole_pairs;			/* 电机极对数 */
	float ol_min_rpm;				/* 最低转速地板 RPM */
	/* 开环电流模式（ol_mode=1）：角度开环 + PI 电流闭环，参考 VESC */
	float ol_iq_set;				/* 目标 Iq 电流 A（电流模式） */
	float ol_id_set;				/* 目标 Id 电流 A（通常 0） */
	float ol_current_kp;			/* 开环电流环 Kp */
	float ol_current_ki;			/* 开环电流环 Ki */
	float ol_vd_int;				/* d轴积分项 */
	float ol_vq_int;				/* q轴积分项 */
	float ol_start_phase_boost;		/* 兼容旧启动方式的一拍相位补偿 rad；启用预对齐时不使用 */
	/*
	 * 无位置传感器起步预对齐参数。
	 * 启动时先把定子磁场固定在 ol_align_angle，并施加 ol_align_current 的 Id
	 * 一段时间，让静止转子被吸到已知电角位置；之后才开始 I/F 旋转磁场。
	 */
	bool  ol_align_enable;			/* true=每次 I/F 启动前执行预对齐 */
	float ol_align_current;			/* 对齐 D 轴电流 Id，单位 A，建议从 0.3~0.5A 起调 */
	float ol_align_time;				/* 固定磁场保持时间，单位 s，建议 0.2~0.5s */
	float ol_align_angle;				/* 固定对齐电角度，单位 rad；默认 0 */

	/* 速度外环：霍尔机械转速 -> PI -> Iq 目标；默认关闭，调试确认后再开启 */
	bool speed_enable;				/* true=速度 PI 接管 Iq，false=继续使用 ol_iq_set */
	bool speed_allow_braking;		/* true=允许反向 Iq 主动制动，false=超速时仅撤掉转矩 */
	float speed_kp;					/* 速度环比例增益，单位 A/RPM */
	float speed_ki;					/* 速度环积分增益，单位 A/(RPM*s) */
	float speed_iq_limit;			/* 速度环输出的 |Iq| 上限，单位 A */
	float speed_loop_hz;				/* 速度环执行频率，单位 Hz，推荐 500~1000 */

	/* 无感 Ortega 磁链观测器与开环切闭环参数，全部使用 SI 单位 */
	bool sensorless_enable;			/* true=后台运行观测器；是否接管由 sensorless_takeover 决定 */
	bool sensorless_takeover;		/* true=允许观测器满足锁定条件后真正接管角度 */
	float sensorless_motor_r;		/* 相电阻 Ω */
	float sensorless_motor_l;		/* 相电感 H */
	float sensorless_flux_linkage;	/* 永磁体磁链 Wb */
	float sensorless_observer_gain;	/* Ortega 观测器增益 */
	float sensorless_pll_kp;		/* 角度 PLL 比例增益 */
	float sensorless_pll_ki;		/* 角度 PLL 积分增益 */
	float sensorless_transition_rpm;/* 高于该机械转速才尝试无感接管 */
	float sensorless_fallback_rpm;	/* 低于该机械转速持续一段时间后回退 I/F */
	float sensorless_openloop_max_rpm;/* 未闭环时 I/F 允许强拖的最高机械转速 */
	float sensorless_lock_time;		/* 锁定条件持续时间 s */
	float sensorless_loss_time;		/* 失锁条件持续时间 s */
	float sensorless_blend_time;		/* 开环角度渐变到观测角度的时间 s */
	float sensorless_phase_tolerance;/* 接管前允许的最大相位误差 rad */
	float sensorless_phase_offset;	/* 观测角度固定补偿 rad */
	float sensorless_speed_filter;	/* PLL 机械转速低通系数 0~1 */

}MOTOR_PARAM_S;
//#pragma pack ()

/*
参数分为几种呢：当然下边的任何保存，都是全部保存
(1) 掉电保存
(2) 修改完立刻保存
(3) 定时保存
*/
typedef struct tagFlashParam
{
    MOTOR_PARAM_S motor;
}FLASH_PARAM_S;
/* Private variables ---------------------------------------------------------*/
extern motor_all_state_t m_motor_1;
extern motor_all_state_t m_motor_2;
extern MOTOR_PARAM_S motor;
extern mc_configuration motorCfg;

extern motor_all_state_t *gpstMotor1;
extern motor_all_state_t *gpstMotor2;
extern MOTOR_PARAM_S *gpstMotor;
extern mc_configuration *gpstMotorCfg;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
VOID MOTOR_Param_Init(VOID);
#ifdef __cplusplus
}
#endif

#endif
