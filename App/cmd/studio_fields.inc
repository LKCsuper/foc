FIELD(ol_max_rpm, 0, 1), /* 0 */
FIELD(ol_v_boost, 0, 1), /* 1 */
FIELD(ol_kv_f, 0, 1), /* 2 */
FIELD(ol_ramp_rpm_s, 0, 1), /* 3 */
FIELD(ol_vq_limit, 0, 1), /* 4 */
FIELD(ol_foc_freq, 0, 0), /* 5 */
FIELD(ol_pole_pairs, 0, 1), /* 6 */
FIELD(ol_min_rpm, 0, 1), /* 7 */
FIELD(ol_current_kp, 0, 1), /* 8 */
FIELD(ol_current_ki, 0, 1), /* 9 */
FIELD(ol_start_phase_boost, 0, 1), /* 10 */
FIELD(ol_align_enable, 1, 1), /* 11 */
FIELD(ol_align_current, 0, 1), /* 12 */
FIELD(ol_align_time, 0, 1), /* 13 */
FIELD(ol_align_angle, 0, 1), /* 14 */
FIELD(hfi_start_enable, 1, 1), /* 15 */
FIELD(hfi_start_voltage, 0, 1), /* 16 */
FIELD(hfi_start_cycles, 2, 1), /* 17 */
FIELD(hfi_start_min_confidence, 0, 1), /* 18 */
FIELD(speed_enable, 1, 1), /* 19 */
FIELD(speed_allow_braking, 1, 1), /* 20 */
FIELD(speed_kp, 0, 1), /* 21 */
FIELD(speed_ki, 0, 1), /* 22 */
FIELD(speed_iq_limit, 0, 1), /* 23 */
FIELD(speed_loop_hz, 0, 1), /* 24 */
FIELD(sensorless_enable, 1, 1), /* 25 */
FIELD(sensorless_takeover, 1, 1), /* 26 */
FIELD(sensorless_motor_r, 0, 1), /* 27 */
FIELD(sensorless_motor_l, 0, 1), /* 28 */
FIELD(sensorless_flux_linkage, 0, 1), /* 29 */
FIELD(sensorless_observer_gain, 0, 1), /* 30 */
FIELD(sensorless_pll_kp, 0, 1), /* 31 */
FIELD(sensorless_pll_ki, 0, 1), /* 32 */
FIELD(sensorless_transition_rpm, 0, 1), /* 33 */
FIELD(sensorless_fallback_rpm, 0, 1), /* 34 */
FIELD(sensorless_openloop_max_rpm, 0, 1), /* 35 */
FIELD(sensorless_lock_time, 0, 1), /* 36 */
FIELD(sensorless_loss_time, 0, 1), /* 37 */
FIELD(sensorless_blend_time, 0, 1), /* 38 */
FIELD(sensorless_phase_tolerance, 0, 1), /* 39 */
FIELD(sensorless_phase_offset, 0, 1), /* 40 */
FIELD(sensorless_speed_filter, 0, 1), /* 41 */
FIELD(hall_extra_samples, 3, 1), /* 42 */
CAL(currentOffset, 0, 0, sizeof(float)),
CAL(currentOffset, 1, 0, sizeof(float)),
CAL(currentOffset, 2, 0, sizeof(float)),
CAL(voltOffset, 0, 0, sizeof(float)),
CAL(voltOffset, 1, 0, sizeof(float)),
CAL(voltOffset, 2, 0, sizeof(float)),
CAL(voltundrivenOffset, 0, 0, sizeof(float)),
CAL(voltundrivenOffset, 1, 0, sizeof(float)),
CAL(voltundrivenOffset, 2, 0, sizeof(float)),
CAL(hall_table, 0, 4, sizeof(int8_t)),
CAL(hall_table, 1, 4, sizeof(int8_t)),
CAL(hall_table, 2, 4, sizeof(int8_t)),
CAL(hall_table, 3, 4, sizeof(int8_t)),
CAL(hall_table, 4, 4, sizeof(int8_t)),
CAL(hall_table, 5, 4, sizeof(int8_t)),
CAL(hall_table, 6, 4, sizeof(int8_t)),
CAL(hall_table, 7, 4, sizeof(int8_t)),
CAL(encoder_offset_rad, 0, 0, sizeof(float)),
CAL(encoder_inverted, 0, 1, sizeof(bool)),
CAL(encoder_calibrated, 0, 1, sizeof(bool)),
