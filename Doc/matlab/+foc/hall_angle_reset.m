function st = hall_angle_reset()
%HALL_ANGLE_RESET  对齐 Motor_Hall_AngleReset。
c = foc.constants();
st.last_state = -1;
st.have_state = false;
st.theta_rad = 0.0;
st.mech_rpm = 0.0;
st.mech_rpm_raw = 0.0;
st.mech_rpm_fast = 0.0;
st.angle_valid = false;
st.speed_valid = false;
st.table_ready = false;
st.sector_timer_s = 0.0;
st.last_period_s = 0.0;
st.period_sum_s = 0.0;
st.periods_s = zeros(1, c.HALL_SPEED_WINDOW);
st.period_count = 0;
st.period_index = 0;
st.direction = 0;
st.pll_count = 0;
st.pll_pos = 0.0;
st.pll_vel = 0.0;
st.pll_low_speed_active = true;
end
