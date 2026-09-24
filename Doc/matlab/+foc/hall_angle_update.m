function st = hall_angle_update(st, table, hall_state, pole_pairs, dt_s, force_pll)
%HALL_ANGLE_UPDATE  对齐 Motor_Hall_AngleUpdate。force_pll=true 时高速仍跑 PLL。
c = foc.constants();
if nargin < 6
    force_pll = false;
end
if dt_s < 0.0
    dt_s = 0.0;
end
pp = pole_pairs;
if pp < 1.0
    pp = 1.0;
end

st.angle_valid = false;
st.table_ready = true;
theta = foc.hall_decode(table, hall_state);
if isnan(theta)
    st.speed_valid = false;
    st.mech_rpm = 0.0;
    return
end

st.theta_rad = theta;
st.angle_valid = true;
st.sector_timer_s = st.sector_timer_s + dt_s;

if ~st.have_state
    st.last_state = hall_state;
    st.have_state = true;
    st.mech_rpm = 0.0;
    st.speed_valid = true;
    st.sector_timer_s = 0.0;
    st = pll_reset(st);
    return
end

if hall_state ~= st.last_state
    theta_prev = foc.hall_decode(table, st.last_state);
    if st.sector_timer_s > 1.0e-6 && ~isnan(theta_prev)
        period_s = st.sector_timer_s;
        d_theta = foc.wrap_pi(theta - theta_prev);
        abs_d = abs(d_theta);
        if (period_s >= c.HALL_MIN_PERIOD_S) && (abs_d >= (pi / 6.0)) && (abs_d <= (pi / 2.0))
            direction = 1;
            if d_theta <= 0.0
                direction = -1;
            end
            if st.direction ~= 0 && direction ~= st.direction
                st.period_sum_s = 0.0;
                st.period_count = 0;
                st.period_index = 0;
            end
            st.direction = direction;
            st.pll_count = st.pll_count + direction;
            if st.pll_count >= 6
                st.pll_count = st.pll_count - 6;
            elseif st.pll_count < 0
                st.pll_count = st.pll_count + 6;
            end
            st.mech_rpm_raw = double(direction) * 10.0 / (pp * period_s);
            st.last_period_s = period_s;
            if st.period_count == c.HALL_SPEED_WINDOW
                st.period_sum_s = st.period_sum_s - st.periods_s(st.period_index + 1);
            else
                st.period_count = st.period_count + 1;
            end
            st.periods_s(st.period_index + 1) = period_s;
            st.period_sum_s = st.period_sum_s + period_s;
            st.period_index = mod(st.period_index + 1, c.HALL_SPEED_WINDOW);
            elec_rps = (double(st.period_count) / 6.0) / st.period_sum_s;
            st.mech_rpm = double(direction) * (elec_rps * 60.0) / pp;
            st.speed_valid = true;
        end
    end
    st.last_state = hall_state;
    st.sector_timer_s = 0.0;
    st = update_speed(st, pp, dt_s, force_pll);
    st = interpolate_theta(st, table, theta, pp);
    return
end

if st.sector_timer_s >= c.HALL_STALL_S
    st.mech_rpm = 0.0;
    st.mech_rpm_raw = 0.0;
    st.speed_valid = true;
    st.period_sum_s = 0.0;
    st.period_count = 0;
    st.period_index = 0;
    st.direction = 0;
    st = pll_reset(st);
else
    st = update_speed(st, pp, dt_s, force_pll);
end
st = interpolate_theta(st, table, theta, pp);
end

function st = pll_reset(st)
st.pll_count = 0;
st.pll_pos = 0.0;
st.pll_vel = 0.0;
st.mech_rpm_fast = 0.0;
st.pll_low_speed_active = true;
end

function x = wrap_cpr(x)
c = foc.constants();
if ~(x > -1.0e6) || ~(x < 1.0e6)
    x = 0.0;
    return
end
n = fix(x / c.HALL_PLL_CPR);
x = x - c.HALL_PLL_CPR * n;
if x < 0.0
    x = x + c.HALL_PLL_CPR;
end
if x >= c.HALL_PLL_CPR
    x = x - c.HALL_PLL_CPR;
end
end

function x = wrap_delta(x)
c = foc.constants();
half = 0.5 * c.HALL_PLL_CPR;
if ~(x > -1.0e6) || ~(x < 1.0e6)
    x = 0.0;
    return
end
n = fix(x / c.HALL_PLL_CPR);
x = x - c.HALL_PLL_CPR * n;
if x > half
    x = x - c.HALL_PLL_CPR;
elseif x <= -half
    x = x + c.HALL_PLL_CPR;
end
end

function st = pll_run(st, pole_pairs, dt_s)
c = foc.constants();
kp = 2.0 * c.HALL_PLL_BW_HZ;
ki = 0.25 * kp * kp;
if dt_s <= 0.0 || (dt_s * kp) >= 1.0
    st.pll_vel = 0.0;
    st.pll_pos = double(st.pll_count) + 0.5;
    st.mech_rpm_fast = 0.0;
    return
end
st.pll_pos = wrap_cpr(st.pll_pos + dt_s * st.pll_vel);
delta_pos = wrap_delta(double(st.pll_count) - st.pll_pos);
st.pll_pos = wrap_cpr(st.pll_pos + dt_s * kp * delta_pos);
st.pll_vel = st.pll_vel + dt_s * ki * delta_pos;
if st.pll_vel > c.HALL_PLL_VEL_MAX
    st.pll_vel = c.HALL_PLL_VEL_MAX;
elseif st.pll_vel < -c.HALL_PLL_VEL_MAX
    st.pll_vel = -c.HALL_PLL_VEL_MAX;
end
snap = 0.5 * dt_s * ki;
if abs(st.pll_vel) < snap
    st.pll_vel = 0.0;
end
st.mech_rpm_fast = st.pll_vel * (60.0 / c.HALL_PLL_CPR) / pole_pairs;
if st.mech_rpm_fast > 1.0
    st.direction = 1;
elseif st.mech_rpm_fast < -1.0
    st.direction = -1;
end
end

function st = update_speed(st, pole_pairs, dt_s, force_pll)
c = foc.constants();
rpm_abs = abs(st.mech_rpm);
if st.pll_low_speed_active && rpm_abs >= c.HALL_PLL_EXIT_RPM
    st.pll_low_speed_active = false;
elseif ~st.pll_low_speed_active && rpm_abs <= c.HALL_PLL_ENTER_RPM
    st.pll_low_speed_active = true;
    st.pll_pos = double(st.pll_count);
    st.pll_vel = st.mech_rpm * pole_pairs * (c.HALL_PLL_CPR / 60.0);
end
if force_pll || st.pll_low_speed_active
    st = pll_run(st, pole_pairs, dt_s);
else
    st.mech_rpm_fast = st.mech_rpm;
end
end

function [half_neg, half_pos] = neighbor_half(table, sector_center)
half_neg = pi / 6.0;
half_pos = pi / 6.0;
min_pos = 4.0;
min_neg = -4.0;
have_pos = false;
have_neg = false;
for i = 0:7
    other = foc.hall_decode(table, i);
    if isnan(other)
        continue
    end
    delta = foc.wrap_pi(other - sector_center);
    if delta > 1.0e-3 && delta < min_pos
        min_pos = delta;
        have_pos = true;
    elseif delta < -1.0e-3 && delta > min_neg
        min_neg = delta;
        have_neg = true;
    end
end
if have_pos && have_neg
    half_pos = 0.5 * min_pos;
    half_neg = 0.5 * (-min_neg);
end
end

function st = interpolate_theta(st, table, sector_center, pole_pairs)
c = foc.constants();
if st.direction == 0 || ~st.speed_valid || abs(st.mech_rpm_fast) < 1.0
    return
end
rpm_abs = abs(st.mech_rpm_fast);
if rpm_abs <= c.INTERP_START_RPM
    return
end
blend = (rpm_abs - c.INTERP_START_RPM) / (c.INTERP_FULL_RPM - c.INTERP_START_RPM);
if blend > 1.0
    blend = 1.0;
end
[half_neg, half_pos] = neighbor_half(table, sector_center);
if st.direction > 0
    half_back = half_neg;
    half_fwd = half_pos;
else
    half_back = half_pos;
    half_fwd = half_neg;
end
progress = rpm_abs * pole_pairs * (2.0 * pi / 60.0) * st.sector_timer_s;
if progress > (half_back + half_fwd)
    progress = half_back + half_fwd;
end
delta = double(st.direction) * (progress - half_back);
st.theta_rad = foc.wrap_pi(sector_center + blend * delta);
end
