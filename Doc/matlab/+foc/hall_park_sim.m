function s = hall_park_sim(table, seq, rpm, divider, use_pll_park)
%HALL_PARK_SIM  恒速空载：霍尔角 → Park 误差 → ωλ·sin(δ) → 电流 PI+RL。
c = foc.constants();
dt = c.DT;
pp = c.POLE_PAIRS;
we = rpm / 60.0 * pp * 2.0 * pi;
t_end = 0.20;
n = round(t_end / dt);
st = foc.hall_angle_reset();
st.table_ready = true;
theta_park = 0.0;
id = 0.0;
vint = 0.0;
n_skip = round(0.08 / dt);
n_keep = max(1, floor((n - n_skip) / 8));
delta_hist = zeros(n_keep, 1);
id_hist = zeros(n_keep, 1);
t_hist = zeros(n_keep, 1);
ih = 0;
n_use = 0;
sum_d2 = 0.0;
sum_id = 0.0;
sum_id2 = 0.0;
max_deg = 0.0;
hall_acc = 0;
hall_dt_acc = 0.0;
for k = 1:n
    t = (k - 1) * dt;
    theta_true = foc.wrap_pi(we * t);
    hall_state = foc.hall_state_from_theta(theta_true, table, seq);
    hall_acc = hall_acc + 1;
    hall_dt_acc = hall_dt_acc + dt;
    if hall_acc >= divider
        st = foc.hall_angle_update(st, table, hall_state, pp, hall_dt_acc, use_pll_park);
        hall_acc = 0;
        hall_dt_acc = 0.0;
        if use_pll_park
            center = foc.hall_decode(table, st.last_state);
            frac = st.pll_pos - double(st.pll_count);
            if frac > 3.0
                frac = frac - 6.0;
            elseif frac < -3.0
                frac = frac + 6.0;
            end
            theta_park = foc.wrap_pi(center + 2.0 * pi / 6.0 * frac);
        else
            theta_park = st.theta_rad;
        end
    end
    delta = foc.wrap_pi(theta_park - theta_true);
    vd_dist = we * c.FLUX * sin(delta);
    [vd, vint] = foc.current_pi(0.0, id, vint, c.KP, c.KI, dt, 20.0);
    id = id + dt / c.L_PHASE * (vd + vd_dist - c.R_PHASE * id);
    if k > n_skip
        n_use = n_use + 1;
        deg = abs(delta) * 180.0 / pi;
        sum_d2 = sum_d2 + delta * delta;
        sum_id = sum_id + id;
        sum_id2 = sum_id2 + id * id;
        if deg > max_deg
            max_deg = deg;
        end
        if mod(k, 8) == 0 && ih < n_keep
            ih = ih + 1;
            delta_hist(ih) = delta * 180.0 / pi;
            id_hist(ih) = id;
            t_hist(ih) = t;
        end
    end
end
s.rms_deg = sqrt(sum_d2 / max(n_use, 1)) * 180.0 / pi;
s.max_deg = max_deg;
s.rms_id = sqrt(sum_id2 / max(n_use, 1));
s.mean_id = sum_id / max(n_use, 1);
s.t = t_hist(1:ih);
s.delta_deg = delta_hist(1:ih);
s.id = id_hist(1:ih);
s.rpm = rpm;
end
