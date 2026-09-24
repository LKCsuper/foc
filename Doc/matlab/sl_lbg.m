function [th_lbg_deg, rpm_lbg, bemf_ok] = sl_lbg(valpha, vbeta, ialpha, ibeta, rpm_if, th_if_deg)
%SL_LBG  对齐 motor_luenberger.c，再经 foc_pll_run 同款 PLL + 10 Hz 转速低通。
persistent inited seeded i_ha i_hb e_a e_b speed_e pll_th pll_w pll_wf th_unw th_prev
c = foc.constants();
dt = c.DT;
R = c.R_LINE;
L = c.L_LINE;
flux = c.FLUX;
pp = c.POLE_PAIRS;
wn = 2500.0;
zeta = 0.707;
l1 = 2.0 * zeta * wn - R / L;
if l1 < 0.0
    l1 = 0.0;
end
l2 = wn * wn * L;
min_bemf = 0.50;
seed_rpm = 150.0;
kp = c.SL_PLL_KP;
ki = c.SL_PLL_KI;
a_w = dt * 2.0 * pi * c.SL_PLL_SPD_HZ;
if a_w > 1.0
    a_w = 1.0;
end
inv_l = 1.0 / L;
we_if = rpm_if / 60.0 * 2.0 * pi * pp;
w_max = c.RATED_RPM / 60.0 * 2.0 * pi * pp * 1.5;

if isempty(inited)
    i_ha = ialpha;
    i_hb = ibeta;
    e_a = 0.0;
    e_b = 0.0;
    speed_e = 0.0;
    pll_th = 0.0;
    pll_w = 0.0;
    pll_wf = 0.0;
    th_unw = 0.0;
    th_prev = 0.0;
    seeded = false;
    inited = true;
end

did_seed = false;
if ~seeded && abs(rpm_if) > seed_rpm
    ths = foc.wrap_pi(th_if_deg * pi / 180.0);
    i_ha = ialpha;
    i_hb = ibeta;
    e_a = -we_if * flux * sin(ths);
    e_b = we_if * flux * cos(ths);
    speed_e = we_if;
    pll_th = ths;
    pll_w = we_if;
    pll_wf = we_if;
    seeded = true;
    did_seed = true;
end

eps_a = ialpha - i_ha;
eps_b = ibeta - i_hb;
di_a = inv_l * (valpha - R * i_ha - e_a) + l1 * eps_a;
di_b = inv_l * (vbeta - R * i_hb - e_b) + l1 * eps_b;
de_a = -speed_e * e_b - l2 * eps_a;
de_b = speed_e * e_a - l2 * eps_b;
e_a_old = e_a;
e_b_old = e_b;
i_ha = i_ha + di_a * dt;
i_hb = i_hb + di_b * dt;
e_a = e_a + de_a * dt;
e_b = e_b + de_b * dt;

if ~isfinite(i_ha) || ~isfinite(i_hb) || ~isfinite(e_a) || ~isfinite(e_b)
    i_ha = ialpha;
    i_hb = ibeta;
    e_a = 0.0;
    e_b = 0.0;
    speed_e = 0.0;
    pll_th = 0.0;
    pll_w = 0.0;
    pll_wf = 0.0;
    th_unw = 0.0;
    th_prev = 0.0;
    seeded = false;
    th_lbg_deg = 0.0;
    rpm_lbg = 0.0;
    bemf_ok = 0.0;
    return
end

mag = hypot(e_a, e_b);
cross = e_a_old * e_b - e_b_old * e_a;
if mag > 0.0
    if abs(cross) > 1.0e-12
        if cross >= 0.0
            sgn = 1.0;
        else
            sgn = -1.0;
        end
    else
        if speed_e >= 0.0
            sgn = 1.0;
        else
            sgn = -1.0;
        end
    end
    speed_e = sgn * (mag / flux);
else
    speed_e = 0.0;
end

if speed_e >= 0.0
    ang = foc.wrap_pi(atan2(-e_a, e_b));
else
    ang = foc.wrap_pi(atan2(e_a, -e_b));
end

if ~isfinite(mag) || ~isfinite(ang) || ~isfinite(speed_e)
    i_ha = ialpha;
    i_hb = ibeta;
    e_a = 0.0;
    e_b = 0.0;
    speed_e = 0.0;
    pll_th = 0.0;
    pll_w = 0.0;
    pll_wf = 0.0;
    th_unw = 0.0;
    th_prev = 0.0;
    seeded = false;
    th_lbg_deg = 0.0;
    rpm_lbg = 0.0;
    bemf_ok = 0.0;
    return
end

bemf_ok = double(mag > min_bemf);
if ~seeded
    th_lbg_deg = 0.0;
    rpm_lbg = 0.0;
    return
end

if ~did_seed
    delta = foc.wrap_pi(ang - pll_th);
    pll_th = foc.wrap_pi(pll_th + (pll_w + kp * delta) * dt);
    pll_w = pll_w + ki * delta * dt;
    if pll_w > w_max
        pll_w = w_max;
    elseif pll_w < -w_max
        pll_w = -w_max;
    end
    pll_wf = pll_wf + a_w * (pll_w - pll_wf);
end

if did_seed
    pll_th = ang;
    pll_w = we_if;
    pll_wf = we_if;
    th_unw = th_if_deg;
    th_prev = pll_th;
    th_lbg_deg = th_unw;
    rpm_lbg = pll_wf / pp * 60.0 / (2.0 * pi);
    return
end

dth = foc.wrap_pi(pll_th - th_prev);
th_unw = th_unw + dth * 180.0 / pi;
th_prev = pll_th;
th_lbg_deg = th_unw;
rpm_lbg = pll_wf / pp * 60.0 / (2.0 * pi);
end
