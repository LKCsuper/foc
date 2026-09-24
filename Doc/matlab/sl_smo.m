function [th_smo_deg, rpm_smo, bemf_ok] = sl_smo(valpha, vbeta, ialpha, ibeta, rpm_if, th_if_deg)
%SL_SMO  SMO 原始角 → foc_pll_run 同款 PLL 锁角 → 10 Hz 低通出转速。
persistent inited seeded i_ha i_hb e_a e_b pll_th pll_w pll_wf th_unw th_prev
c = foc.constants();
dt = c.DT;
R = c.R_LINE;
L = c.L_LINE;
flux = c.FLUX;
pp = c.POLE_PAIRS;
gain = 24.0;
lpf = 0.002;
eps_s = 0.08;
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
    pll_th = ths;
    pll_w = we_if;
    pll_wf = we_if;
    seeded = true;
    did_seed = true;
end

z_a = gain * (i_ha - ialpha) / (abs(i_ha - ialpha) + eps_s);
z_b = gain * (i_hb - ibeta) / (abs(i_hb - ibeta) + eps_s);
i_ha = i_ha + dt * inv_l * (valpha - R * i_ha - e_a);
i_hb = i_hb + dt * inv_l * (vbeta - R * i_hb - e_b);

e_a_old = e_a;
e_b_old = e_b;
rot_d = we_if * dt;
rc = cos(rot_d);
rs = sin(rot_d);
e_a = e_a_old * rc - e_b_old * rs + lpf * z_a;
e_b = e_a_old * rs + e_b_old * rc + lpf * z_b;

if ~isfinite(i_ha) || ~isfinite(i_hb) || ~isfinite(e_a) || ~isfinite(e_b)
    i_ha = ialpha;
    i_hb = ibeta;
    e_a = 0.0;
    e_b = 0.0;
    pll_th = 0.0;
    pll_w = 0.0;
    pll_wf = 0.0;
    th_unw = 0.0;
    th_prev = 0.0;
    seeded = false;
    th_smo_deg = 0.0;
    rpm_smo = 0.0;
    bemf_ok = 0.0;
    return
end

mag = hypot(e_a, e_b);
if rpm_if >= 0.0
    angle_raw = atan2(-e_a, e_b);
else
    angle_raw = atan2(e_a, -e_b);
end
ang = foc.wrap_pi(angle_raw);

if ~isfinite(mag) || ~isfinite(ang)
    i_ha = ialpha;
    i_hb = ibeta;
    e_a = 0.0;
    e_b = 0.0;
    pll_th = 0.0;
    pll_w = 0.0;
    pll_wf = 0.0;
    th_unw = 0.0;
    th_prev = 0.0;
    seeded = false;
    th_smo_deg = 0.0;
    rpm_smo = 0.0;
    bemf_ok = 0.0;
    return
end

bemf_ok = double(mag > min_bemf);
if ~seeded
    th_smo_deg = 0.0;
    rpm_smo = 0.0;
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
    th_smo_deg = th_unw;
    rpm_smo = pll_wf / pp * 60.0 / (2.0 * pi);
    return
end

dth = foc.wrap_pi(pll_th - th_prev);
th_unw = th_unw + dth * 180.0 / pi;
th_prev = pll_th;
th_smo_deg = th_unw;
rpm_smo = pll_wf / pp * 60.0 / (2.0 * pi);
end
