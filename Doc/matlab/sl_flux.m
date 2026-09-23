function [th_flux_deg, rpm_flux, flux_ok] = sl_flux(valpha, vbeta, ialpha, ibeta, rpm_if, th_if_deg)
%SL_FLUX  对齐 motor_flux.c Ortega，再经 foc_pll_run 同款 PLL + 10 Hz 转速低通。
persistent inited seeded x_a x_b pll_th pll_w pll_wf th_unw th_prev
c = foc.constants();
dt = c.DT;
% PMSM 植物的 dq 方程使用每相参数。线参数直接用于观测器会在
% 大电流弱磁时造成 R*i 补偿加倍，进而把观测角拉偏。
R = c.R_PHASE;
% VESC 以 (Ld+Lq)/2 为基值，再按当前 dq 电流方向补偿凸极性。
L_avg = 0.5 * (c.HFI_LD + c.HFI_LQ);
L_diff = c.HFI_LQ - c.HFI_LD;
L = L_avg;
flux = c.FLUX;
pp = c.POLE_PAIRS;
gamma = c.FLUX_GAIN;
seed_rpm = 150.0;
kp = c.SL_PLL_KP;
ki = c.SL_PLL_KI;
a_w = dt * 2.0 * pi * c.SL_PLL_SPD_HZ;
if a_w > 1.0
    a_w = 1.0;
end
we_if = rpm_if / 60.0 * 2.0 * pi * pp;
w_max = c.RATED_RPM / 60.0 * 2.0 * pi * pp * 1.5;

if isempty(inited)
    x_a = 0.0;
    x_b = 0.0;
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
    x_a = L * ialpha + flux * cos(ths);
    x_b = L * ibeta + flux * sin(ths);
    pll_th = ths;
    pll_w = we_if;
    pll_wf = we_if;
    seeded = true;
    did_seed = true;
end

if ~seeded
    th_flux_deg = 0.0;
    rpm_flux = 0.0;
    flux_ok = 0.0;
    return
end

% 对齐 VESC foc_observer_update 的 saliency adjustment：
%   L = Lavg - (Lq-Ld)/2 + (Lq-Ld)*Iq^2/(Id^2+Iq^2)
% 弱磁时 |Id| 为主，L 自然趋近 Ld，避免负 Id 被误投影成 Iq。
cs_obs = cos(pll_th);
sn_obs = sin(pll_th);
id_obs = ialpha * cs_obs + ibeta * sn_obs;
iq_obs = -ialpha * sn_obs + ibeta * cs_obs;
i2_obs = id_obs * id_obs + iq_obs * iq_obs;
if i2_obs > 0.01
    L = L_avg - 0.5 * L_diff + L_diff * iq_obs * iq_obs / i2_obs;
end

psi_a = x_a - L * ialpha;
psi_b = x_b - L * ibeta;
e_mag = flux * flux - (psi_a * psi_a + psi_b * psi_b);
if e_mag > 0.0
    e_mag = 0.0;
end
x_a = x_a + (valpha - R * ialpha + 0.5 * gamma * psi_a * e_mag) * dt;
x_b = x_b + (vbeta - R * ibeta + 0.5 * gamma * psi_b * e_mag) * dt;

if ~isfinite(x_a) || ~isfinite(x_b)
    x_a = 0.0;
    x_b = 0.0;
    pll_th = 0.0;
    pll_w = 0.0;
    pll_wf = 0.0;
    th_unw = 0.0;
    th_prev = 0.0;
    seeded = false;
    th_flux_deg = 0.0;
    rpm_flux = 0.0;
    flux_ok = 0.0;
    return
end

psi_a = x_a - L * ialpha;
psi_b = x_b - L * ibeta;
mag = hypot(psi_a, psi_b);
ang = foc.wrap_pi(atan2(psi_b, psi_a));

if ~isfinite(mag) || ~isfinite(ang)
    x_a = 0.0;
    x_b = 0.0;
    pll_th = 0.0;
    pll_w = 0.0;
    pll_wf = 0.0;
    th_unw = 0.0;
    th_prev = 0.0;
    seeded = false;
    th_flux_deg = 0.0;
    rpm_flux = 0.0;
    flux_ok = 0.0;
    return
end

flux_ok = double(mag > flux * 0.50 && mag < flux * 1.50);

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
    th_flux_deg = th_unw;
    rpm_flux = pll_wf / pp * 60.0 / (2.0 * pi);
    return
end

dth = foc.wrap_pi(pll_th - th_prev);
th_unw = th_unw + dth * 180.0 / pi;
th_prev = pll_th;
th_flux_deg = th_unw;
rpm_flux = pll_wf / pp * 60.0 / (2.0 * pi);
end
