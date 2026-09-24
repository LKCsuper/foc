function s = ekf_motor_sim(varargin)
%EKF_MOTOR_SIM  ATK-BL57H95D20E 铭牌植物：电气+机械 + I/F 电流环，EKF 影子观察。
%
%  植物与 EKF 都用线电阻/线电感（与 motor_ekf.c / sensorless_motor_r 一致）。
%  电压用教科书逆 Park，不含 HD75 的 180° 与 A-C-B。平均电压模型，不是开关 PWM。
%  固件 ol_iq_set=0.6 且 start_phase_boost=π/2，等效把电流打在虚拟 d 轴上拖转子。
%  本仿真直接 Id=0.6、Iq=0；先对齐 40 ms，再 1200 RPM/s 拉到 500 RPM。
%  B 铭牌没有，按空载 3000 RPM 约 5% 额定转矩假设。

c = foc.constants();
p.dt = c.DT;
p.t_end = 1.50;
p.rpm_target = c.SL_TRANS_RPM;
p.ramp_rpm_s = 1200.0;
p.t_align = 0.04;
p.iq_if = 0.0;
p.id_if = c.IQ_IF;
p.tl = 0.0;
p.kp = c.KP;
p.ki = c.KI;
p.takeover = false;
if nargin >= 1 && isstruct(varargin{1})
    q = varargin{1};
    fn = fieldnames(q);
    for i = 1:numel(fn)
        p.(fn{i}) = q.(fn{i});
    end
end

dt = p.dt;
nstep = max(2, round(p.t_end / dt));
dec = 15;
nlog = floor((nstep - 1) / dec) + 1;

R = c.R_LINE;
L = c.L_LINE;
flux = c.FLUX;
pp = c.POLE_PAIRS;
J = c.J;
B = c.B_VISC;
inv_l = 1.0 / L;
vlim = c.VBUS / sqrt(3.0) * c.MAX_DUTY;

cfg.enabled = true;
cfg.resistance = R;
cfg.inductance = L;
cfg.flux_linkage = flux;
cfg.q_current = 0.0;
cfg.q_speed = 0.0;
cfg.q_angle = 0.0;
cfg.r_current = 0.0;
cfg.phase_offset = 0.0;

ia = 0.0;
ib = 0.0;
theta_e = 0.0;
omega_m = 0.0;
theta_if = 0.0;
rpm_if = 0.0;
vd_int = 0.0;
vq_int = 0.0;
va_cmd = 0.0;
vb_cmd = 0.0;
obs = seed(cfg, 0.0, 0.0, 0.0, 0.0);

s.t = zeros(nlog, 1);
s.rpm_true = zeros(nlog, 1);
s.rpm_if = zeros(nlog, 1);
s.rpm_ekf = zeros(nlog, 1);
s.id_r = zeros(nlog, 1);
s.iq_r = zeros(nlog, 1);
s.id_c = zeros(nlog, 1);
s.iq_c = zeros(nlog, 1);
s.ekf_err_deg = zeros(nlog, 1);
s.if_err_deg = zeros(nlog, 1);
s.th_e = zeros(nlog, 1);
s.th_if = zeros(nlog, 1);
s.th_ekf = zeros(nlog, 1);
s.te = zeros(nlog, 1);
s.ia = zeros(nlog, 1);
s.speed_ok = false(nlog, 1);

ekf_ok = true;
ilog = 0;
park_ekf = false;
max_i = 0.0;
err_last = Inf;
rpm_last = 0.0;
speed_ok_last = false;

for k = 1:nstep
    [id_c, iq_c] = park(ia, ib, theta_if);
    [id_r, iq_r] = park(ia, ib, theta_e);
    te = 1.5 * pp * flux * iq_r;
    if abs(ia) > max_i
        max_i = abs(ia);
    end
    if abs(ib) > max_i
        max_i = abs(ib);
    end

    if p.takeover && park_ekf && obs.initialized
        theta_ctrl = obs.angle;
    else
        theta_ctrl = theta_if;
    end
    [id_pi, iq_pi] = park(ia, ib, theta_ctrl);
    [vd, vd_int] = pi_axis(p.id_if, id_pi, vd_int, p.kp, p.ki, dt, vlim);
    [vq, vq_int] = pi_axis(p.iq_if, iq_pi, vq_int, p.kp, p.ki, dt, vlim);
    mag = hypot(vd, vq);
    if mag > vlim && mag > 0.0
        sc = vlim / mag;
        vd = vd * sc;
        vq = vq * sc;
    end
    [va, vb] = inv_park(vd, vq, theta_ctrl);

    [obs, out, ok] = step(obs, cfg, va_cmd, vb_cmd, ia, ib, dt);
    if ~ok
        ekf_ok = false;
        break
    end
    va_cmd = va;
    vb_cmd = vb;

    rpm_true = omega_m * 60.0 / (2.0 * pi);
    if p.takeover && out.speed_ok && rpm_true > 300.0 && abs(wrap(out.angle - theta_e)) < (15.0 * pi / 180.0)
        park_ekf = true;
    end

    if mod(k - 1, dec) == 0
        ilog = ilog + 1;
        s.t(ilog) = (k - 1) * dt;
        s.rpm_true(ilog) = rpm_true;
        s.rpm_if(ilog) = rpm_if;
        s.rpm_ekf(ilog) = out.speed_e / pp * 60.0 / (2.0 * pi);
        s.id_r(ilog) = id_r;
        s.iq_r(ilog) = iq_r;
        s.id_c(ilog) = id_c;
        s.iq_c(ilog) = iq_c;
        s.ekf_err_deg(ilog) = wrap(out.angle - theta_e) * 180.0 / pi;
        s.if_err_deg(ilog) = wrap(theta_if - theta_e) * 180.0 / pi;
        s.th_e(ilog) = theta_e;
        s.th_if(ilog) = theta_if;
        s.th_ekf(ilog) = out.angle;
        s.te(ilog) = te;
        s.ia(ilog) = ia;
        s.speed_ok(ilog) = out.speed_ok;
    end
    err_last = abs(wrap(out.angle - theta_e));
    rpm_last = rpm_true;
    speed_ok_last = out.speed_ok;

    omega_e = omega_m * pp;
    ia = ia + dt * inv_l * (va - R * ia + omega_e * flux * sin(theta_e));
    ib = ib + dt * inv_l * (vb - R * ib - omega_e * flux * cos(theta_e));
    omega_m = omega_m + dt * (te - B * omega_m - p.tl) / J;
    theta_e = wrap(theta_e + dt * omega_m * pp);

    if ((k - 1) * dt) >= p.t_align && rpm_if < p.rpm_target
        rpm_if = rpm_if + p.ramp_rpm_s * dt;
        if rpm_if > p.rpm_target
            rpm_if = p.rpm_target;
        end
    end
    omega_if = rpm_if / 60.0 * 2.0 * pi * pp;
    theta_if = wrap(theta_if + omega_if * dt);
end

if ilog < nlog
    s.t = s.t(1:max(ilog, 1));
    s.rpm_true = s.rpm_true(1:max(ilog, 1));
    s.rpm_if = s.rpm_if(1:max(ilog, 1));
    s.rpm_ekf = s.rpm_ekf(1:max(ilog, 1));
    s.id_r = s.id_r(1:max(ilog, 1));
    s.iq_r = s.iq_r(1:max(ilog, 1));
    s.id_c = s.id_c(1:max(ilog, 1));
    s.iq_c = s.iq_c(1:max(ilog, 1));
    s.ekf_err_deg = s.ekf_err_deg(1:max(ilog, 1));
    s.if_err_deg = s.if_err_deg(1:max(ilog, 1));
    s.th_e = s.th_e(1:max(ilog, 1));
    s.th_if = s.th_if(1:max(ilog, 1));
    s.th_ekf = s.th_ekf(1:max(ilog, 1));
    s.te = s.te(1:max(ilog, 1));
    s.ia = s.ia(1:max(ilog, 1));
    s.speed_ok = s.speed_ok(1:max(ilog, 1));
end

s.ok = ekf_ok && ilog > 10;
s.park_ekf = park_ekf;
s.max_i = max_i;
s.final_rpm = rpm_last;
s.final_ekf_err = err_last;
s.speed_ok_last = speed_ok_last;
s.nstep = k;
s.J = J;
s.iq_if = p.iq_if;
s.rpm_target = p.rpm_target;
end

function x = wrap(x)
x = mod(x + pi, 2.0 * pi) - pi;
end

function [id, iq] = park(ia, ib, th)
s = sin(th);
c = cos(th);
id = ia * c + ib * s;
iq = -ia * s + ib * c;
end

function [va, vb] = inv_park(vd, vq, th)
s = sin(th);
c = cos(th);
va = vd * c - vq * s;
vb = vd * s + vq * c;
end

function [v_out, v_int] = pi_axis(target, measured, v_int, kp, ki, dt, v_limit)
ierr = target - measured;
v_int = v_int + ierr * ki * dt;
v_out = v_int + ierr * kp;
if v_out > v_limit
    v_out = v_limit;
    v_int = v_limit - ierr * kp;
elseif v_out < -v_limit
    v_out = -v_limit;
    v_int = -v_limit - ierr * kp;
end
end

function obs = seed(cfg, angle, omega_e, i_alpha, i_beta)
obs.initialized = true;
obs.x = [double(i_alpha); double(i_beta); double(omega_e); wrap(angle)];
obs.p = zeros(4, 4);
obs.p(1, 1) = 1.0e-4;
obs.p(2, 2) = 1.0e-4;
obs.p(3, 3) = 25.0;
obs.p(4, 4) = 0.02;
obs.speed_e = obs.x(3);
obs.angle = wrap(obs.x(4) + cfg.phase_offset);
end

function [obs, output, ok] = step(obs, cfg, v_alpha, v_beta, i_alpha, i_beta, dt)
output.valid = false;
output.speed_ok = false;
output.angle = 0.0;
output.speed_e = 0.0;
ok = false;
min_l = 1.0e-6;
min_flux = 1.0e-6;
max_w = 1500.0;
min_w = 50.0;
s_det_min = 1.0e-12;
if ~cfg.enabled || ~(dt > 0.0) || ~(cfg.inductance >= min_l)
    obs.initialized = false;
    return
end
resistance = cfg.resistance;
inductance = cfg.inductance;
flux = cfg.flux_linkage;
if ~(flux >= min_flux)
    flux = min_flux;
end
inv_l = 1.0 / inductance;
q_i = 1.0e-3;
q_w = 80.0;
q_th = 1.0e-4;
r_i = 5.0e-4;
if ~obs.initialized
    obs = seed(cfg, 0.0, 0.0, i_alpha, i_beta);
end
omega = obs.x(3);
theta = obs.x(4);
s_th = sin(theta);
c_th = cos(theta);
F = zeros(4, 4);
F(1, 1) = 1.0 - dt * resistance * inv_l;
F(1, 3) = dt * flux * inv_l * s_th;
F(1, 4) = dt * flux * inv_l * omega * c_th;
F(2, 2) = 1.0 - dt * resistance * inv_l;
F(2, 3) = -dt * flux * inv_l * c_th;
F(2, 4) = dt * flux * inv_l * omega * s_th;
F(3, 3) = 1.0;
F(4, 3) = dt;
F(4, 4) = 1.0;
obs.x(1) = obs.x(1) + dt * inv_l * (v_alpha - resistance * obs.x(1) + omega * flux * s_th);
obs.x(2) = obs.x(2) + dt * inv_l * (v_beta - resistance * obs.x(2) - omega * flux * c_th);
obs.x(4) = wrap(obs.x(4) + omega * dt);
p_pred = F * obs.p * F.';
p_pred(1, 1) = p_pred(1, 1) + q_i;
p_pred(2, 2) = p_pred(2, 2) + q_i;
p_pred(3, 3) = p_pred(3, 3) + q_w;
p_pred(4, 4) = p_pred(4, 4) + q_th;
s00 = p_pred(1, 1) + r_i;
s01 = p_pred(1, 2);
s11 = p_pred(2, 2) + r_i;
det = s00 * s11 - s01 * s01;
if ~(det > s_det_min) || ~isfinite(det)
    obs.initialized = false;
    return
end
inv_det = 1.0 / det;
K = zeros(4, 2);
for i = 1:4
    p_i0 = p_pred(i, 1);
    p_i1 = p_pred(i, 2);
    K(i, 1) = (p_i0 * s11 - p_i1 * s01) * inv_det;
    K(i, 2) = (-p_i0 * s01 + p_i1 * s00) * inv_det;
end
obs.x = obs.x + K * [i_alpha - obs.x(1); i_beta - obs.x(2)];
A = eye(4);
A(:, 1) = A(:, 1) - K(:, 1);
A(:, 2) = A(:, 2) - K(:, 2);
obs.p = A * p_pred * A.' + r_i * (K * K.');
obs.p = 0.5 * (obs.p + obs.p.');
if any(~isfinite(obs.x)) || any(~isfinite(diag(obs.p)))
    obs.initialized = false;
    return
end
obs.x(4) = wrap(obs.x(4));
obs.x(3) = max(min(obs.x(3), max_w), -max_w);
obs.speed_e = obs.x(3);
obs.angle = wrap(obs.x(4) + cfg.phase_offset);
if ~isfinite(obs.angle) || ~isfinite(obs.speed_e)
    obs.initialized = false;
    return
end
output.valid = true;
output.speed_ok = abs(obs.speed_e) > min_w;
output.angle = obs.angle;
output.speed_e = obs.speed_e;
ok = true;
end
