function s = ekf_if_handoff(cfg, rpm, seed_deg, nstep, dt)
%EKF_IF_HANDOFF  欧拉植物 + I/F 种子偏差：模拟开环角交给 EKF 后能不能锁。
%
%  植物：表贴机电压方程，ω 钳在目标（空载高惯量近似）。
%  电压：按真 θ 的 Id=0、Iq=0 反电势（空载 I/F 到速后）。
%  种子：I/F 角 = 真角 + seed_deg，ω 用目标电角速度。

if nargin < 5 || isempty(dt)
    dt = 1.0 / 15000.0;
end
c = foc.constants();
omega_e = rpm / 60.0 * 2.0 * pi * c.POLE_PAIRS;
theta = 0.0;
ia = 0.0;
ib = 0.0;
seed_th = seed_deg * pi / 180.0;
obs = seed(cfg, seed_th, omega_e, ia, ib);

err = zeros(nstep, 1);
we = zeros(nstep, 1);
n_ok = 0;
speed_ok_last = false;
angle_last = 0.0;
speed_last = 0.0;

R = cfg.resistance;
L = cfg.inductance;
flux = cfg.flux_linkage;
inv_l = 1.0 / L;

for k = 1:nstep
    s_th = sin(theta);
    c_th = cos(theta);
    % 空载稳态：v ≈ e，Iq≈0。I/F 到速后电流很小。
    va = -omega_e * flux * s_th;
    vb = omega_e * flux * c_th;
    ia = ia + dt * inv_l * (va - R * ia + omega_e * flux * s_th);
    ib = ib + dt * inv_l * (vb - R * ib - omega_e * flux * c_th);
    [obs, out, ok] = step(obs, cfg, va, vb, ia, ib, dt);
    if ~ok
        break
    end
    n_ok = k;
    speed_ok_last = out.speed_ok;
    angle_last = out.angle;
    speed_last = out.speed_e;
    err(k) = wrap(out.angle - theta);
    we(k) = out.speed_e;
    theta = wrap(theta + omega_e * dt);
end

s.ok = n_ok == nstep;
s.n_ok = n_ok;
s.speed_ok = speed_ok_last;
s.omega_e = omega_e;
s.rpm = rpm;
s.seed_deg = seed_deg;
s.err = err(1:max(n_ok, 1));
s.we = we(1:max(n_ok, 1));
if n_ok > 0
    tail = max(1, n_ok - 199):n_ok;
    s.tail_rms_deg = sqrt(mean(err(tail).^2)) * 180.0 / pi;
    s.tail_max_deg = max(abs(err(tail))) * 180.0 / pi;
    s.angle_err = abs(wrap(angle_last - theta));
    s.we_err_rel = abs(speed_last - omega_e) / max(abs(omega_e), 1.0);
    s.speed_e = speed_last;
else
    s.tail_rms_deg = Inf;
    s.tail_max_deg = Inf;
    s.angle_err = Inf;
    s.we_err_rel = Inf;
    s.speed_e = 0.0;
end
end

function x = wrap(x)
two_pi = 2.0 * pi;
x = mod(x + pi, two_pi) - pi;
end

function obs = seed(cfg, angle, omega_e, i_alpha, i_beta)
max_w = 1500.0;
obs.initialized = true;
obs.x = [double(i_alpha); double(i_beta); max(min(double(omega_e), max_w), -max_w); wrap(angle)];
obs.p = zeros(4, 4);
obs.p(1, 1) = 1.0e-4;
obs.p(2, 2) = 1.0e-4;
obs.p(3, 3) = 25.0;
obs.p(4, 4) = 0.02;
obs.speed_e = obs.x(3);
off = 0.0;
if isfield(cfg, 'phase_offset') && isfinite(cfg.phase_offset)
    off = double(cfg.phase_offset);
end
obs.angle = wrap(obs.x(4) + off);
end

function [obs, output, ok] = step(obs, cfg, v_alpha, v_beta, i_alpha, i_beta, dt)
output.valid = false;
output.speed_ok = false;
output.i_hat_alpha = 0.0;
output.i_hat_beta = 0.0;
output.angle = 0.0;
output.speed_e = 0.0;
ok = false;

min_l = 1.0e-6;
min_flux = 1.0e-6;
max_w = 1500.0;
min_w = 50.0;
s_det_min = 1.0e-12;
q_i_def = 1.0e-3;
q_w_def = 80.0;
q_th_def = 1.0e-4;
r_def = 5.0e-4;

if ~cfg.enabled || ~(dt > 0.0) || ~isfinite(dt) || ~(cfg.inductance >= min_l)
    obs.initialized = false;
    return
end

resistance = cfg.resistance;
if ~(resistance >= 0.0)
    resistance = 0.0;
end
inductance = cfg.inductance;
flux = cfg.flux_linkage;
if ~(flux >= min_flux)
    flux = min_flux;
end
inv_l = 1.0 / inductance;
q_i = cfg.q_current;
if ~(q_i > 0.0)
    q_i = q_i_def;
end
q_w = cfg.q_speed;
if ~(q_w > 0.0)
    q_w = q_w_def;
end
q_th = cfg.q_angle;
if ~(q_th > 0.0)
    q_th = q_th_def;
end
r_i = cfg.r_current;
if ~(r_i > 0.0)
    r_i = r_def;
end

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
off = 0.0;
if isfield(cfg, 'phase_offset') && isfinite(cfg.phase_offset)
    off = double(cfg.phase_offset);
end
obs.angle = wrap(obs.x(4) + off);
if ~isfinite(obs.angle) || ~isfinite(obs.speed_e)
    obs.initialized = false;
    return
end

output.valid = true;
output.speed_ok = abs(obs.speed_e) > min_w;
output.i_hat_alpha = obs.x(1);
output.i_hat_beta = obs.x(2);
output.angle = obs.angle;
output.speed_e = obs.speed_e;
ok = true;
end
