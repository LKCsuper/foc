function [obs, output, ok] = ekf_step(obs, cfg, v_alpha, v_beta, i_alpha, i_beta, dt)
%EKF_STEP  对齐 Motor_Ekf_Step：四状态 EKF 一拍预测+校正。
%
%  x = [iα, iβ, ωe, θe]
%  返回的 obs 必须接回调用方（MATLAB 结构体按值传递）。

output = empty_output();
ok = false;
if nargin < 7 || isempty(obs) || isempty(cfg)
    return
end

min_l = 1.0e-6;
min_flux = 1.0e-6;
max_w = 1500.0;
min_w = 50.0;
s_det_min = 1.0e-12;
q_i_def = 1.0e-3;
q_w_def = 80.0;
q_th_def = 1.0e-4;
r_def = 5.0e-4;

enabled = isfield(cfg, 'enabled') && cfg.enabled;
if ~enabled || ~(dt > 0.0) || ~isfinite(dt) || ~isfinite(v_alpha) || ~isfinite(v_beta) || ...
        ~isfinite(i_alpha) || ~isfinite(i_beta) || ~(cfg.inductance >= min_l)
    obs = foc.ekf_reset(obs);
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
    obs = foc.ekf_seed(obs, cfg, 0.0, 0.0, i_alpha, i_beta);
end

omega = obs.x(3);
theta = obs.x(4);
s_th = sin(theta);
c_th = cos(theta);

% F ≈ I + dt·∂f/∂x，在预测前的 x 处线性化。
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
obs.x(4) = foc.wrap_pi(obs.x(4) + omega * dt);

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
    obs = foc.ekf_reset(obs);
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

inn = [i_alpha - obs.x(1); i_beta - obs.x(2)];
obs.x = obs.x + K * inn;

A = eye(4);
A(:, 1) = A(:, 1) - K(:, 1);
A(:, 2) = A(:, 2) - K(:, 2);
obs.p = A * p_pred * A.' + r_i * (K * K.');
obs.p = 0.5 * (obs.p + obs.p.');

if any(~isfinite(obs.x)) || any(~isfinite(diag(obs.p)))
    obs = foc.ekf_reset(obs);
    return
end

obs.x(4) = foc.wrap_pi(obs.x(4));
obs.x(3) = max(min(obs.x(3), max_w), -max_w);
obs.speed_e = obs.x(3);
off = 0.0;
if isfield(cfg, 'phase_offset') && isfinite(cfg.phase_offset)
    off = double(cfg.phase_offset);
end
obs.angle = foc.wrap_pi(obs.x(4) + off);

if ~isfinite(obs.angle) || ~isfinite(obs.speed_e)
    obs = foc.ekf_reset(obs);
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

function output = empty_output()
output.valid = false;
output.speed_ok = false;
output.i_hat_alpha = 0.0;
output.i_hat_beta = 0.0;
output.angle = 0.0;
output.speed_e = 0.0;
end
