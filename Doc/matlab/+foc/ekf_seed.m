function obs = ekf_seed(obs, cfg, angle, omega_e, i_alpha, i_beta)
%EKF_SEED  对齐 Motor_Ekf_Seed：电流较确信，转速/角方差留大。
if nargin < 1 || isempty(obs)
    obs = foc.ekf_reset();
end
if nargin < 2 || isempty(cfg)
    return
end
if ~(isfinite(angle) && isfinite(omega_e) && isfinite(i_alpha) && isfinite(i_beta))
    return
end

max_w = 1500.0;
obs = foc.ekf_reset(obs);
obs.x = [double(i_alpha); double(i_beta); max(min(double(omega_e), max_w), -max_w); foc.wrap_pi(angle)];
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
obs.angle = foc.wrap_pi(obs.x(4) + off);
obs.initialized = true;
end
