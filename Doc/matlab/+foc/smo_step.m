function [obs, out] = smo_step(obs, cfg, v_alpha, v_beta, i_alpha, i_beta, dt)
%SMO_STEP  对齐 Motor_Smo_Step：饱和滑模估 ê，电流方程只放 ê。
min_l = 1.0e-6;
min_flux = 1.0e-6;
max_gain = 80.0;
def_gain = 24.0;
def_lpf = 0.002;
def_eps = 0.08;
min_bemf = 0.50;

out.valid = false;
out.bemf_ok = false;
out.e_alpha = 0.0;
out.e_beta = 0.0;
out.magnitude = 0.0;
out.angle = 0.0;
out.speed_e = 0.0;

if isempty(obs) || ~isfield(obs, 'initialized')
    obs = empty_obs();
end
if ~isfield(cfg, 'enabled') || ~cfg.enabled || ~(dt > 0.0) || ~isfinite(dt) || ...
        ~isfinite(v_alpha) || ~isfinite(v_beta) || ~isfinite(i_alpha) || ~isfinite(i_beta) || ...
        ~(cfg.inductance >= min_l)
    obs = empty_obs();
    return
end

R = cfg.resistance;
if ~(R >= 0.0) || ~isfinite(R)
    R = 0.0;
end
L = cfg.inductance;
inv_l = 1.0 / L;
gain = cfg.slide_gain;
if ~(gain > 0.0)
    gain = def_gain;
end
if gain > max_gain
    gain = max_gain;
end
lpf = cfg.lpf;
if ~(lpf > 0.0 && lpf <= 1.0)
    lpf = def_lpf;
end
eps = cfg.sigmoid_eps;
if ~(eps > 0.0)
    eps = def_eps;
end
flux = cfg.flux_linkage;
if ~(flux >= min_flux) || ~isfinite(flux)
    flux = min_flux;
end
phase_off = cfg.phase_offset;
if ~isfinite(phase_off)
    phase_off = 0.0;
end

if ~obs.initialized
    obs = foc.smo_seed(obs, cfg, 0.0, 0.0, i_alpha, i_beta);
end

obs.z_alpha = gain * smo_sat(obs.i_hat_alpha - i_alpha, eps);
obs.z_beta = gain * smo_sat(obs.i_hat_beta - i_beta, eps);
obs.i_hat_alpha = obs.i_hat_alpha + dt * inv_l * (v_alpha - R * obs.i_hat_alpha - obs.e_alpha);
obs.i_hat_beta = obs.i_hat_beta + dt * inv_l * (v_beta - R * obs.i_hat_beta - obs.e_beta);

e_a_old = obs.e_alpha;
e_b_old = obs.e_beta;
rot_d = obs.speed_e * dt;
rc = cos(rot_d);
rs = sin(rot_d);
obs.e_alpha = e_a_old * rc - e_b_old * rs + lpf * obs.z_alpha;
obs.e_beta = e_a_old * rs + e_b_old * rc + lpf * obs.z_beta;

if ~isfinite(obs.i_hat_alpha) || ~isfinite(obs.i_hat_beta) || ...
        ~isfinite(obs.e_alpha) || ~isfinite(obs.e_beta)
    obs = empty_obs();
    return
end

obs.magnitude = hypot(obs.e_alpha, obs.e_beta);
cross = e_a_old * obs.e_beta - e_b_old * obs.e_alpha;
if obs.magnitude > 0.0 && flux >= min_flux
    if abs(cross) > 1.0e-12
        if cross >= 0.0
            sgn = 1.0;
        else
            sgn = -1.0;
        end
        obs.speed_e = sgn * (obs.magnitude / flux);
    else
        if obs.speed_e >= 0.0
            sgn = 1.0;
        else
            sgn = -1.0;
        end
        obs.speed_e = sgn * (obs.magnitude / flux);
    end
else
    obs.speed_e = 0.0;
end

if obs.speed_e >= 0.0
    angle_raw = atan2(-obs.e_alpha, obs.e_beta);
else
    angle_raw = atan2(obs.e_alpha, -obs.e_beta);
end
obs.angle = foc.wrap_pi(angle_raw + phase_off);

if ~isfinite(obs.magnitude) || ~isfinite(obs.angle) || ~isfinite(obs.speed_e)
    obs = empty_obs();
    return
end

out.valid = true;
out.bemf_ok = obs.magnitude > min_bemf;
out.e_alpha = obs.e_alpha;
out.e_beta = obs.e_beta;
out.magnitude = obs.magnitude;
out.angle = obs.angle;
out.speed_e = obs.speed_e;
end

function y = smo_sat(err, eps)
y = err / (abs(err) + eps);
end

function obs = empty_obs()
obs.initialized = false;
obs.i_hat_alpha = 0.0;
obs.i_hat_beta = 0.0;
obs.z_alpha = 0.0;
obs.z_beta = 0.0;
obs.e_alpha = 0.0;
obs.e_beta = 0.0;
obs.magnitude = 0.0;
obs.angle = 0.0;
obs.speed_e = 0.0;
end
