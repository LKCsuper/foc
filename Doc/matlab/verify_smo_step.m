function verify_smo_step()
%VERIFY_SMO_STEP  对照 test_motor_smo.c 的 100 Hz 跟踪，不启 Simulink。
this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
cfg.enabled = true;
cfg.resistance = 0.4;
cfg.inductance = 0.0008;
cfg.flux_linkage = 0.02573;
cfg.slide_gain = 0.0;
cfg.lpf = 0.0;
cfg.sigmoid_eps = 0.0;
cfg.phase_offset = 0.0;
dt = 1.0 / 15000.0;
we = 2.0 * pi * 100.0;
obs = foc.smo_seed(struct('initialized', false), cfg, 0.40, we, 0.0, 0.0);
theta = 0.40;
for i = 1:600
    [va, vb, ia, ib] = pmsm_v(theta, we, 0.0, cfg);
    [obs, out] = foc.smo_step(obs, cfg, va, vb, ia, ib, dt);
    theta = foc.wrap_pi(theta + we * dt);
end
err = abs(foc.wrap_pi(out.angle - theta));
rel_w = abs(out.speed_e - we) / we;
fprintf('seeded100Hz valid=%d bemf=%d |dth|=%.3f deg we_rel=%.3f mag=%.3f\n', ...
        out.valid, out.bemf_ok, err * 180.0 / pi, rel_w, out.magnitude);
if ~(out.valid && out.bemf_ok && err < 0.15 && rel_w < 0.20)
    error('verify_smo_step:track', 'seeded 100 Hz 未锁');
end
fprintf('VERIFY_SMO_STEP_OK\n');
end

function [va, vb, ia, ib] = pmsm_v(theta, we, iq, cfg)
s = sin(theta);
c = cos(theta);
flux = cfg.flux_linkage;
ia = -iq * s;
ib = iq * c;
va = cfg.resistance * ia - cfg.inductance * iq * we * c - we * flux * s;
vb = cfg.resistance * ib - cfg.inductance * iq * we * s + we * flux * c;
end
