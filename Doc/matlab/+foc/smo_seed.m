function obs = smo_seed(obs, cfg, angle, omega_e, i_alpha, i_beta)
%SMO_SEED  对齐 Motor_Smo_Seed：用假设电角、电角速度和当前电流装初值。
min_flux = 1.0e-6;
if isempty(obs) || ~isfield(obs, 'initialized')
    obs = struct();
end
if ~isfinite(angle) || ~isfinite(omega_e) || ~isfinite(i_alpha) || ~isfinite(i_beta)
    return
end
flux = cfg.flux_linkage;
if ~(flux >= min_flux) || ~isfinite(flux)
    flux = min_flux;
end
off = 0.0;
if isfield(cfg, 'phase_offset') && isfinite(cfg.phase_offset)
    off = cfg.phase_offset;
end
wrapped = foc.wrap_pi(angle);
s = sin(wrapped);
c = cos(wrapped);
obs.i_hat_alpha = i_alpha;
obs.i_hat_beta = i_beta;
obs.e_alpha = -omega_e * flux * s;
obs.e_beta = omega_e * flux * c;
obs.z_alpha = 0.0;
obs.z_beta = 0.0;
obs.magnitude = abs(omega_e) * flux;
obs.speed_e = omega_e;
obs.angle = foc.wrap_pi(wrapped + off);
obs.initialized = true;
end
