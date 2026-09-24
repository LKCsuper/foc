function [v_alpha, v_beta, i_alpha, i_beta] = ekf_pmsm_voltage(theta, omega_e, iq, cfg)
%EKF_PMSM_VOLTAGE  对齐 test_motor_ekf.c 的理想表贴机电压方程（Id=0）。
s = sin(theta);
c = cos(theta);
flux = cfg.flux_linkage;
i_alpha = -iq * s;
i_beta = iq * c;
v_alpha = cfg.resistance * i_alpha - cfg.inductance * iq * omega_e * c - omega_e * flux * s;
v_beta = cfg.resistance * i_beta - cfg.inductance * iq * omega_e * s + omega_e * flux * c;
end
