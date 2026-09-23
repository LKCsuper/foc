function [ia, ib, ic, rpm, th_deg, te] = sl_pmsm_discrete(va, vb, vc)
%SL_PMSM_DISCRETE  与项目 Clarke/Park 约定一致的离散凸极 PMSM 植物。
%
% 输入为相对中性点三相电压 (V)，输出相电流 (A)、机械转速 (RPM)、
% 展开电角度 (deg) 和电磁转矩 (N*m)。状态仅由本函数持有。
persistent id_z iq_z wm_z theta_e_z theta_unw theta_prev
c = foc.constants();
dt = c.DT;

if isempty(id_z)
    id_z = 0.0;
    iq_z = 0.0;
    wm_z = 0.0;
    theta_e_z = 0.0;
    theta_unw = 0.0;
    theta_prev = 0.0;
end

[valpha, vbeta] = foc.clarke(va, vb, vc);
[vd, vq] = foc.park(valpha, vbeta, theta_e_z);
we = wm_z * c.POLE_PAIRS;

did = (vd - c.R_PHASE * id_z + we * c.HFI_LQ * iq_z) / c.HFI_LD;
diq = (vq - c.R_PHASE * iq_z - we * (c.HFI_LD * id_z + c.FLUX)) / c.HFI_LQ;
id_z = id_z + did * dt;
iq_z = iq_z + diq * dt;

te = 1.5 * c.POLE_PAIRS * ...
    (c.FLUX * iq_z + (c.HFI_LD - c.HFI_LQ) * id_z * iq_z);
wm_z = wm_z + ((te - c.B_VISC * wm_z) / c.J) * dt;
theta_e_z = foc.wrap_pi(theta_e_z + wm_z * c.POLE_PAIRS * dt);

% 植物使用标准 dq→αβ；不能调用带有固件功率板负号的 foc.inverse_park。
cs = cos(theta_e_z);
sn = sin(theta_e_z);
ialpha = id_z * cs - iq_z * sn;
ibeta = id_z * sn + iq_z * cs;
[ia, ib, ic] = foc.inverse_clarke(ialpha, ibeta);
rpm = wm_z * 60.0 / (2.0 * pi);

dtheta = foc.wrap_pi(theta_e_z - theta_prev);
theta_unw = theta_unw + dtheta * 180.0 / pi;
theta_prev = theta_e_z;
th_deg = theta_unw;
end
