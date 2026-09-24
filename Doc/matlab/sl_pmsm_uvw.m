function [ia, ib, ic, rpm, th_deg, te] = sl_pmsm_uvw(va, vb, vc)
%SL_PMSM_UVW  ATK-BL57H95D20E。UVW 电压用三电阻 Clarke 进 αβ，电流用逆 Clarke 出。
persistent ial ibe th om th_unw
c = foc.constants();
dt = c.DT;
R = c.R_LINE;
L = c.L_LINE;
flux = c.FLUX;
pp = c.POLE_PAIRS;
J = c.J;
B = c.B_VISC;
if isempty(ial)
    ial = 0.0;
    ibe = 0.0;
    th = 0.0;
    om = 0.0;
    th_unw = 0.0;
end
[val, vbe] = foc.clarke(va, vb, vc);
[~, iqr] = foc.park(ial, ibe, th);
te = 1.5 * pp * flux * iqr;
we = om * pp;
ial = ial + dt / L * (val - R * ial + we * flux * sin(th));
ibe = ibe + dt / L * (vbe - R * ibe - we * flux * cos(th));
om = om + dt * (te - B * om) / J;
dth = dt * om * pp;
th = foc.wrap_pi(th + dth);
th_unw = th_unw + dth * 180.0 / pi;
[ia, ib, ic] = foc.inverse_clarke(ial, ibe);
rpm = om * 60.0 / (2.0 * pi);
th_deg = th_unw;
end
