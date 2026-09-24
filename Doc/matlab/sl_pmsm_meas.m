function [ia, ib, ic, rpm, th_deg, te] = sl_pmsm_meas(ia, ib, ic, wm, thm)
%SL_PMSM_MEAS  库电机传感器：相电流、机械转速、电角展开、电磁转矩。
persistent th_unw th_prev
c = foc.constants();
if isempty(th_unw)
    th_unw = 0.0;
    th_prev = 0.0;
end
the = thm * c.POLE_PAIRS;
dth = mod(the - th_prev + pi, 2.0 * pi) - pi;
th_unw = th_unw + dth * 180.0 / pi;
th_prev = the;
[ialpha, ibeta] = foc.clarke(ia, ib, ic);
[~, iqr] = foc.park(ialpha, ibeta, the);
te = 1.5 * c.POLE_PAIRS * c.FLUX * iqr;
rpm = wm * 60.0 / (2.0 * pi);
th_deg = th_unw;
end
