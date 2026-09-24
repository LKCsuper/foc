function [va, vb, vc, valpha, vbeta, ialpha, ibeta, rpm_if, th_if_deg, id, iq] = sl_if_ctrl(ia, ib, ic, t)
%SL_IF_CTRL  电流环：最长零矢量重构 → Clarke → Park → PI → 逆 Park。
%
%  采样与 motor_current.c / motor_foc.c 一致。库电机没有 HD75 的 SVM(-v)+A-C-B，
%  端电压和观测器都用 inverse_park 再取反后的 αβ，保证和相电流同一极性。
persistent th_if rpm_if_s vd_int vq_int th_unw duty_a duty_b duty_c
c = foc.constants();
dt = c.DT;
vlim = c.VBUS / sqrt(3.0) * c.MAX_DUTY;
if isempty(th_if)
    th_if = 0.0;
    rpm_if_s = 0.0;
    vd_int = 0.0;
    vq_int = 0.0;
    th_unw = 0.0;
    mid = idivide(c.ARR, uint32(2));
    duty_a = mid;
    duty_b = mid;
    duty_c = mid;
end

[ia_s, ib_s, ic_s] = foc.reconstruct_longest_zero(ia, ib, ic, duty_a, duty_b, duty_c, c.ARR);
[ialpha, ibeta] = foc.clarke(ia_s, ib_s, ic_s);
[id, iq] = foc.park(ialpha, ibeta, th_if);

id_ref = c.IQ_IF;
iq_ref = 0.0;
[vd, vd_int] = foc.current_pi(id_ref, id, vd_int, c.KP, c.KI, dt, vlim);
[vq, vq_int] = foc.current_pi(iq_ref, iq, vq_int, c.KP, c.KI, dt, vlim);
[vd, vq] = foc.saturate_vector_2d(vd, vq, vlim);

[valpha_if, vbeta_if] = foc.inverse_park(vd, vq, th_if);
% 库 PMSM 没有 HD75 的 SVM(-v) 和 A-C-B。端电压把 inverse_park 的 180° 抵掉，
% 电机吃教科书相电压。EKF/SMO 必须用同一套 αβ，否则转速会反号。
[va, vb, vc] = foc.inverse_clarke(-valpha_if, -vbeta_if);
[duty_a, duty_b, duty_c] = foc.foc_svm_from_voltage(-valpha_if, -vbeta_if, c.VBUS, c.MAX_DUTY, c.ARR);
valpha = -valpha_if;
vbeta = -vbeta_if;

if t >= 0.04 && rpm_if_s < c.SL_TRANS_RPM
    rpm_if_s = rpm_if_s + 1200.0 * dt;
    if rpm_if_s > c.SL_TRANS_RPM
        rpm_if_s = c.SL_TRANS_RPM;
    end
end
dth = rpm_if_s / 60.0 * 2.0 * pi * c.POLE_PAIRS * dt;
th_if = foc.wrap_pi(th_if + dth);
th_unw = th_unw + dth * 180.0 / pi;
rpm_if = rpm_if_s;
th_if_deg = th_unw;
end
