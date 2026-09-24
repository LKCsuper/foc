function [va, vb, vc, valpha, vbeta, ialpha, ibeta, rpm_if, th_if_deg, id, iq] = sl_hfi_if(ia, ib, ic, t)
%SL_HFI_IF  低速 I/F：72 RPM 斜坡，Clarke/Park/PI 与 sl_if_ctrl 相同。
%
%  72 RPM、2 对极时 ωeλ≈0.4 V，反电势观测器不可用。本块只拖转子；
%  注入和跟踪在 sl_hfi。
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
[va, vb, vc] = foc.inverse_clarke(-valpha_if, -vbeta_if);
[duty_a, duty_b, duty_c] = foc.foc_svm_from_voltage(-valpha_if, -vbeta_if, c.VBUS, c.MAX_DUTY, c.ARR);
valpha = -valpha_if;
vbeta = -vbeta_if;

if t >= 0.04 && rpm_if_s < c.HFI_RPM
    rpm_if_s = rpm_if_s + c.HFI_RAMP * dt;
    if rpm_if_s > c.HFI_RPM
        rpm_if_s = c.HFI_RPM;
    end
end
dth = rpm_if_s / 60.0 * 2.0 * pi * c.POLE_PAIRS * dt;
th_if = foc.wrap_pi(th_if + dth);
th_unw = th_unw + dth * 180.0 / pi;
rpm_if = rpm_if_s;
th_if_deg = th_unw;
end
