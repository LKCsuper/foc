function [va, vb, vc, rpm_cmd, th_hfi_deg, th_flux_deg, th_used_deg, rpm_used, id, iq] = sl_hfi_flux(ia, ib, ic, t)
%SL_HFI_FLUX  闭环：VESC 风格按 ERPM 回差选择 HFI 或磁链观测角。
persistent rpm_cmd_s spd_int vd_int vq_int duty_a duty_b duty_c ...
    valpha_z vbeta_z rpm_pi iq_ref_z th_used_unw th_used_prev seed_hold using_hfi hfi_inj_on ...
    fw vd_fw_z vq_fw_z
c = foc.constants();
dt = c.DT;
vlim = c.VBUS / sqrt(3.0) * c.MAX_DUTY;
align_s = c.HFI_FLUX_ALIGN_S;
if isempty(rpm_cmd_s)
    rpm_cmd_s = 0.0;
    spd_int = 0.0;
    vd_int = 0.0;
    vq_int = 0.0;
    mid = idivide(c.ARR, uint32(2));
    duty_a = mid;
    duty_b = mid;
    duty_c = mid;
    valpha_z = 0.0;
    vbeta_z = 0.0;
    rpm_pi = 0.0;
    iq_ref_z = 0.0;
    th_used_unw = 0.0;
    th_used_prev = 0.0;
    seed_hold = 0.0;
    using_hfi = true;
    hfi_inj_on = true;
    fw.id = 0.0;
    vd_fw_z = 0.0;
    vq_fw_z = 0.0;
end

[ia_s, ib_s, ic_s] = foc.reconstruct_longest_zero(ia, ib, ic, duty_a, duty_b, duty_c, c.ARR);
[ialpha, ibeta] = foc.clarke(ia_s, ib_s, ic_s);

hold_hfi = double(t < align_s);
[th_hfi_deg, rpm_hfi, va_inj, vb_inj, vc_inj] = sl_hfi(ialpha, ibeta, 0.0, hold_hfi);

rpm_to_flux = 0.0;
if t >= align_s && abs(rpm_hfi) > 150.0 && abs(rpm_hfi) < 700.0
    seed_hold = seed_hold + dt;
else
    seed_hold = 0.0;
end
if seed_hold > 0.05
    rpm_to_flux = rpm_hfi;
end
[th_flux_deg, rpm_flux, flux_ok] = sl_flux(valpha_z, vbeta_z, ialpha, ibeta, rpm_to_flux, th_hfi_deg);

id_ref = 0.0;
iq_cmd = 0.0;
iq_ref = 0.0;
th_used = 0.0;
w = 0.0;
rpm_cmd = 0.0;
rpm_used = 0.0;
id = 0.0;
iq = 0.0;

if t < align_s
    rpm_cmd_s = 0.0;
    spd_int = 0.0;
    id_ref = c.IQ_IF;
    iq_cmd = 0.0;
    th_used = 0.0;
    w = 0.0;
else
    id_ref = 0.0;
    if rpm_cmd_s < c.HFI_FLUX_TARGET
        rpm_cmd_s = rpm_cmd_s + c.HFI_FLUX_RAMP * dt;
        if rpm_cmd_s > c.HFI_FLUX_TARGET
            rpm_cmd_s = c.HFI_FLUX_TARGET;
        end
    end

    erpm_abs = abs(rpm_pi) * c.POLE_PAIRS;
    erpm_hyst = c.HFI_FLUX_SWITCH_ERPM * c.HFI_FLUX_SWITCH_HYST;
    if using_hfi
        if erpm_abs > c.HFI_FLUX_SWITCH_ERPM + erpm_hyst && flux_ok > 0.5
            using_hfi = false;
        end
    else
        if erpm_abs < c.HFI_FLUX_SWITCH_ERPM - erpm_hyst || flux_ok < 0.5
            using_hfi = true;
        end
    end

    w = double(~using_hfi);
    th_h = foc.wrap_pi(th_hfi_deg * pi / 180.0);
    th_f = foc.wrap_pi(th_flux_deg * pi / 180.0);
    % 凸极 IPMSM 在深弱磁时的剩余相位误差会把大幅负 Id 投影成假 Iq。
    % 按弱磁深度逐渐加入小角度滞后，零弱磁时不影响 HFI 交接。
    fw_ratio = min(abs(fw.id) / max(c.FW_CURRENT_MAX, 1.0e-6), 1.0);
    th_f = foc.wrap_pi(th_f - c.FW_PHASE_COMP_DEG * fw_ratio * pi / 180.0);
    th_used = foc.wrap_pi(th_h + w * foc.wrap_pi(th_f - th_h));
end
rpm_cmd = rpm_cmd_s;

rpm_raw = (1.0 - w) * rpm_hfi + w * rpm_flux;
if t < align_s
    rpm_raw = 0.0;
end
a_pi = dt * 2.0 * pi * 25.0;
if a_pi > 1.0
    a_pi = 1.0;
end
rpm_pi = rpm_pi + a_pi * (rpm_raw - rpm_pi);
rpm_used = rpm_pi;

if t >= align_s
    [iq_cmd, spd_int] = foc.current_pi(rpm_cmd, rpm_used, spd_int, ...
        c.HFI_FLUX_SPD_KP, c.HFI_FLUX_SPD_KI, dt, c.IQ_LIM);
else
    iq_cmd = 0.0;
end
diq_max = c.IQ_SLEW * dt;
if iq_cmd > iq_ref_z + diq_max
    iq_ref = iq_ref_z + diq_max;
elseif iq_cmd < iq_ref_z - diq_max
    iq_ref = iq_ref_z - diq_max;
else
    iq_ref = iq_cmd;
end
iq_ref_z = iq_ref;

% VESC 式弱磁：根据上一周期的电压利用率平滑增加负 Id。
% HFI 阶段不弱磁；进入磁链观测后才允许启动，避免低速注入电压误触发。
if t >= align_s && ~using_hfi
    [id_fw, fw] = foc.fw_voltage_step(fw, c.FW_CURRENT_MAX, c.FW_DUTY_START, ...
        c.FW_VOLTAGE_TARGET, c.FW_VOLTAGE_GAIN, vd_fw_z, vq_fw_z, vlim, dt);
    id_room = foc.fw_id_room(iq_ref, c.FW_CURRENT_BUDGET);
    id_ref = max(id_fw, -id_room);
    iq_room = sqrt(max(c.FW_CURRENT_BUDGET * c.FW_CURRENT_BUDGET - id_ref * id_ref, 0.0));
    iq_ref = min(max(iq_ref, -iq_room), iq_room);
else
    fw.id = 0.0;
    id_ref = max(id_ref, 0.0);
end

erpm_abs = abs(rpm_pi) * c.POLE_PAIRS;
if hfi_inj_on
    if erpm_abs > c.HFI_FLUX_SWITCH_ERPM * c.HFI_INJ_EXIT_FACTOR
        hfi_inj_on = false;
    end
else
    if erpm_abs < c.HFI_FLUX_SWITCH_ERPM * c.HFI_INJ_ENTER_FACTOR
        hfi_inj_on = true;
    end
end
inj = double(hfi_inj_on);
va_inj = va_inj * inj;
vb_inj = vb_inj * inj;
vc_inj = vc_inj * inj;

[id, iq] = foc.park(ialpha, ibeta, th_used);
[vd, vd_int] = foc.current_pi(id_ref, id, vd_int, c.KP, c.KI, dt, vlim);
[vq, vq_int] = foc.current_pi(iq_ref, iq, vq_int, c.KP, c.KI, dt, vlim);
% 弱磁要观察限幅前的电压请求，否则饱和后只能看到固定的 100%。
vd_fw_z = vd;
vq_fw_z = vq;
[vd, vq] = foc.saturate_vector_2d(vd, vq, vlim);

[valpha_if, vbeta_if] = foc.inverse_park(vd, vq, th_used);
% foc.inverse_park 内含固件功率板极性负号；离散数学植物前需抵消。
[va, vb, vc] = foc.inverse_clarke(-valpha_if, -vbeta_if);
va = va + va_inj;
vb = vb + vb_inj;
vc = vc + vc_inj;
valpha = -valpha_if;
vbeta = -vbeta_if;
[valpha_i, vbeta_i] = foc.clarke(va_inj, vb_inj, vc_inj);
valpha = valpha + valpha_i;
vbeta = vbeta + vbeta_i;
[duty_a, duty_b, duty_c] = foc.foc_svm_from_voltage(valpha, vbeta, c.VBUS, c.MAX_DUTY, c.ARR);
valpha_z = valpha;
vbeta_z = vbeta;

dth = foc.wrap_pi(th_used - th_used_prev);
th_used_unw = th_used_unw + dth * 180.0 / pi;
th_used_prev = th_used;
th_used_deg = th_used_unw;
end
