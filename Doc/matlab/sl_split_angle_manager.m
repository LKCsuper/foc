function [theta_deg, rpm_used, use_flux, inj_on] = sl_split_angle_manager(th_hfi_deg, rpm_hfi, th_flux_deg, rpm_flux, flux_ok, align_on, fw_id)
%SL_SPLIT_ANGLE_MANAGER  VESC式HFI/磁链选源、回差和注入门控。
persistent using_hfi hfi_inj rpm_f th_unw th_prev
c = foc.constants(); dt = c.DT;
if isempty(using_hfi)
    using_hfi = true; hfi_inj = true; rpm_f = 0.0;
    th_unw = 0.0; th_prev = 0.0;
end
erpm = abs(rpm_f) * c.POLE_PAIRS;
hyst = c.HFI_FLUX_SWITCH_ERPM * c.HFI_FLUX_SWITCH_HYST;
if using_hfi
    if erpm > c.HFI_FLUX_SWITCH_ERPM + hyst && flux_ok > 0.5, using_hfi = false; end
elseif erpm < c.HFI_FLUX_SWITCH_ERPM - hyst || flux_ok < 0.5
    using_hfi = true;
end
if hfi_inj
    if erpm > c.HFI_FLUX_SWITCH_ERPM*c.HFI_INJ_EXIT_FACTOR, hfi_inj = false; end
elseif erpm < c.HFI_FLUX_SWITCH_ERPM*c.HFI_INJ_ENTER_FACTOR
    hfi_inj = true;
end
th = foc.wrap_pi(th_hfi_deg*pi/180.0);
rpm_raw = rpm_hfi;
if ~using_hfi
    ratio = min(abs(fw_id)/max(c.FW_CURRENT_MAX,1e-6),1.0);
    th = foc.wrap_pi(th_flux_deg*pi/180.0-c.FW_PHASE_COMP_DEG*ratio*pi/180.0);
    rpm_raw = rpm_flux;
end
if align_on > 0.5, th = 0.0; rpm_raw = 0.0; end
a = min(dt*2*pi*25.0,1.0); rpm_f = rpm_f+a*(rpm_raw-rpm_f);
dth = foc.wrap_pi(th-th_prev); th_unw = th_unw+dth*180/pi; th_prev = th;
theta_deg = th_unw; rpm_used = rpm_f;
use_flux = double(~using_hfi); inj_on = double(hfi_inj);
end
