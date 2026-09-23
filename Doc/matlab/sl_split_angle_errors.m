function [hfi_err_deg, flux_err_deg, sync_err_deg] = sl_split_angle_errors(th_hfi_deg, th_flux_deg, th_sync_deg, th_true_deg)
%SL_SPLIT_ANGLE_ERRORS  三路估算角相对PMSM真实电角度的包角误差，仅用于仿真诊断。
hfi_err_deg=foc.wrap_pi((th_hfi_deg-th_true_deg)*pi/180.0)*180.0/pi;
flux_err_deg=foc.wrap_pi((th_flux_deg-th_true_deg)*pi/180.0)*180.0/pi;
sync_err_deg=foc.wrap_pi((th_sync_deg-th_true_deg)*pi/180.0)*180.0/pi;
end
