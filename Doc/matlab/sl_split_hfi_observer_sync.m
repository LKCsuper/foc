function [th_sync_deg, rpm_sync, err_deg, offset_out] = sl_split_hfi_observer_sync(th_hfi_deg, rpm_hfi, th_flux_deg, rpm_flux, flux_ok, align_on, enable)
%SL_SPLIT_HFI_OBSERVER_SYNC  切换前学习HFI/磁链角差，切换后平滑释放偏置。
persistent offset_deg
c=foc.constants(); dt=c.DT;
if isempty(offset_deg), offset_deg=0.0; end
raw_err=foc.wrap_pi((th_hfi_deg-th_flux_deg)*pi/180.0)*180.0/pi;
erpm=abs(rpm_hfi)*c.POLE_PAIRS;
sync_limit=c.HFI_FLUX_SWITCH_ERPM*(1.0+c.HFI_FLUX_SWITCH_HYST);
if enable<=0.5
    offset_deg=0.0;
elseif align_on>0.5
    offset_deg=0.0;
elseif flux_ok>0.5 && erpm<=sync_limit
    a=min(dt/max(c.HFI_SYNC_TAU,dt),1.0);
    offset_deg=offset_deg+a*(raw_err-offset_deg);
else
    a=min(dt/max(c.HFI_SYNC_RELEASE_TAU,dt),1.0);
    offset_deg=offset_deg+a*(0.0-offset_deg);
end
th_sync_deg=th_flux_deg+offset_deg;
rpm_sync=rpm_flux;
err_deg=raw_err;
offset_out=offset_deg;
end
