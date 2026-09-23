function i_fw_cmd = sl_split_fw_duty_map(duty_filt, use_flux)
%SL_SPLIT_FW_DUTY_MAP  VESC式占空比到弱磁电流的线性映射。
c=foc.constants();
start=c.FW_DUTY_START*c.MAX_DUTY;
if use_flux<=0.5 || duty_filt<=start
    i_fw_cmd=0.0;
else
    x=(duty_filt-start)/max(c.MAX_DUTY-start,1.0e-6);
    i_fw_cmd=c.FW_CURRENT_MAX*min(max(x,0.0),1.0);
end
end
