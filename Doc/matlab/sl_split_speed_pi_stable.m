function iq_ref = sl_split_speed_pi_stable(rpm_cmd, rpm_used, align_on)
%SL_SPLIT_SPEED_PI_STABLE  与整体稳定版一致的并行PI、钳位和Iq限斜率。
persistent vint iq_z
c=foc.constants();
if isempty(vint), vint=0.0; iq_z=0.0; end
if align_on>0.5
    vint=0.0;
    iq_cmd=0.0;
else
    [iq_cmd,vint]=foc.current_pi(rpm_cmd,rpm_used,vint, ...
        c.HFI_FLUX_SPD_KP,c.HFI_FLUX_SPD_KI,c.DT,c.IQ_LIM);
end
step=c.IQ_SLEW*c.DT;
iq_ref=min(max(iq_cmd,iq_z-step),iq_z+step);
iq_z=iq_ref;
end
