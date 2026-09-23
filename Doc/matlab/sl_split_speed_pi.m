function iq_ref = sl_split_speed_pi(rpm_cmd, rpm_used, align_on)
%SL_SPLIT_SPEED_PI  VESC式速度PID：斜坡输入、滤波D、限幅和回算抗饱和。
persistent vint iq_z prev_err d_filt
c=foc.constants();
if isempty(vint), vint=0.0; iq_z=0.0; prev_err=0.0; d_filt=0.0; end
% MATLAB Function/Coder 要求所有执行路径上的局部量都先完整定义。
% 对齐分支不会使用候选积分和未限幅命令，但仍给它们标量默认值。
vint_candidate=vint;
cmd_raw=0.0;
cmd_lim=0.0;
if align_on>0.5
    vint=0.0; prev_err=0.0; d_filt=0.0; cmd_lim=0.0;
else
    err=rpm_cmd-rpm_used;
    d_raw=c.HFI_FLUX_SPD_KD*(err-prev_err)/c.DT;
    d_filt=d_filt+c.HFI_FLUX_SPD_D_FILTER*(d_raw-d_filt);
    prev_err=err;
    % 先算未限幅PI，再限幅。积分器稍后根据“实际施加的Iq”回算，
    % 因此弱磁未建立、Iq限幅或斜率受限时不会继续积累过量积分。
    vint_candidate=vint+c.HFI_FLUX_SPD_KI*err*c.DT;
    cmd_raw=c.HFI_FLUX_SPD_KP*err+vint_candidate+d_filt;
    cmd_lim=min(max(cmd_raw,-c.IQ_LIM),c.IQ_LIM);
end
step=c.IQ_SLEW*c.DT;
iq_ref=min(max(cmd_lim,iq_z-step),iq_z+step);
if align_on<=0.5
    % Back calculation：让积分状态跟随限幅和斜率之后真正送出的Iq。
    vint=vint_candidate+c.HFI_FLUX_SPD_AW*(iq_ref-cmd_raw)*c.DT;
    vint=min(max(vint,-c.IQ_LIM),c.IQ_LIM);
end
iq_z=iq_ref;
end
