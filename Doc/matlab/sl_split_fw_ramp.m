function id_fw = sl_split_fw_ramp(i_fw_safe)
%SL_SPLIT_FW_RAMP  按VESC foc_fw_ramp_time语义渐变弱磁电流。
persistent i_fw
c=foc.constants(); if isempty(i_fw), i_fw=0.0; end
step=c.FW_CURRENT_MAX*c.DT/max(c.FW_RAMP_TIME,c.DT);
i_fw=min(max(i_fw_safe,i_fw-step),i_fw+step);
id_fw=-i_fw;
end
