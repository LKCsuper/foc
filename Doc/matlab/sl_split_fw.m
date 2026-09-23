function id_fw = sl_split_fw(vd_req_z, vq_req_z, iq_ref, use_flux, vbus)
%SL_SPLIT_FW  独立弱磁电压积分器与电流圆限制。
persistent fw
c=foc.constants(); if isempty(fw), fw.id=0.0; end
vlim=max(vbus,0.1)/sqrt(3)*c.MAX_DUTY;
if use_flux>0.5
    [id_fw,fw]=foc.fw_voltage_step(fw,c.FW_CURRENT_MAX,c.FW_DUTY_START,c.FW_VOLTAGE_TARGET,c.FW_VOLTAGE_GAIN,vd_req_z,vq_req_z,vlim,c.DT);
    id_fw=max(id_fw,-foc.fw_id_room(iq_ref,c.FW_CURRENT_BUDGET));
else
    fw.id=0.0; id_fw=0.0;
end
end
