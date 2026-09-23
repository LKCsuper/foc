function [id_ref, iq_ref_limited] = sl_split_current_circle(id_raw, iq_raw)
%SL_SPLIT_CURRENT_CIRCLE  实现 Id^2+Iq^2<=Imax^2，优先保留弱磁Id。
c=foc.constants();
id_ref=min(max(id_raw,-c.FW_CURRENT_BUDGET),c.FW_CURRENT_BUDGET);
iq_room=sqrt(max(c.FW_CURRENT_BUDGET^2-id_ref^2,0.0));
iq_ref_limited=min(max(iq_raw,-iq_room),iq_room);
end
