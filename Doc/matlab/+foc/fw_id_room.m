function room = fw_id_room(iq, i_budget)
%FW_ID_ROOM  电流圆内优先留 Iq。对齐 motor_fw.c。
if i_budget <= 0.0
    room = 0.0;
    return
end
iq_abs = abs(iq);
if iq_abs >= i_budget
    room = 0.0;
    return
end
room = sqrt(i_budget * i_budget - iq_abs * iq_abs);
end
