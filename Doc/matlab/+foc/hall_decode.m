function theta = hall_decode(table, hall_state)
%HALL_DECODE  霍尔表 → 电角，对齐 Motor_Hall_DecodeAngle。
%  table 长度 8，下标 0..7；255 无效。
if hall_state < 0 || hall_state > 7
    theta = NaN;
    return
end
raw = double(table(hall_state + 1));
if raw == 255 || raw > 200
    theta = NaN;
    return
end
theta = foc.wrap_pi(raw / 200.0 * 2.0 * pi);
end
