function [ia, ib, ic, hit] = reconstruct_longest_zero(ia, ib, ic, duty_a, duty_b, duty_c, pwm_period)
%RECONSTRUCT_LONGEST_ZERO  VESC 最长零矢量，门槛 ARR-500。对齐 motor_current.c。
c = foc.constants();
hit = 0;
if pwm_period <= c.SHUNT_PICK
    return
end
limit = double(pwm_period) - c.SHUNT_PICK;
if duty_a <= limit && duty_b <= limit && duty_c <= limit
    return
end
if duty_a > duty_b && duty_a > duty_c
    ia = -(ib + ic);
    hit = 1;
elseif duty_b > duty_a && duty_b > duty_c
    ib = -(ia + ic);
    hit = 2;
elseif duty_c > duty_a && duty_c > duty_b
    ic = -(ia + ib);
    hit = 3;
end
end
