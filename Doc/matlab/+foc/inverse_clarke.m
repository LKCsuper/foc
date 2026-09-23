function [ia, ib, ic] = inverse_clarke(ialpha, ibeta)
%INVERSE_CLARKE  αβ → 相量，对齐 motor_foc.c。
c = foc.constants();
ia = ialpha;
ib = ibeta * c.SQRT3_BY_2 - ialpha / 2.0;
ic = -ialpha / 2.0 - ibeta * c.SQRT3_BY_2;
end
