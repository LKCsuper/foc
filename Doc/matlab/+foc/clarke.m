function [ialpha, ibeta] = clarke(ia, ib, ic)
%CLARKE  三电阻幅值不变 Clarke，对齐 motor_foc.c MOTOR_FOC_THREE_SHUNTS=1。
c = foc.constants();
ialpha = (2.0 / 3.0) * ia - (1.0 / 3.0) * ib - (1.0 / 3.0) * ic;
ibeta = (ib - ic) * c.ONE_BY_SQRT3;
end
