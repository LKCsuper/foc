function [id, iq] = park(ialpha, ibeta, theta)
%PARK  αβ → dq，对齐 motor_foc.c。
s = sin(theta);
c = cos(theta);
id = ialpha * c + ibeta * s;
iq = -ialpha * s + ibeta * c;
end
