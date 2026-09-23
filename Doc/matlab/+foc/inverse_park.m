function [valpha, vbeta] = inverse_park(vd, vq, theta)
%INVERSE_PARK  dq → αβ。C 侧多乘一次负号（3e5d564 功率板极性）。
s = sin(theta);
c = cos(theta);
valpha = -(vd * c - vq * s);
vbeta = -(vd * s + vq * c);
end
