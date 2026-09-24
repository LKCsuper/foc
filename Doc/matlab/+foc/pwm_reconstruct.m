function [valpha, vbeta, vmag] = pwm_reconstruct(tA, tB, tC, period, vbus)
%PWM_RECONSTRUCT  由 CCR 还原相电压再做 Clarke，对应固件 PWM ideal。
da = double(tA) / double(period);
db = double(tB) / double(period);
dc = double(tC) / double(period);
ua = (2.0 * da - db - dc) / 3.0 * vbus;
ub = (2.0 * db - da - dc) / 3.0 * vbus;
uc = (2.0 * dc - da - db) / 3.0 * vbus;
[valpha, vbeta] = foc.clarke(ua, ub, uc);
vmag = hypot(valpha, vbeta);
end
