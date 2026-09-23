function duty_filt = sl_split_duty_lpf(duty)
%SL_SPLIT_DUTY_LPF  弱磁使用的占空比低通，避免PWM/观测器纹波触发弱磁。
persistent y
c=foc.constants(); if isempty(y), y=0.0; end
y=y+c.FW_DUTY_LPF*(duty-y);
duty_filt=y;
end
