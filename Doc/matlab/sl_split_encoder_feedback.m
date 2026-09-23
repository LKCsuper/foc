function [theta_enc_deg, rpm_enc] = sl_split_encoder_feedback(theta_true_deg, rpm_true)
%SL_SPLIT_ENCODER_FEEDBACK  理想增量编码器：电角度按CPR量化，速度一阶低通。
persistent rpm_f
c=foc.constants(); if isempty(rpm_f), rpm_f=0.0; end
step_deg=360.0*c.POLE_PAIRS/max(c.SIM_ENCODER_CPR,1.0);
theta_enc_deg=round(theta_true_deg/step_deg)*step_deg;
a=min(c.DT*2.0*pi*c.SIM_SENSOR_SPEED_LPF_HZ,1.0);
rpm_f=rpm_f+a*(rpm_true-rpm_f);
rpm_enc=rpm_f;
end
