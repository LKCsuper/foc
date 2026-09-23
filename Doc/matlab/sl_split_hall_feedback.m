function [theta_hall_deg, rpm_hall] = sl_split_hall_feedback(theta_true_deg, rpm_true)
%SL_SPLIT_HALL_FEEDBACK  理想六步霍尔：输出扇区中心电角度和低通机械转速。
persistent rpm_f
c=foc.constants(); if isempty(rpm_f), rpm_f=0.0; end
wrapped=mod(theta_true_deg,360.0);
center=floor(wrapped/60.0)*60.0+30.0;
err=foc.wrap_pi((center-wrapped)*pi/180.0)*180.0/pi;
theta_hall_deg=theta_true_deg+err;
a=min(c.DT*2.0*pi*c.SIM_SENSOR_SPEED_LPF_HZ,1.0);
rpm_f=rpm_f+a*(rpm_true-rpm_f);
rpm_hall=rpm_f;
end
