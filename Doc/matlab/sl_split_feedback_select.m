function [theta_deg, rpm, source, fallback, sensored_weight] = sl_split_feedback_select( ...
    th_sensorless, rpm_sensorless, th_enc, rpm_enc, th_hall, rpm_hall, ...
    enc_enable, hall_enable, enc_fault, hall_fault, align_on, enc_mode)
%SL_SPLIT_FEEDBACK_SELECT  编码器模式与故障回退。enc_mode<0.5 纯编码器，>=0.5 低速编码器/高速无感。
persistent valid_s selected_type using_encoder
c=foc.constants();
if isempty(valid_s), valid_s=0.0; selected_type=0.0; using_encoder=true; end

requested=0.0;
sensor_valid=false;
th_s=th_sensorless; rpm_s=rpm_sensorless;
if enc_enable>0.5
    requested=2.0; th_s=th_enc; rpm_s=rpm_enc;
    sensor_valid=enc_fault<=0.5 && isfinite(th_enc) && isfinite(rpm_enc);
elseif hall_enable>0.5
    requested=1.0; th_s=th_hall; rpm_s=rpm_hall;
    sensor_valid=hall_fault<=0.5 && isfinite(th_hall) && isfinite(rpm_hall);
end

if requested~=selected_type
    valid_s=0.0;
    selected_type=requested;
end
if requested>0.5 && sensor_valid
    valid_s=min(valid_s+c.DT,c.SIM_SENSOR_RECOVER_S);
else
    valid_s=0.0;
end
sensor_ready=requested>0.5 && sensor_valid && valid_s>=c.SIM_SENSOR_RECOVER_S;

erpm=abs(rpm_sensorless)*c.POLE_PAIRS;
w=0.0;
if requested==2.0 && enc_mode<0.5
    using_encoder=true;
    w=1.0;
elseif requested==2.0
    hyst=0.05*c.SIM_SENSORED_ERPM_END;
    if using_encoder && erpm>c.SIM_SENSORED_ERPM_END+hyst
        using_encoder=false;
    elseif ~using_encoder && erpm<c.SIM_SENSORED_ERPM_END-hyst
        using_encoder=true;
    end
    w=double(using_encoder);
elseif requested==1.0
    if erpm<=c.SIM_SENSORED_ERPM_START
        w=1.0;
    elseif erpm>=c.SIM_SENSORED_ERPM_END
        w=0.0;
    else
        w=(c.SIM_SENSORED_ERPM_END-erpm)/ ...
            (c.SIM_SENSORED_ERPM_END-c.SIM_SENSORED_ERPM_START);
    end
else
    using_encoder=true;
end

fallback=0.0;
if align_on>0.5
    theta_deg=0.0; rpm=0.0; source=0.0; sensored_weight=0.0;
elseif ~sensor_ready
    theta_deg=th_sensorless; rpm=rpm_sensorless; sensored_weight=0.0;
    if requested>0.5
        source=5.0; fallback=1.0;
    else
        source=0.0;
    end
else
    diff=foc.wrap_pi((th_s-th_sensorless)*pi/180.0)*180.0/pi;
    theta_deg=th_sensorless+w*diff;
    rpm=(1.0-w)*rpm_sensorless+w*rpm_s;
    sensored_weight=w;
    if w>=0.999
        source=requested;
    elseif w<=0.001
        source=0.0;
    else
        source=requested+2.0;
    end
end
end
