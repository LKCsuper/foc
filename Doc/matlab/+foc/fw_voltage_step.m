function [id, fw] = fw_voltage_step(fw, current_max, duty_start, voltage_target, gain_a_s, vd_req, vq_req, v_limit, dt)
%FW_VOLTAGE_STEP  用电压利用率积分调节负 Id。
% 输入 vd_req/vq_req 必须是矢量限幅前的电压请求，否则看不到饱和深度。
if current_max <= 0.0 || v_limit <= 0.0 || dt <= 0.0
    fw.id = 0.0;
    id = 0.0;
    return
end

duty_start = min(max(duty_start, 0.50), 0.99);
voltage_target = min(max(voltage_target, duty_start + 0.01), 0.99);
mod_req = hypot(vd_req, vq_req) / v_limit;

if mod_req >= duty_start || fw.id < -1.0e-6
    err = mod_req - voltage_target;
    if abs(err) < 0.005
        err = 0.0;
    end
    id = fw.id - gain_a_s * err * dt;
else
    id = fw.id;
end

id = min(max(id, -current_max), 0.0);
fw.id = id;
end
