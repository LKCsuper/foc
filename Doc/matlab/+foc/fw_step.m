function [id, fw] = fw_step(fw, current_max, duty_start, ramp_a_s, vd, vq, v_limit, dt)
%FW_STEP  按 |V|/Vlim 给出 VESC 式负 Id，再做电流斜坡。
if current_max <= 0.0 || v_limit <= 0.0
    fw.id = 0.0;
    id = 0.0;
    return
end
mod = hypot(vd, vq) / v_limit;
if duty_start < 0.50
    duty_start = 0.50;
end
if duty_start > 0.99
    duty_start = 0.99;
end
if mod <= duty_start
    target = 0.0;
else
    % VESC foc_run_fw：从 duty_start 到最大调制度线性映射 0..I_fw_max。
    fw_frac = (mod - duty_start) / (1.0 - duty_start);
    fw_frac = min(max(fw_frac, 0.0), 1.0);
    target = -current_max * fw_frac;
end
id = fw.id;
if ramp_a_s <= 0.0 || dt <= 0.0
    id = target;
else
    step = ramp_a_s * dt;
    if id < target
        id = id + step;
        if id > target
            id = target;
        end
    elseif id > target
        id = id - step;
        if id < target
            id = target;
        end
    end
end
if id > 0.0
    id = 0.0;
end
if id < -current_max
    id = -current_max;
end
fw.id = id;
end
