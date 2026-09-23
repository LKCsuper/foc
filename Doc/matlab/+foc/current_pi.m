function [v_out, v_int] = current_pi(target, measured, v_int, kp, ki, dt, v_limit)
%CURRENT_PI  VESC 并行 PI + clamping，对齐 motor_pid.c。
ierr = target - measured;
v_int = v_int + ierr * ki * dt;
v_out = v_int + ierr * kp;
if v_out > v_limit
    v_out = v_limit;
    v_int = v_limit - ierr * kp;
elseif v_out < -v_limit
    v_out = -v_limit;
    v_int = -v_limit - ierr * kp;
end
end
