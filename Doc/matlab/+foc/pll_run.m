function [phase, speed] = pll_run(phase_in, dt, phase, speed, kp, ki, max_speed)
%PLL_RUN  对齐 foc_pll_run / Motor_Sensorless_RunPll：二阶锁相，滤 atan2 抖动。
delta = foc.wrap_pi(phase_in - phase);
phase = foc.wrap_pi(phase + (speed + kp * delta) * dt);
speed = speed + ki * delta * dt;
if speed > max_speed
    speed = max_speed;
elseif speed < -max_speed
    speed = -max_speed;
end
end
