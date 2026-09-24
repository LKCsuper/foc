function obs = ekf_reset(obs)
%EKF_RESET  对齐 Motor_Ekf_Reset。
if nargin < 1 || isempty(obs)
    obs = struct();
end
obs.initialized = false;
obs.x = zeros(4, 1);
obs.p = zeros(4, 4);
obs.angle = 0.0;
obs.speed_e = 0.0;
end
