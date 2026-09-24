function varargout = sim_ekf_motor()
%SIM_EKF_MOTOR  用 ATK-BL57H95D20E 铭牌做空载 I/F + 影子 EKF 时域仿真。
%
%  桌面：cd Doc/matlab，输入 sim_ekf_motor
%  图：弹出并写入 out/ekf_motor_sim.png

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
out_dir = fullfile(this_dir, 'out');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end
log_path = fullfile(out_dir, 'sim_ekf_motor.log');
fid = fopen(log_path, 'w');

c = foc.constants();
fprintf('ATK-BL57H95D20E  plant  R=%.3fΩ  L=%.1f mH  ψf=%.5f Wb  J=%.2e kg·m²\n', ...
        c.R_LINE, c.L_LINE * 1e3, c.FLUX, c.J);
fprintf(fid, 'ATK-BL57H95D20E  R=%.3f L=%.4f flux=%.5f J=%.2e\n', ...
        c.R_LINE, c.L_LINE, c.FLUX, c.J);

m = foc.ekf_motor_sim();
line = sprintf('done  true=%.1f RPM  IF=%.1f  EKF=%.1f  |dth|=%.2f deg  max|i|=%.2f A  speed_ok=%d  ekf_ok=%d', ...
               m.final_rpm, m.rpm_if(end), m.rpm_ekf(end), m.final_ekf_err * 180.0 / pi, ...
               m.max_i, m.speed_ok_last, m.ok);
fprintf('%s\n', line);
fprintf(fid, '%s\n', line);

env_plot = getenv('FOC_MATLAB_PLOT');
if isempty(env_plot)
    do_plot = true;
else
    do_plot = ~strcmp(env_plot, '0');
end
if do_plot
    foc.ekf_motor_plot(m, out_dir, 'ekf_motor_sim.png');
    fprintf('PNG %s\n', fullfile(out_dir, 'ekf_motor_sim.png'));
    fprintf(fid, 'PNG %s\n', fullfile(out_dir, 'ekf_motor_sim.png'));
end
fclose(fid);
if nargout > 0
    varargout{1} = m;
end
end
