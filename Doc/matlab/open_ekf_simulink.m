function open_ekf_simulink()
%OPEN_EKF_SIMULINK  打开 MATLAB 库 PMSM（a/b/c 端子）+ 示波器的 EKF 模型。
%
%  必须在桌面 MATLAB 里运行（不要用 -batch）：
%    cd F:\code\foc\Doc\matlab
%    open_ekf_simulink
%  画布中间应是 Simscape Electrical 的 PMSM 电机图标，三相端子 a/b/c。
%  点绿色运行。示波器：相电流 U/V/W、转速、电角、转矩。

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
try
    load_system('simulink');
catch me
    error('open_ekf_simulink:noSL', ...
          '这台 MATLAB 没有 Simulink（%s）。图形电机+示波器需要 Simulink；脚本请运行 sim_ekf_motor。', me.message);
end
slx = fullfile(this_dir, 'ekf_pmsm.slx');
build_ekf_simulink();
open_system(slx);
end
