function open_flux_simulink()
%OPEN_FLUX_SIMULINK  打开 MATLAB 库 PMSM（a/b/c 端子）+ 示波器的 Ortega 磁链模型。
%
%  必须在桌面 MATLAB 里运行（不要用 -batch）：
%    cd F:\code\foc\Doc\matlab
%    open_flux_simulink
%  画布中间应是 Simscape Electrical 的 PMSM 电机图标，三相端子 a/b/c。
%  点绿色运行。示波器：相电流 U/V/W、转速、电角、转矩。
%  Mux 第三路是磁链观测 + PLL（青块）。

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
try
    load_system('simulink');
catch me
    error('open_flux_simulink:noSL', ...
          '这台 MATLAB 没有 Simulink（%s）。图形电机+示波器需要 Simulink。', me.message);
end
slx = fullfile(this_dir, 'flux_pmsm.slx');
build_flux_simulink();
open_system(slx);
end
