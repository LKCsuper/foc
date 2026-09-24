function open_hfi_simulink()
%OPEN_HFI_SIMULINK  打开低速脉振 HFI 库电机模型（凸极 + 72 RPM I/F + 示波器）。
%
%  必须在桌面 MATLAB 里运行（不要用 -batch）：
%    cd F:\code\foc\Doc\matlab
%    open_hfi_simulink
%  画布中间应是 Simscape Electrical 的 PMSM 电机图标，三相端子 a/b/c。
%  点绿色运行。示波器：相电流 U/V/W、转速、电角、转矩。
%  Mux 第三路是 HFI 跟踪角/转速（青块）。植物用了 Ld≠Lq，铭牌电机没有分电感。

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
try
    load_system('simulink');
catch me
    error('open_hfi_simulink:noSL', ...
          '这台 MATLAB 没有 Simulink（%s）。图形电机+示波器需要 Simulink。', me.message);
end
slx = fullfile(this_dir, 'hfi_pmsm.slx');
build_hfi_simulink();
open_system(slx);
end
