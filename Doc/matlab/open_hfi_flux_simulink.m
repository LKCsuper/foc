function open_hfi_flux_simulink()
%OPEN_HFI_FLUX_SIMULINK  闭环 HFI + Ortega 磁链交接（Park 用估计角）。
%
%  必须在桌面 MATLAB 里运行（不要用 -batch）：
%    cd F:\code\foc\Doc\matlab
%    open_hfi_flux_simulink
%  自动验收（会打印 PASS/FAIL）：
%    verify_hfi_flux_simulink
%  青块 HFI_FLUX：零速 HFI 锁角并注入，磁链观测器始终后台运行；仿照 VESC
%  按 3000 ERPM、±5%% 回差选择 HFI/磁链角，注入另用 1.5/1.8 倍门限回差。
%  磁链观测阶段按电压利用率加入 VESC 式负 Id 弱磁，24 V 目标 3000 RPM。
%  Scope_IdIq 中黄色第一路为 Id，蓝色第二路为 Iq；高速时 Id 应明显为负。
%  黄=Park角/转速给定，蓝=转子，橙=磁链。
%  植物为显式 dq 方程的离散 PMSM，使用 Ld≠Lq，不依赖 Simscape 端口方向。
%  磁链观测器使用每相 R，并按 VESC 公式用 dq 电流动态补偿 Ld/Lq 凸极性。
%  深弱磁时再按 |Id_fw| 比例加入最大 4° 相位补偿，避免负 Id 投影成假 Iq。
%  未接入固件 ISR，Hall / slenable 不变。

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
try
    load_system('simulink');
catch me
    error('open_hfi_flux_simulink:noSL', ...
          '这台 MATLAB 没有 Simulink（%s）。图形电机+示波器需要 Simulink。', me.message);
end
slx = fullfile(this_dir, 'hfi_flux_pmsm.slx');
build_hfi_flux_simulink();
open_system(slx);
end
