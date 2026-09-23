function result = verify_hfi_flux_simulink()
%VERIFY_HFI_FLUX_SIMULINK  运行 HFI+磁链交接模型并自动检查关键结果。
%
%  模型只在内存中增加记录块，不保存验证用连线。返回结构体并打印 PASS/FAIL。

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
build_hfi_flux_simulink();

mdl = 'hfi_flux_pmsm';
load_system(mdl);
cleanup = onCleanup(@() close_system(mdl, 0)); %#ok<NASGU>

signals = { ...
    'HFI_FLUX/4',  'rpm_cmd'; ...
    'PMSM_Discrete/4', 'rpm_motor'; ...
    'HFI_FLUX/8',  'rpm_used'; ...
    'HFI_FLUX/6',  'theta_flux'; ...
    'HFI_FLUX/7',  'theta_used'; ...
    'PMSM_Discrete/5', 'theta_motor'; ...
    'HFI_FLUX/9',  'id'; ...
    'HFI_FLUX/10', 'iq'};

for i = 1:size(signals, 1)
    name = signals{i, 2};
    block = sprintf('%s/verify_%s', mdl, name);
    add_block('simulink/Sinks/To Workspace', block, ...
        'VariableName', name, 'SaveFormat', 'Timeseries', ...
        'Position', [1100, 30 + 35 * i, 1200, 50 + 35 * i]);
    add_line(mdl, signals{i, 1}, sprintf('verify_%s/1', name), 'autorouting', 'on');
end

out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
rpm_cmd = out.get('rpm_cmd');
rpm_motor = out.get('rpm_motor');
rpm_used = out.get('rpm_used');
theta_flux = out.get('theta_flux');
theta_used = out.get('theta_used');
theta_motor = out.get('theta_motor');
id = out.get('id');
iq = out.get('iq');

t = rpm_motor.Time;
tail = t >= max(t(end) - 0.50, 0.0);
handoff = t >= 1.05 & t <= 1.35;
speed_error = rpm_motor.Data - rpm_cmd.Data;
angle_error = wrap_deg(theta_used.Data - theta_motor.Data);
flux_error = wrap_deg(theta_flux.Data - theta_motor.Data);

result.final_rpm_cmd = rpm_cmd.Data(end);
result.final_rpm_motor = rpm_motor.Data(end);
result.final_rpm_used = rpm_used.Data(end);
result.tail_rpm_error_mean = mean(speed_error(tail));
result.tail_rpm_error_max_abs = max(abs(speed_error(tail)));
result.tail_angle_error_mean_abs_deg = mean(abs(angle_error(tail)));
result.tail_angle_error_max_abs_deg = max(abs(angle_error(tail)));
result.tail_flux_error_mean_abs_deg = mean(abs(flux_error(tail)));
result.handoff_angle_step_max_deg = max(abs(diff(theta_used.Data(handoff))));
result.tail_id_mean_abs = mean(abs(id.Data(tail)));
result.tail_iq_mean = mean(iq.Data(tail));
result.finite = all(isfinite([rpm_motor.Data; rpm_used.Data; theta_used.Data; id.Data; iq.Data]));

result.pass = result.finite && ...
    abs(result.final_rpm_motor - result.final_rpm_cmd) <= 50.0 && ...
    result.tail_angle_error_mean_abs_deg <= 20.0 && ...
    result.handoff_angle_step_max_deg <= 5.0;

disp(result);
if result.pass
    fprintf('HFI_FLUX_VERIFY: PASS\n');
else
    fprintf('HFI_FLUX_VERIFY: FAIL\n');
end
end

function y = wrap_deg(x)
y = mod(x + 180.0, 360.0) - 180.0;
end
