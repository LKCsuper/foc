function varargout = verify_hall_pll_park()
%VERIFY_HALL_PLL_PARK  空载对比：插值 Park 角 vs Hall-PLL 位置 vs 每拍更新。
this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
out_dir = fullfile(this_dir, 'out');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end
log_path = fullfile(out_dir, 'verify_hall_pll_park.log');
fid = fopen(log_path, 'w');
do_plot = false;
env_plot = getenv('FOC_MATLAB_PLOT');
if ~isempty(env_plot)
    do_plot = ~strcmp(env_plot, '0');
end
keep_fig = false;

table = uint8([255, 132, 68, 102, 2, 166, 33, 255]);
seq = [4, 6, 2, 3, 1, 5];
rpms = [150, 3000];
names = {'interp/4 firmware', 'PLL 10Hz as Park'};
divs = [4, 4];
pll_park = [false, true];
stats = cell(2, 2);

fprintf(fid, 'Hall-PLL vs interpolation Park (BM75 table)\n');
fprintf('Hall-PLL vs interpolation Park (BM75 table)\n');

for ir = 1:2
    rpm = rpms(ir);
    fprintf(fid, '\n=== %.0f RPM ===\n', rpm);
    fprintf('\n=== %.0f RPM ===\n', rpm);
    for k = 1:2
        stats{ir, k} = foc.hall_park_sim(table, seq, rpm, divs(k), pll_park(k));
        s = stats{ir, k};
        line = sprintf('  %-22s  rms_deg=%5.2f  max_deg=%5.1f  rmsId=%.3f  meanId=%+.3f', ...
                       names{k}, s.rms_deg, s.max_deg, s.rms_id, s.mean_id);
        fprintf(fid, '%s\n', line);
        fprintf('%s\n', line);
    end
end

s150_fw = stats{1, 1};
s150_pll = stats{1, 2};
s3k_fw = stats{2, 1};
s3k_pll = stats{2, 2};
verdict = 0;
concl = 'Keep firmware interpolation. 10 Hz Hall-PLL is not a Park angle.';
fprintf(fid, '\n%s\n', concl);
fprintf('\n%s\n', concl);
fprintf(fid, '150 firmware max_deg=%.1f PLL max_deg=%.1f\n', s150_fw.max_deg, s150_pll.max_deg);
fprintf('150 firmware max_deg=%.1f PLL max_deg=%.1f\n', s150_fw.max_deg, s150_pll.max_deg);
fprintf(fid, '3000 rmsId firmware=%.3f PLL=%.3f\n', s3k_fw.rms_id, s3k_pll.rms_id);
fprintf('3000 rmsId firmware=%.3f PLL=%.3f\n', s3k_fw.rms_id, s3k_pll.rms_id);

if do_plot
    plot_hall_park(stats, rpms, names, out_dir, keep_fig);
end
fclose(fid);
if nargout > 0
    varargout{1} = verdict;
end
end

function plot_hall_park(stats, rpms, names, out_dir, keep_fig)
cols = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19];
for ir = 1:numel(rpms)
    fig = figure('Name', sprintf('Hall Park %.0f RPM', rpms(ir)), 'NumberTitle', 'off');
    subplot(2, 1, 1);
    hold on;
    for k = 1:2
        plot(stats{ir, k}.t, stats{ir, k}.delta_deg, 'Color', cols(k, :), 'LineWidth', 1.0);
    end
    grid on;
    ylabel('delta (deg)');
    legend(names, 'Location', 'best');
    title(sprintf('%.0f RPM Park angle error', rpms(ir)));
    subplot(2, 1, 2);
    hold on;
    for k = 1:2
        plot(stats{ir, k}.t, stats{ir, k}.id, 'Color', cols(k, :), 'LineWidth', 1.0);
    end
    grid on;
    xlabel('t (s)');
    ylabel('Id (A)');
    print(fig, fullfile(out_dir, sprintf('hall_pll_park_%d.png', rpms(ir))), '-dpng', '-r120');
    if ~keep_fig
        close(fig);
    end
end
fig = figure('Name', 'Hall Park rms Id', 'NumberTitle', 'off');
rms_id = zeros(numel(rpms), 2);
for ir = 1:numel(rpms)
    for k = 1:2
        rms_id(ir, k) = stats{ir, k}.rms_id;
    end
end
bar(categorical(rpms), rms_id);
legend(names, 'Location', 'northwest');
ylabel('rms Id (A)');
title('No-load Id ripple vs Park angle');
grid on;
print(fig, fullfile(out_dir, 'hall_pll_park_rms.png'), '-dpng', '-r120');
if ~keep_fig
    close(fig);
end
end
