function ekf_motor_plot(m, out_dir, png_name)
%EKF_MOTOR_PLOT  把「转子追 I/F、EKF 追转子」画成能分开的四张。
if ~isfield(m, 'th_e')
    error('ekf_motor_plot:need_angle', '需要 ekf_motor_sim 的 th_e/th_if/th_ekf');
end

th = unwrap([m.th_if, m.th_e, m.th_ekf]) * 180.0 / pi;
fig = figure('Name', 'I/F 与 EKF 跟随', 'NumberTitle', 'off', 'Color', 'w');

subplot(2, 2, 1);
plot(m.t, th(:, 1), '--', 'LineWidth', 1.3);
hold on;
plot(m.t, th(:, 2), 'LineWidth', 1.6);
plot(m.t, th(:, 3), ':', 'LineWidth', 2.0);
hold off;
grid on;
ylabel('展开电角 (°)');
legend('I/F 开环角', '转子真角', 'EKF', 'Location', 'northwest');
title('跟随看这里：EKF 贴转子，I/F 在前面拖');

subplot(2, 2, 2);
plot(m.t, m.rpm_if, '--', 'LineWidth', 1.3);
hold on;
plot(m.t, m.rpm_true, 'LineWidth', 1.6);
hold off;
grid on;
ylabel('RPM');
legend('I/F 斜坡', '转子', 'Location', 'southeast');
title('转速：转子在追 I/F 斜坡');

subplot(2, 2, 3);
yyaxis left;
plot(m.t, m.if_err_deg, 'LineWidth', 1.4);
ylabel('I/F − 转子 (°)');
ylim([-10, 90]);
yyaxis right;
plot(m.t, m.ekf_err_deg, 'LineWidth', 1.4);
ylabel('EKF − 转子 (°)');
grid on;
xlabel('t (s)');
title('负载角大 ≠ EKF 没跟上');

subplot(2, 2, 4);
plot(m.t, m.rpm_true - m.rpm_if, 'LineWidth', 1.3);
hold on;
plot(m.t, m.rpm_ekf - m.rpm_true, 'LineWidth', 1.5);
hold off;
grid on;
ylabel('RPM');
xlabel('t (s)');
legend('转子 − I/F', 'EKF − 转子', 'Location', 'best');
title('负值=还没追上；黄线≈0 才是 EKF 跟上了');

if nargin >= 3 && ~isempty(out_dir) && ~isempty(png_name)
    print(fig, fullfile(out_dir, png_name), '-dpng', '-r120');
end
end
