function varargout = verify_ekf()
%VERIFY_EKF  对照 motor_ekf.c / test_motor_ekf.c，开一条 I/F→EKF 算法线。
%
%  Hall 日用路径不动（olfb hall, slenable 0）。本脚本不改固件。
%  桌面：cd Doc/matlab，输入 verify_ekf
%  无界面：matlab -nojvm -batch "cd('F:/code/foc/Doc/matlab'); verify_ekf"
%  日志：Doc/matlab/out/verify_ekf.log
%  出图：桌面 MATLAB 默认弹出并写入 out/ekf_*.png；命令行设 FOC_MATLAB_PLOT=0 关掉

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
out_dir = fullfile(this_dir, 'out');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end
log_path = fullfile(out_dir, 'verify_ekf.log');
log_fid = fopen(log_path, 'w');
env_plot = getenv('FOC_MATLAB_PLOT');
if ~isempty(env_plot)
    do_plot = ~strcmp(env_plot, '0');
else
    do_plot = true;
end
keep_fig = do_plot;

c = foc.constants();
dt = c.DT;
nfail = 0;
npass = 0;

    function say(msg)
        fprintf('%s\n', msg);
        if log_fid > 0
            fprintf(log_fid, '%s\n', msg);
        end
    end

    function check(cond, msg)
        if cond
            npass = npass + 1;
            say(sprintf('  PASS  %s', msg));
        else
            nfail = nfail + 1;
            say(sprintf('  FAIL  %s', msg));
        end
    end

cfg = default_cfg(c);

say('');
say('=== EKF 算法线（Hall 冻结，不进 ISR） ===');
say(sprintf('  R=%.3f Ω  L=%.4f mH  ψf=%.5f Wb  fs=%.0f Hz', ...
            cfg.resistance, cfg.inductance * 1e3, cfg.flux_linkage, c.FS));

say('');
say('=== 1. 关闭 / 非法 dt 复位 ===');
obs = foc.ekf_reset();
check(~obs.initialized, 'Reset 后 uninitialized');
cfg_off = cfg;
cfg_off.enabled = false;
[obs, out, ok] = foc.ekf_step(obs, cfg_off, 1.0, 0.0, 0.0, 0.0, dt);
check(~ok && ~out.valid && ~obs.initialized, 'enabled=false 复位且 invalid');
obs = foc.ekf_seed(foc.ekf_reset(), cfg, 0.2, 100.0, 0.0, 0.0);
[obs, out, ok] = foc.ekf_step(obs, cfg, 1.0, 0.0, 0.0, 0.0, 0.0);
check(~ok && ~obs.initialized, 'dt=0 复位');

say('');
say('=== 2. Seed / phase_offset ===');
obs = foc.ekf_seed(foc.ekf_reset(), cfg, 0.40, 2.0 * pi * 100.0, 0.0, 0.0);
check(obs.initialized, 'Seed 后 initialized');
check(abs(obs.angle - 0.40) < 1e-5, sprintf('Seed 角=0.40  得 %.5f', obs.angle));
check(abs(obs.speed_e - 2.0 * pi * 100.0) < 1e-4, 'Seed ωe=2π·100');
cfg_off = cfg;
cfg_off.phase_offset = 0.30;
obs = foc.ekf_seed(foc.ekf_reset(), cfg_off, 0.10, 100.0, 0.0, 0.0);
check(abs(foc.wrap_pi(obs.angle - 0.40)) < 1e-5, 'phase_offset 加在 θ 之后');

say('');
say('=== 3. 对照 test_motor_ekf.c ===');
we100 = 2.0 * pi * 100.0;
s = foc.ekf_track(cfg, 0.40, we100, 0.0, 400, 0.40, we100, dt);
check(s.ok && s.angle_err < 0.08, sprintf('100 Hz Iq=0  400拍  |Δθ|=%.4f rad（阈 0.08）', s.angle_err));
check(s.we_err_rel < 0.10, sprintf('100 Hz 转速相对误差 %.3f（阈 0.10）', s.we_err_rel));
check(s.speed_ok, '100 Hz speed_ok（|ωe|>50）');

s = foc.ekf_track(cfg, 0.20, we100, 2.0, 400, 0.20, we100, dt);
check(s.ok && s.angle_err < 0.10, sprintf('100 Hz Iq=2  400拍  |Δθ|=%.4f rad（阈 0.10）', s.angle_err));

s_mis = foc.ekf_track(cfg, 1.00, we100, 0.0, 1200, 0.50, we100, dt);
check(s_mis.ok && s_mis.angle_err < 0.15, sprintf('初值差 0.50 rad  1200拍  |Δθ|=%.4f rad（阈 0.15）', s_mis.angle_err));

we_rev = -2.0 * pi * 80.0;
s = foc.ekf_track(cfg, 0.30, we_rev, 0.0, 500, 0.30, we_rev, dt);
check(s.ok && s.angle_err < 0.12, sprintf('反转 −80 Hz  |Δθ|=%.4f rad（阈 0.12）', s.angle_err));
check(s.speed_e < 0.0, sprintf('反转 ωe=%.1f < 0', s.speed_e));

say('');
say('=== 4. I/F 种子 → EKF（空载欧拉植物） ===');
s300 = foc.ekf_if_handoff(cfg, 300.0, 20.0, 1200, dt);
say(sprintf('  300 RPM  种子+20°  tail_rms=%.2f°  tail_max=%.1f°  |Δθ|=%.3f rad  we_err=%.3f  speed_ok=%d', ...
            s300.tail_rms_deg, s300.tail_max_deg, s300.angle_err, s300.we_err_rel, s300.speed_ok));
check(s300.ok && s300.angle_err < 0.15 && s300.speed_ok, ...
      sprintf('300 RPM I/F+20° 锁角 |Δθ|=%.4f rad 且 speed_ok', s300.angle_err));

s3000 = foc.ekf_if_handoff(cfg, 3000.0, 20.0, 1200, dt);
say(sprintf('  3000 RPM 种子+20°  tail_rms=%.2f°  tail_max=%.1f°  |Δθ|=%.3f rad  we_err=%.3f', ...
            s3000.tail_rms_deg, s3000.tail_max_deg, s3000.angle_err, s3000.we_err_rel));
check(s3000.ok && s3000.angle_err < 0.15, sprintf('3000 RPM I/F+20° 锁角 |Δθ|=%.4f rad', s3000.angle_err));

s72 = foc.ekf_if_handoff(cfg, 72.0, 20.0, 1200, dt);
say(sprintf('  72 RPM   种子+20°  tail_rms=%.2f°  |Δθ|=%.3f rad  speed_ok=%d  （零速附近应不可观）', ...
            s72.tail_rms_deg, s72.angle_err, s72.speed_ok));
check(~s72.speed_ok, '72 RPM speed_ok=false（|ωe|≈15 < 50，不可交给 Park）');

say('');
say('=== 5. 铭牌电机植物（ATK-BL57H95D20E） ===');
say(sprintf('  J=%.2e kg·m²  B=%.2e N·m·s/rad（B 为假设）  Id_if=%.2f A  目标 %.0f RPM', ...
            c.J, c.B_VISC, c.IQ_IF, c.SL_TRANS_RPM));
m = foc.ekf_motor_sim();
say(sprintf('  终了 真=%.1f RPM  I/F=%.1f  EKF=%.1f  |Δθ|=%.2f°  max|i|=%.2f A  speed_ok=%d', ...
            m.final_rpm, m.rpm_if(end), m.rpm_ekf(end), m.final_ekf_err * 180.0 / pi, m.max_i, m.speed_ok_last));
check(m.ok, sprintf('铭牌植物 %d 拍 EKF 未复位', m.nstep));
check(abs(m.final_rpm - m.rpm_target) < 80.0, sprintf('空载跟上 I/F %.0f RPM  真=%.1f', m.rpm_target, m.final_rpm));
check(m.final_ekf_err < 15.0 * pi / 180.0, sprintf('影子 EKF |Δθ|=%.2f°（阈 15°）', m.final_ekf_err * 180.0 / pi));
check(m.speed_ok_last, '500 RPM 时 EKF speed_ok');
check(m.max_i < c.PEAK_CURRENT, sprintf('相电流峰值 %.2f A < 19.8 A', m.max_i));

say('');
say('=== 6. 边界 ===');
say('  Hall 日用：olfb hall, slenable 0, olkp 0.12, olki 18。本脚本未改固件。');
say('  EKF 仍未接入 Motor_Control_Step。植物仿真不是开关 PWM，也不含 HD75 180°。');
say('  固件斜坡默认 10 RPM/s；仿真先对齐 40 ms，再 1200 RPM/s 拉到 500 RPM。');

say('');
say('=== 7. 曲线 ===');
if do_plot
    try
        plot_case(s_mis, 'EKF seed mismatch 0.50 rad @ 100 Hz', dt, out_dir, 'ekf_seed_mismatch.png');
        plot_case(s300, 'EKF I/F+20° @ 300 RPM', dt, out_dir, 'ekf_if_300.png');
        plot_case(s3000, 'EKF I/F+20° @ 3000 RPM', dt, out_dir, 'ekf_if_3000.png');
        plot_case(s72, 'EKF I/F+20° @ 72 RPM (speed_ok=0)', dt, out_dir, 'ekf_if_72.png');
        foc.ekf_motor_plot(m, out_dir, 'ekf_motor_sim.png');
        say(sprintf('  已出图，PNG 在 %s', out_dir));
        if ~keep_fig
            pause(0.3);
            close all;
        end
    catch me
        say(sprintf('  跳过绘图：%s', me.message));
    end
else
    say('  跳过绘图（桌面 MATLAB 会出图；命令行设 FOC_MATLAB_PLOT=1）');
end

say('');
if nfail == 0
    say(sprintf('RESULT  PASS  %d/%d', npass, npass + nfail));
else
    say(sprintf('RESULT  FAIL  %d passed, %d failed', npass, nfail));
end
if log_fid > 0
    fclose(log_fid);
    log_fid = -1;
end

if nargout > 0
    varargout{1} = nfail == 0;
end
end

function cfg = default_cfg(c)
cfg.enabled = true;
cfg.resistance = c.R_LINE;
cfg.inductance = c.L_LINE;
cfg.flux_linkage = c.FLUX;
cfg.q_current = 0.0;
cfg.q_speed = 0.0;
cfg.q_angle = 0.0;
cfg.r_current = 0.0;
cfg.phase_offset = 0.0;
end

function plot_case(s, title_str, dt, out_dir, png_name)
if ~isfield(s, 'err') || numel(s.err) < 2
    return
end
t = (0:numel(s.err) - 1).' * dt * 1e3;
fig = figure('Name', title_str, 'NumberTitle', 'off', 'Color', 'w');
subplot(2, 1, 1);
plot(t, s.err * 180.0 / pi, 'LineWidth', 1.2);
grid on;
ylabel('\Delta\theta (deg)');
title(title_str);
subplot(2, 1, 2);
plot(t, s.we, 'LineWidth', 1.2);
grid on;
hold on;
if isfield(s, 'omega_e') && isfinite(s.omega_e)
    yline(s.omega_e, '--');
end
hold off;
ylabel('\omega_e (rad/s)');
xlabel('t (ms)');
if nargin >= 5 && ~isempty(out_dir) && ~isempty(png_name)
    print(fig, fullfile(out_dir, png_name), '-dpng', '-r120');
end
end
