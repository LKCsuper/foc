function varargout = verify_foc_algorithms()
%VERIFY_FOC_ALGORITHMS  用 MATLAB 对照当前 C 实现验证 FOC 核心算法。
%
%  桌面：当前文件夹切到 Doc/matlab，输入 verify_foc_algorithms
%  无界面：matlab -nojvm -batch "cd('F:/code/foc/Doc/matlab'); verify_foc_algorithms"
%  日志：Doc/matlab/out/verify_foc_algorithms.log
%  出图：桌面 MATLAB 默认弹出；命令行设 FOC_MATLAB_PLOT=0 可关掉
%
%  覆盖：Clarke/Park/逆 Park、VESC foc_svm、svm_hqd、最长零矢量重构、
%        电流 PI、Hall 扇区插值、弱磁门槛、空载 3000 反电势。

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
out_dir = fullfile(this_dir, 'out');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end
log_path = fullfile(out_dir, 'verify_foc_algorithms.log');
log_fid = fopen(log_path, 'w');
env_plot = getenv('FOC_MATLAB_PLOT');
if ~isempty(env_plot)
    do_plot = ~strcmp(env_plot, '0');
else
    do_plot = usejava('desktop');
end
keep_fig = do_plot && usejava('desktop');

c = foc.constants();
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

    function check_close(a, b, tol, msg)
        d = max(abs(double(a(:)) - double(b(:))));
        check(d <= tol, sprintf('%s  maxΔ=%.4g', msg, d));
    end

say('');
say('=== 1. Clarke / Park / 逆 Park ===');
[ia, ib, ic] = deal(1.0, -0.5, -0.5);
[al, be] = foc.clarke(ia, ib, ic);
check_close(al, 1.0, 1e-6, '平衡 1/-0.5/-0.5 → Iα=1');
check_close(be, 0.0, 1e-6, '平衡 1/-0.5/-0.5 → Iβ=0');

[ia2, ib2, ic2] = foc.inverse_clarke(al, be);
check_close([ia2, ib2, ic2], [ia, ib, ic], 1e-6, 'Clarke 与逆 Clarke 互逆');

[id0, iq0] = foc.park(al, be, 0.0);
check_close([id0, iq0], [1.0, 0.0], 1e-6, 'theta=0 时 Id=Iα, Iq=Iβ');
[id90, iq90] = foc.park(al, be, pi / 2);
check_close([id90, iq90], [0.0, -1.0], 1e-6, 'theta=90° 时 Id=Iβ, Iq=-Iα');

vd = 1.0;
vq = 2.0;
th = 0.7;
[va, vb] = foc.inverse_park(vd, vq, th);
[idb, iqb] = foc.park(va, vb, th);
check_close([idb, iqb], [-vd, -vq], 1e-6, 'park(inverse_park)= -v（C 侧多一次 180°）');

s = sin(th);
co = cos(th);
va_tb = vd * co - vq * s;
vb_tb = vd * s + vq * co;
check_close([va, vb], [-va_tb, -vb_tb], 1e-6, 'inverse_park 相对教科书差 180°');

say('');
say('=== 2. foc_svm 零点与线性增益 ===');
arr = c.ARR;
vbus = c.VBUS;
[tA, tB, tC] = foc.foc_svm(0, 0, arr);
check(all([tA, tB, tC] == idivide(arr, uint32(2))), 'foc_svm(0,0) CCR = ARR/2 = 2800');

vlim_lin = vbus / sqrt(3);
vlim_duty = vlim_lin * c.MAX_DUTY;
[tA, tB, tC] = foc.foc_svm_from_voltage(0, 0, vbus, c.MAX_DUTY, arr);
check(all([tA, tB, tC] == uint32(2800)), 'foc_svm_from_voltage(0) CCR=2800');

req = 10.0;
[tA, tB, tC] = foc.foc_svm_from_voltage(req, 0, vbus, c.MAX_DUTY, arr);
[~, ~, vmag] = foc.pwm_reconstruct(tA, tB, tC, arr, vbus);
check_close(vmag, req, 0.08, sprintf('请求 %.1f V 时 PWM ideal≈请求（VESC 归一）', req));

[tA, tB, tC] = foc.svm_hqd(req, 0, vbus, arr);
[~, ~, vmag_hqd] = foc.pwm_reconstruct(tA, tB, tC, arr, vbus);
check(vmag_hqd < req * 0.45 && vmag_hqd > req * 0.25, ...
      sprintf('svm_hqd 请求 10 V 时电机端约 1/3（实测 %.2f V）', vmag_hqd));

[tAh, tBh, tCh] = foc.svm_hqd(0.2, 0.0, vbus, arr);
mid_hqd = mean(double([tAh, tBh, tCh]));
check(abs(mid_hqd - double(arr) / 4) < 80, sprintf('svm_hqd 小电压中点约 ARR/4（%.1f）', mid_hqd));

[tA, tB, tC] = foc.foc_svm_from_voltage(16.0, 0, vbus, c.MAX_DUTY, arr);
[~, ~, vmag_sat] = foc.pwm_reconstruct(tA, tB, tC, arr, vbus);
check(vmag_sat <= vlim_duty + 0.15, sprintf('过调制钳到 (√3/2)*0.95 → %.2f V', vmag_sat));

say('');
say('=== 3. Hall 路径：αβ 再取反 + A-C-B ===');
vd = 0.0;
vq = 6.8;
th = 0.0;
[va, vb] = foc.inverse_park(vd, vq, th);
[tA, tB, tC] = foc.foc_svm_from_voltage(-va, -vb, vbus, c.MAX_DUTY, arr);
[pa, pb, pc] = deal(tA, tC, tB);
[ra, rb] = foc.pwm_reconstruct(pa, pb, pc, arr, vbus);
ang = atan2(rb, ra);
check(abs(foc.wrap_pi(ang - (-pi / 2))) < 0.15, ...
      sprintf('Hall 路径 theta=0、纯 Vq 经 A-C-B 后 β 为负（ang=%.1f°）', ang * 180 / pi));

say('');
say('=== 4. 最长零矢量重构 ===');
[ia, ib, ic, hit] = foc.reconstruct_longest_zero(1.0, -0.4, -0.2, 2000, 1800, 1600, 5600);
check(hit == 0 && abs(ia - 1.0) < 1e-12, '窗口足够（CCR≤ARR-500）不改写');
[ia, ib, ic, hit] = foc.reconstruct_longest_zero(1.0, -0.4, -0.2, 4864, 3105, 735, 5600);
check(hit == 0, '空载 3000 峰值 CCR≈4864 门槛 500 仍用三相实测');
[ia, ib, ic, hit] = foc.reconstruct_longest_zero(1.0, -0.4, -0.2, 5200, 1800, 1600, 5600);
check(hit == 1 && abs(ia - 0.6) < 1e-12, 'A 相唯一最高 → ia=-(ib+ic)=0.6');
[~, ~, ~, hit] = foc.reconstruct_longest_zero(1.0, -0.4, -0.2, 5200, 5200, 1600, 5600);
check(hit == 0, '两相并列最大不改写');

say('');
say('=== 5. 电流 PI + RL 植物 ===');
vlim = vbus * c.ONE_BY_SQRT3;
[i12, os12] = sim_current_step(0.8, c.KP, c.KI, c.DT, c.R_PHASE, c.L_PHASE, vlim);
[i20, os20] = sim_current_step(0.8, 0.2, c.KI, c.DT, c.R_PHASE, c.L_PHASE, vlim);
check(abs(i12(end) - 0.8) < 0.05, sprintf('olkp=0.12 80 ms 后 Iq≈0.80（%.3f A）', i12(end)));
check(os12 < 0.25, sprintf('olkp=0.12 电流超调 %.1f%%（150→210 RPM 是速度环）', 100 * os12));
bw12 = c.KP / (2.0 * pi * c.L_PHASE);
bw20 = 0.2 / (2.0 * pi * c.L_PHASE);
check(bw20 > bw12, sprintf('olkp=0.20 电流环带宽更高（%.0f vs %.0f Hz），速度环更容易过冲', bw20, bw12));
[v0_12, ~] = foc.current_pi(0.8, 0.0, 0.0, c.KP, c.KI, c.DT, vlim);
[v0_20, ~] = foc.current_pi(0.8, 0.0, 0.0, 0.2, c.KI, c.DT, vlim);
check(v0_20 > v0_12, sprintf('首拍电压随 Kp 变大（%.3f → %.3f V）', v0_12, v0_20));

vint = 0.0;
meas = 0.2;
for k = 1:20
    [~, vint] = foc.current_pi(1.0, meas, vint, 0.2, 30.0, 0.001, 12.0);
end
check(vint > 0.0, 'Iq 偏低时积分正向累积');

say('');
say('=== 6. Hall 扇区插值 ===');
center = 0.0;
th150 = foc.hall_interpolate(center, 150.0, c.POLE_PAIRS, 0.001, 1);
check_close(th150, 0.0, 1e-12, '150 RPM 钉在扇区中心');
th300_enter = foc.hall_interpolate(center, 300.0, c.POLE_PAIRS, 0.0, 1);
check_close(th300_enter * 180 / pi, -30.0, 0.5, '300 RPM 刚进扇区：中心-30°');
sec_t = (pi / 6) / (300.0 / 60.0 * c.POLE_PAIRS * 2.0 * pi);
th300_mid = foc.hall_interpolate(center, 300.0, c.POLE_PAIRS, sec_t, 1);
check_close(th300_mid, 0.0, 1e-3, '300 RPM 扇区中点回到中心');

say('');
say('=== 7. 弱磁与额定反电势 ===');
check_close(foc.fw_id_room(0.8, 2.5), sqrt(2.5^2 - 0.8^2), 1e-9, '电流圆优先留 Iq');
check_close(foc.fw_id_room(2.5, 2.5), 0.0, 1e-12, '|Iq| 顶满则弱磁余量=0');
fw = struct('id', 0.0);
[id_off, fw] = foc.fw_step(fw, 0.0, 0.9, 10.0, 10.3, 0.0, 13.8, c.DT);
check(id_off == 0.0, 'fw_current_max=0 时 Id_fw=0');
[id_lo, fw] = foc.fw_step(fw, 1.5, 0.9, 1.0e6, 10.3, 0.0, 13.8, c.DT);
check(id_lo == 0.0, '空载 3000 |V|/Vlim≈0.75 < 0.90，不开弱磁');
[id_mid, fw] = foc.fw_step(fw, 1.5, 0.9, 1.0e6, 13.11, 0.0, 13.8, c.DT);
check_close(id_mid, -0.75, 1e-12, 'VESC 弱磁在启动点到最大调制度之间线性增加');
[id_hi, fw] = foc.fw_step(fw, 1.5, 0.9, 1.0e6, 13.8, 0.0, 13.8, c.DT);
check_close(id_hi, -1.5, 1e-12, '|V|/Vlim=1 时目标=-current_max');
fw_v = struct('id', 0.0);
[id_v1, fw_v] = foc.fw_voltage_step(fw_v, 12.0, 0.8, 0.92, 10.0, 13.8, 0.0, 13.8, 0.1);
check_close(id_v1, -0.08, 1e-12, '电压超过目标时弱磁积分向负 Id 增加');
[id_v2, ~] = foc.fw_voltage_step(fw_v, 12.0, 0.8, 0.92, 10.0, 11.04, 0.0, 13.8, 0.1);
check(id_v2 > id_v1, '电压低于目标时弱磁积分退出');

we = c.RATED_RPM / 60.0 * c.POLE_PAIRS * 2.0 * pi;
e_bemf = we * c.FLUX;
check(e_bemf > vlim, sprintf('3000 RPM ωeλ=%.2f V > Vlim=%.2f V（带载才需要弱磁）', e_bemf, vlim));
check(10.3 < vlim, '空载 3000 实测 |V|≈10.3 V 仍低于 Vlim，弱磁保持关闭');

if do_plot
    say('');
    say('=== 8. 扫角 / 阶跃曲线 ===');
    try
        make_plots(c, out_dir, i12, i20, os12, os20, keep_fig);
    catch me
        say(sprintf('  跳过绘图：%s', me.message));
    end
else
    say('');
    say('=== 8. 扫角 / 阶跃曲线 ===');
    say('  跳过绘图（桌面 MATLAB 会出图；命令行设 FOC_MATLAB_PLOT=1）');
end

say('');
say(sprintf('结果：%d PASS, %d FAIL', npass, nfail));
status_path = fullfile(out_dir, 'verify_status.txt');
sf = fopen(status_path, 'w');
if nfail == 0
    fprintf(sf, 'PASS %d\n', npass);
else
    fprintf(sf, 'FAIL %d %d\n', npass, nfail);
end
fclose(sf);
if log_fid > 0
    fclose(log_fid);
    log_fid = -1;
end
if nargout > 0
    varargout{1} = nfail;
end
if nfail > 0
    error('verify_foc_algorithms: %d 项失败', nfail);
end
end

function [i_hist, overshoot, t90] = sim_current_step(target, kp, ki, dt, r, l, vlim)
n = round(0.08 / dt);
i = 0.0;
v_delay = 0.0;
vint = 0.0;
i_hist = zeros(n, 1);
for k = 1:n
    [v, vint] = foc.current_pi(target, i, vint, kp, ki, dt, vlim);
    i = i + dt / l * (v_delay - r * i);
    v_delay = v;
    i_hist(k) = i;
end
overshoot = max(0.0, (max(i_hist) - target) / target);
idx = find(i_hist >= 0.9 * target, 1, 'first');
if isempty(idx)
    t90 = inf;
else
    t90 = idx * dt;
end
end

function make_plots(c, out_dir, i12, i20, os12, os20, keep_fig)
arr = c.ARR;
vbus = c.VBUS;
vreq = linspace(0.2, 16.0, 80);
gain_svm = zeros(size(vreq));
gain_hqd = zeros(size(vreq));
for k = 1:numel(vreq)
    [tA, tB, tC] = foc.foc_svm_from_voltage(vreq(k), 0, vbus, c.MAX_DUTY, arr);
    [~, ~, mag] = foc.pwm_reconstruct(tA, tB, tC, arr, vbus);
    gain_svm(k) = mag / vreq(k);
    [tA, tB, tC] = foc.svm_hqd(vreq(k), 0, vbus, arr);
    [~, ~, mag] = foc.pwm_reconstruct(tA, tB, tC, arr, vbus);
    gain_hqd(k) = mag / vreq(k);
end

fig = figure('Name', 'SVM gain', 'NumberTitle', 'off');
plot(vreq, gain_svm, 'b', vreq, gain_hqd, 'r', 'LineWidth', 1.2);
yline(1.0, ':k');
yline(1 / 3, ':k');
grid on;
xlabel('|V_{αβ}| 请求 (V)');
ylabel('PWM ideal / 请求');
legend('foc\_svm\_from\_voltage', 'svm\_hqd', 'Location', 'northeast');
title('调制增益：VESC SVM ≈ 1，旧 svm\_hqd ≈ 1/3');
print(fig, fullfile(out_dir, 'svm_gain.png'), '-dpng', '-r120');
if ~keep_fig
    close(fig);
end

th = linspace(0, 2 * pi, 361);
ccr_a = zeros(size(th));
ccr_b = zeros(size(th));
ccr_c = zeros(size(th));
vmag = zeros(size(th));
vq = 10.3;
for k = 1:numel(th)
    [va, vb] = foc.inverse_park(0.0, vq, th(k));
    [tA, tB, tC] = foc.foc_svm_from_voltage(-va, -vb, vbus, c.MAX_DUTY, arr);
    ccr_a(k) = double(tA);
    ccr_b(k) = double(tC);
    ccr_c(k) = double(tB);
    [~, ~, vmag(k)] = foc.pwm_reconstruct(tA, tC, tB, arr, vbus);
end
fig = figure('Name', 'SVM CCR', 'NumberTitle', 'off');
plot(th * 180 / pi, ccr_a, th * 180 / pi, ccr_b, th * 180 / pi, ccr_c, 'LineWidth', 1.1);
yline(2800, ':k');
grid on;
xlabel('电角 (deg)');
ylabel('CCR');
legend('A', 'B(phys=C)', 'C(phys=B)', 'ARR/2', 'Location', 'best');
title(sprintf('Hall 路径 |Vq|=%.1f V：中点 2800，峰值约 %.0f', vq, max(ccr_a)));
print(fig, fullfile(out_dir, 'svm_ccr_3000.png'), '-dpng', '-r120');
if ~keep_fig
    close(fig);
end

t = (1:numel(i12)) * c.DT * 1e3;
fig = figure('Name', 'current step', 'NumberTitle', 'off');
plot(t, i12, 'b', t, i20, 'r', 'LineWidth', 1.2);
yline(0.8, ':k');
grid on;
xlabel('时间 (ms)');
ylabel('Iq (A)');
legend(sprintf('kp=0.12 超调 %.1f%%', 100 * os12), sprintf('kp=0.20 超调 %.1f%%', 100 * os20), ...
       'Location', 'southeast');
title('电流环阶跃 0→0.8 A（R=0.2 Ω，L=0.4 mH，15 kHz，1 拍延迟）');
print(fig, fullfile(out_dir, 'current_pi_step.png'), '-dpng', '-r120');
if ~keep_fig
    close(fig);
end

rpm = [50, 150, 225, 300, 1000, 3000];
err_enter = zeros(size(rpm));
for k = 1:numel(rpm)
    th0 = foc.hall_interpolate(0.0, rpm(k), c.POLE_PAIRS, 0.0, 1);
    true_enter = -pi / 6;
    if rpm(k) <= 150
        true_enter = 0.0;
    elseif rpm(k) < 300
        blend = (rpm(k) - 150) / 150;
        true_enter = blend * (-pi / 6);
    end
    err_enter(k) = abs(th0 - true_enter) * 180 / pi;
end
fig = figure('Name', 'hall interp', 'NumberTitle', 'off');
bar(categorical(rpm), err_enter);
ylabel('刚进扇区相对模型的误差 (deg)');
title('Hall 插值：150 钉中心，300 以上满 ±30°');
grid on;
print(fig, fullfile(out_dir, 'hall_interp.png'), '-dpng', '-r120');
if ~keep_fig
    close(fig);
end

if license('test', 'Control_Toolbox')
    try
        s = tf('s');
        L = c.L_PHASE;
        R = c.R_PHASE;
        Td = 1.5 / c.FS;
        G = 1 / (L * s + R);
        C12 = c.KP + c.KI / s;
        C20 = 0.2 + c.KI / s;
        Gc12 = feedback(C12 * G * exp(-Td * s), 1);
        Gc20 = feedback(C20 * G * exp(-Td * s), 1);
        fig = figure('Name', 'current bode', 'NumberTitle', 'off');
        bodemag(Gc12, Gc20);
        legend('kp=0.12 ki=18', 'kp=0.20 ki=18', 'Location', 'southwest');
        title('电流闭环伯德图（含 1.5/fs 延迟）');
        print(fig, fullfile(out_dir, 'current_bode.png'), '-dpng', '-r120');
        if ~keep_fig
    close(fig);
end
        fprintf('  已写 current_bode.png（Control System Toolbox）\n');
    catch me
        fprintf('  跳过伯德图：%s\n', me.message);
    end
else
    fprintf('  未装 Control System Toolbox，跳过伯德图\n');
end
fprintf('  曲线已写入 %s\n', out_dir);
end
