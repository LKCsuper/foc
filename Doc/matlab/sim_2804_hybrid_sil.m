function result = sim_2804_hybrid_sil(do_plot)
%SIM_2804_HYBRID_SIL  2804/BM75 编码器速度环软件在环仿真。
%
%  对齐当前固件的关键离散语义：
%    - FOC 15 kHz、速度环 1 kHz；
%    - 16 V 母线、DQ 电压上限 9 V；
%    - MT6701 16384 count/rev；
%    - BM75 0.0806 A/ADC count，三相分别量化后再 Clarke/Park；
%    - |目标|<=120 RPM 直接 Vq，超过后使用 Id/Iq 电流环；
%    - 2804 profile 的 PI、前馈、Rs/Ls、磁链和 7 极对。
%
%  桌面：cd Doc/matlab; sim_2804_hybrid_sil
%  命令行：matlab -batch "cd('F:/code/foc/Doc/matlab'); sim_2804_hybrid_sil(false)"

if nargin < 1
    do_plot = usejava('desktop');
end

p = parameters();
hybrid = run_case(p, true);
current_only = run_case(p, false);

out_dir = fullfile(fileparts(mfilename('fullpath')), 'out');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

checks = evaluate_case(hybrid, p);
result = struct('parameters', p, 'hybrid', hybrid, ...
                'current_only', current_only, 'checks', checks);
save(fullfile(out_dir, 'sim_2804_hybrid_sil.mat'), 'result');
write_report(fullfile(out_dir, 'sim_2804_hybrid_sil.log'), result);

if do_plot
    make_plots(out_dir, hybrid, current_only);
end

fprintf('2804 SIL: %d PASS, %d FAIL\n', sum([checks.pass]), sum(~[checks.pass]));
fprintf('Log: %s\n', fullfile(out_dir, 'sim_2804_hybrid_sil.log'));
if any(~[checks.pass])
    error('sim_2804_hybrid_sil:checkFailed', '2804 SIL acceptance failed');
end
end

function p = parameters()
profile = load_2804_profile();
p.dt = 1 / 15000;
p.speed_div = 15;
p.speed_dt = p.dt * p.speed_div;
p.vbus = 16.0;
p.v_limit = profile.speed_voltage_limit;
p.pole_pairs = profile.pole_pairs;
p.rs = profile.rs;
p.ld = profile.inductance;
p.lq = profile.inductance;
p.speed_ff_slope = profile.speed_ff_slope;
p.speed_ff_boost = profile.speed_ff_boost;
p.nameplate_flux = profile.nameplate_flux;
% 空载 Vq/RPM 拟合对应的有效磁链；植物用实测值，铭牌值单独保留供偏差研究。
p.flux = p.speed_ff_slope / (p.pole_pairs * 2*pi / 60);
p.inertia = profile.inertia;
p.viscous = 1.0e-6;
p.kt = 1.5 * p.pole_pairs * p.flux;
p.adc_a_per_count = 0.0806;
p.adc_zero_counts = [2045.512, 2044.619, 2044.888];
p.encoder_cpr = 16384;
p.encoder_dt = 1/10000;
p.encoder_pll_bw = profile.encoder_pll_bw;
p.current_kp = profile.current_kp;
p.current_ki = profile.current_ki;
p.speed_kp = profile.speed_kp;
p.speed_ki = profile.speed_ki;
p.speed_iq_limit = profile.speed_iq_limit;
p.speed_voltage_per_amp = profile.speed_voltage_per_amp;
p.hybrid_rpm = profile.speed_voltage_max_rpm;
p.ramp_rpm_s = 25000 / p.pole_pairs;
% 用实测 boost 折算空载库仑摩擦，使低速工作点与板上数据同量级。
p.coulomb_torque = p.kt * p.speed_ff_boost / p.rs;
p.segment_s = 3.0;
p.commands_rpm = [10, 100, 150, 2600, 150, 100, 10, 0];
end

function log = run_case(p, hybrid_enable)
n_segment = round(p.segment_s / p.dt);
n = n_segment * numel(p.commands_rpm);
t = (0:n-1)' * p.dt;
rpm_cmd = repelem(p.commands_rpm(:), n_segment);

rpm_ref = 0.0;
omega = 0.0;
theta_m = 0.0;
id = 0.0;
iq = 0.0;
vd_int = 0.0;
vq_int = 0.0;
speed_int = 0.0;
iq_ref = 0.0;
vd_cmd = 0.0;
vq_cmd = 0.0;
rpm_meas = 0.0;
pll_pos = 0.0;
pll_vel = 0.0;
encoder_time = 0.0;

rpm = zeros(n, 1);
rpm_meas_log = zeros(n, 1);
ref = zeros(n, 1);
id_meas_log = zeros(n, 1);
iq_meas_log = zeros(n, 1);
iq_ref_log = zeros(n, 1);
vd_log = zeros(n, 1);
vq_log = zeros(n, 1);
voltage_mode = false(n, 1);

for k = 1:n
    target = rpm_cmd(k);
    dr = p.ramp_rpm_s * p.dt;
    rpm_ref = rpm_ref + min(max(target - rpm_ref, -dr), dr);

    theta_e = wrap_2pi(theta_m * p.pole_pairs);
    encoder_count = mod(round(theta_m / (2*pi) * p.encoder_cpr), p.encoder_cpr);
    encoder_time = encoder_time + p.dt;
    if encoder_time + 1e-12 >= p.encoder_dt
        encoder_time = encoder_time - p.encoder_dt;
        pll_kp = 2.0 * p.encoder_pll_bw;
        pll_ki = 0.25 * pll_kp * pll_kp;
        pll_pos = mod(pll_pos + p.encoder_dt * pll_vel, p.encoder_cpr);
        pll_error = wrap_count(encoder_count - pll_pos, p.encoder_cpr);
        pll_pos = mod(pll_pos + p.encoder_dt * pll_kp * pll_error, p.encoder_cpr);
        pll_vel = pll_vel + p.encoder_dt * pll_ki * pll_error;
        if abs(pll_vel) < 0.5 * p.encoder_dt * pll_ki, pll_vel = 0.0; end
        rpm_meas = pll_vel / p.encoder_cpr * 60;
    end
    theta_m_q = encoder_count / p.encoder_cpr * 2*pi;
    theta_e_q = wrap_2pi(theta_m_q * p.pole_pairs);

    [ia, ib, ic] = dq_to_abc(id, iq, theta_e);
    % 对齐实机：ADC 先围绕各相的小数零点取整，再减去校准得到的浮点零点。
    % 小数零点会形成天然抖动量化，不能用理想整数零点替代，否则低电流死区会被夸大。
    ia = (round(p.adc_zero_counts(1) + ia / p.adc_a_per_count) - p.adc_zero_counts(1)) * p.adc_a_per_count;
    ib = (round(p.adc_zero_counts(2) + ib / p.adc_a_per_count) - p.adc_zero_counts(2)) * p.adc_a_per_count;
    ic = (round(p.adc_zero_counts(3) + ic / p.adc_a_per_count) - p.adc_zero_counts(3)) * p.adc_a_per_count;
    [ialpha, ibeta] = foc.clarke(ia, ib, ic);
    [id_meas, iq_meas] = foc.park(ialpha, ibeta, theta_e_q);

    if mod(k-1, p.speed_div) == 0
        speed_err = rpm_ref - rpm_meas;
        speed_int = speed_int + speed_err * p.speed_ki * p.speed_dt;
        iq_ref = speed_int + speed_err * p.speed_kp;
        if iq_ref > p.speed_iq_limit
            iq_ref = p.speed_iq_limit;
            speed_int = iq_ref - speed_err * p.speed_kp;
        elseif iq_ref < -p.speed_iq_limit
            iq_ref = -p.speed_iq_limit;
            speed_int = iq_ref - speed_err * p.speed_kp;
        end
    end

    use_voltage = hybrid_enable && abs(target) <= p.hybrid_rpm;
    if use_voltage
        if target > 0
            ff = p.speed_ff_boost + p.speed_ff_slope * abs(target);
        elseif target < 0
            ff = -(p.speed_ff_boost + p.speed_ff_slope * abs(target));
        else
            ff = 0.0;
        end
        vd_cmd = 0.0;
        vq_cmd = clamp(ff + iq_ref * p.speed_voltage_per_amp, -p.v_limit, p.v_limit);
        vd_int = vd_cmd;
        vq_int = vq_cmd;
    else
        [vd_cmd, vd_int] = foc.current_pi(0.0, id_meas, vd_int, ...
                                           p.current_kp, p.current_ki, p.dt, p.v_limit);
        [vq_cmd, vq_int] = foc.current_pi(iq_ref, iq_meas, vq_int, ...
                                           p.current_kp, p.current_ki, p.dt, p.v_limit);
        mag = hypot(vd_cmd, vq_cmd);
        if mag > p.v_limit
            scale = p.v_limit / mag;
            vd_cmd = vd_cmd * scale;
            vq_cmd = vq_cmd * scale;
            vd_int = vd_cmd - (0.0 - id_meas) * p.current_kp;
            vq_int = vq_cmd - (iq_ref - iq_meas) * p.current_kp;
        end
    end

    angle_error = wrap_pi(theta_e_q - theta_e);
    vd_true = vd_cmd * cos(angle_error) - vq_cmd * sin(angle_error);
    vq_true = vd_cmd * sin(angle_error) + vq_cmd * cos(angle_error);
    omega_e = omega * p.pole_pairs;
    id = id + p.dt / p.ld * (vd_true - p.rs * id + omega_e * p.lq * iq);
    iq = iq + p.dt / p.lq * (vq_true - p.rs * iq - omega_e * (p.ld * id + p.flux));

    friction = p.viscous * omega;
    if abs(omega) > 1e-3
        friction = friction + p.coulomb_torque * sign(omega);
    elseif abs(p.kt * iq) <= p.coulomb_torque
        friction = p.kt * iq;
    else
        friction = p.coulomb_torque * sign(p.kt * iq);
    end
    omega = omega + p.dt / p.inertia * (p.kt * iq - friction);
    theta_m = theta_m + omega * p.dt;

    rpm(k) = omega * 60 / (2*pi);
    rpm_meas_log(k) = rpm_meas;
    ref(k) = rpm_ref;
    id_meas_log(k) = id_meas;
    iq_meas_log(k) = iq_meas;
    iq_ref_log(k) = iq_ref;
    vd_log(k) = vd_cmd;
    vq_log(k) = vq_cmd;
    voltage_mode(k) = use_voltage;
end

log = struct('t', t, 'command_rpm', rpm_cmd, 'reference_rpm', ref, ...
             'rpm', rpm, 'rpm_meas', rpm_meas_log, ...
             'id_meas', id_meas_log, 'iq_meas', iq_meas_log, ...
             'iq_ref', iq_ref_log, 'vd', vd_log, 'vq', vq_log, ...
             'voltage_mode', voltage_mode);
end

function checks = evaluate_case(log, p)
targets = [10, 100, 150, 2600];
tol = [6, 6, 12, 120];
checks = repmat(struct('name', '', 'pass', false, 'value', 0, 'limit', 0), numel(targets)+4, 1);
n_segment = round(p.segment_s / p.dt);
for i = 1:numel(targets)
    segment_number = find(p.commands_rpm == targets(i), 1, 'first');
    first = (segment_number - 1) * n_segment + 1;
    segment = first:first + n_segment - 1;
    segment = segment(round(numel(segment) * 0.8):end);
    err = max(abs(log.rpm(segment) - targets(i)));
    checks(i) = make_check(sprintf('%g RPM 稳态最大误差', targets(i)), err <= tol(i), err, tol(i));
end
checks(5) = make_check('电压矢量不超过 9 V', ...
                       max(hypot(log.vd, log.vq)) <= p.v_limit + 1e-9, ...
                       max(hypot(log.vd, log.vq)), p.v_limit);
checks(6) = make_check('100 RPM 使用直接 Vq', ...
                       all(log.voltage_mode(log.command_rpm == 100)), 1, 1);
checks(7) = make_check('150 RPM 使用 Id/Iq 电流环', ...
                       ~any(log.voltage_mode(log.command_rpm == 150)), 1, 1);
% PLL 的职责就是滤掉ADC量化和转矩纹波，不能用“与瞬时机械速度的逐点最大差”验收。
% 分段比较稳态平均值，检查测速是否存在系统偏置；瞬时差只在报告中作为纹波诊断。
pll_mean_bias = 0.0;
for i = 1:numel(p.commands_rpm)
    segment = (i-1)*n_segment+1:i*n_segment;
    segment = segment(round(numel(segment)*0.8):end);
    pll_mean_bias = max(pll_mean_bias, ...
                        abs(mean(log.rpm_meas(segment))-mean(log.rpm(segment))));
end
checks(8) = make_check('编码器 PLL 稳态平均测速偏差', ...
                       pll_mean_bias <= 5.0, pll_mean_bias, 5.0);
end

function c = make_check(name, pass, value, limit)
c = struct('name', name, 'pass', logical(pass), 'value', value, 'limit', limit);
end

function write_report(path, result)
fid = fopen(path, 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
p = result.parameters;
fprintf(fid, '2804/BM75 hybrid speed-loop SIL\n');
fprintf(fid, 'FOC=%.0f Hz speed=%.0f Hz Vbus=%.1f V ADC=%.4f A/count encoder=%d CPR\n', ...
        1/p.dt, 1/p.speed_dt, p.vbus, p.adc_a_per_count, p.encoder_cpr);
fprintf(fid, 'encoder PLL=%.0f rad/s @ %.0f Hz\n', p.encoder_pll_bw, 1/p.encoder_dt);
fprintf(fid, 'backend: |target|<=%.0f RPM Vq, otherwise Id/Iq\n', p.hybrid_rpm);
fprintf(fid, 'speed PI kp=%.4f ki=%.4f iq_limit=%.2f A\n', ...
        p.speed_kp, p.speed_ki, p.speed_iq_limit);
fprintf(fid, 'feedforward: %.4f V + %.7f V/RPM\n\n', p.speed_ff_boost, p.speed_ff_slope);
fprintf(fid, 'flux: nameplate=%.7f Wb plant_from_Vq/RPM=%.7f Wb\n', p.nameplate_flux, p.flux);
fprintf(fid, 'ADC zero counts: %.3f %.3f %.3f\n\n', p.adc_zero_counts);
for i = 1:numel(result.checks)
    c = result.checks(i);
    fprintf(fid, '%s  %-34s value=%g limit=%g\n', pass_text(c.pass), c.name, c.value, c.limit);
end
steady = mod(result.hybrid.t, p.segment_s) >= 0.8*p.segment_s;
fprintf(fid, '\n诊断：PLL与瞬时机械速度最大差=%g RPM（含被PLL滤除的高频纹波，不作FAIL判据）\n', ...
        max(abs(result.hybrid.rpm_meas(steady)-result.hybrid.rpm(steady))));
end

function text = pass_text(pass)
if pass
    text = 'PASS';
else
    text = 'FAIL';
end
end

function make_plots(out_dir, hybrid, current_only)
fig = figure('Name', '2804 hybrid SIL', 'NumberTitle', 'off');
tiledlayout(3, 1);
nexttile;
plot(hybrid.t, hybrid.command_rpm, ':k', hybrid.t, hybrid.rpm, 'b', ...
     hybrid.t, hybrid.rpm_meas, 'c', current_only.t, current_only.rpm, 'r');
grid on; ylabel('RPM');
legend('命令', '混合后端真实速度', '混合后端PLL速度', '全程电流环', 'Location', 'best');
title('2804/BM75：低速 Vq + 中高速 Id/Iq');
nexttile;
plot(hybrid.t, hybrid.iq_ref, 'k', hybrid.t, hybrid.iq_meas, 'b');
grid on; ylabel('Iq (A)'); legend('目标', 'ADC量化反馈');
nexttile;
plot(hybrid.t, hybrid.vd, 'b', hybrid.t, hybrid.vq, 'r');
grid on; ylabel('电压 (V)'); xlabel('时间 (s)'); legend('Vd', 'Vq');
print(fig, fullfile(out_dir, 'sim_2804_hybrid_sil.png'), '-dpng', '-r140');
if ~usejava('desktop')
    close(fig);
end
end

function [ia, ib, ic] = dq_to_abc(id, iq, theta)
c = cos(theta);
s = sin(theta);
ialpha = id * c - iq * s;
ibeta = id * s + iq * c;
ia = ialpha;
ib = -0.5 * ialpha + sqrt(3)/2 * ibeta;
ic = -0.5 * ialpha - sqrt(3)/2 * ibeta;
end

function d = wrap_count(d, cpr)
if d > cpr/2
    d = d - cpr;
elseif d < -cpr/2
    d = d + cpr;
end
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end

function y = wrap_2pi(x)
y = mod(x, 2*pi);
end

function y = wrap_pi(x)
y = mod(x + pi, 2*pi) - pi;
end
