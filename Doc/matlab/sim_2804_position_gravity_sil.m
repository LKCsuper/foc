function result = sim_2804_position_gravity_sil(do_plot)
%SIM_2804_POSITION_GRAVITY_SIL  2804位置环与转子偏心重力扰动SIL。
%
% 工程图给出转子惯量 3700 g.mm^2 = 3.7e-6 kg.m^2、整机质量37 g。
% 偏心距离没有给出，因此只做参数扫描，不把扫描值当成实测结论。

if nargin < 1
    do_plot = usejava('desktop');
end
p = parameters();
ecc_mm = [0.0, 0.05, 0.10, 0.20];
cases = cell(size(ecc_mm));
for i = 1:numel(ecc_mm)
    cases{i} = run_case(p, ecc_mm(i) * 1e-3);
end

checks = evaluate_unloaded(cases{1}, p);
result = struct('parameters', p, 'eccentricity_mm', ecc_mm, ...
                'cases', {cases}, 'checks', checks);
out_dir = fullfile(fileparts(mfilename('fullpath')), 'out');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
save(fullfile(out_dir, 'sim_2804_position_gravity_sil.mat'), 'result');
write_report(fullfile(out_dir, 'sim_2804_position_gravity_sil.log'), result);
if do_plot, make_plots(out_dir, result); end

fprintf('2804 position/gravity SIL: %d PASS, %d FAIL\n', ...
        sum([checks.pass]), sum(~[checks.pass]));
if any(~[checks.pass])
    error('sim_2804_position_gravity_sil:checkFailed', 'position SIL acceptance failed');
end
end

function p = parameters()
profile = load_2804_profile();
p.dt = 1/15000;
p.position_div = 15;
p.position_dt = p.dt * p.position_div;
p.pole_pairs = profile.pole_pairs;
p.rs = profile.rs;
p.lq = profile.inductance;
p.flux = 0.0032564;
p.kt = 1.5 * p.pole_pairs * p.flux;
p.inertia = profile.inertia;
p.motor_mass_kg = 0.037;
p.gravity = 9.80665;
p.viscous = 1.0e-6;
p.coulomb_torque = p.kt * 0.0741 / p.rs;
p.encoder_cpr = 16384;
p.encoder_dt = 1/10000;
p.encoder_pll_bw = profile.encoder_pll_bw;
p.position_kp_rpm_per_deg = profile.position_kp;
p.maximum_rpm = profile.position_max_rpm;
p.velocity_kp_v_per_rpm = profile.position_vel_kp;
p.velocity_ki_v_per_rpm_s = profile.position_vel_ki;
p.voltage_limit = profile.position_voltage_limit;
p.accel_rpm_s = profile.position_accel;
p.commands_deg = [0, 2, 5, -2, 0];
% 对齐固件 pos test 8000 的每点观察时间；2 RPM 保守配置不能用高速测试的短窗口验收。
p.segment_s = 8.0;
end

function log = run_case(p, eccentricity_m)
n_segment = round(p.segment_s / p.dt);
n = n_segment * numel(p.commands_deg);
t = (0:n-1)' * p.dt;
target = repelem(p.commands_deg(:), n_segment);

theta_m = 0.0;
omega = 0.0;
iq = 0.0;
commanded_rpm = 0.0;
velocity_integrator = 0.0;
encoder_prev = 0;
turns = 0;
rpm_meas = 0.0;
pll_pos = 0.0;
pll_vel = 0.0;
encoder_time = 0.0;
vq = 0.0;

position_deg = zeros(n, 1);
rpm = zeros(n, 1);
rpm_meas_log = zeros(n, 1);
rpm_command_log = zeros(n, 1);
vq_log = zeros(n, 1);
gravity_torque = zeros(n, 1);

for k = 1:n
    % 对齐 Motor_Position_SetTargetDeg：新目标不能继承旧目标为克服齿槽积出的电压。
    if k == 1 || target(k) ~= target(k-1)
        commanded_rpm = 0.0;
        velocity_integrator = 0.0;
        vq = 0.0;
    end
    raw_count = round(theta_m / (2*pi) * p.encoder_cpr);
    count = mod(raw_count, p.encoder_cpr);
    encoder_time = encoder_time + p.dt;
    if encoder_time + 1e-12 >= p.encoder_dt
        encoder_time = encoder_time - p.encoder_dt;
        pll_kp = 2.0 * p.encoder_pll_bw;
        pll_ki = 0.25 * pll_kp * pll_kp;
        pll_pos = mod(pll_pos + p.encoder_dt * pll_vel, p.encoder_cpr);
        pll_error = wrap_count(count - pll_pos, p.encoder_cpr);
        pll_pos = mod(pll_pos + p.encoder_dt * pll_kp * pll_error, p.encoder_cpr);
        pll_vel = pll_vel + p.encoder_dt * pll_ki * pll_error;
        if abs(pll_vel) < 0.5 * p.encoder_dt * pll_ki
            pll_vel = 0.0;
        end
    end
    if mod(k-1, p.position_div) == 0
        dcount = encoder_delta(count, encoder_prev, p.encoder_cpr);
        encoder_prev = count;
        turns = turns + dcount / p.encoder_cpr;
        pos_deg = turns * 360.0;
        rpm_meas = pll_vel / p.encoder_cpr * 60;

        error_deg = target(k) - pos_deg;
        raw_target_rpm = clamp(p.position_kp_rpm_per_deg * error_deg, ...
                               -p.maximum_rpm, p.maximum_rpm);
        step = p.accel_rpm_s * p.position_dt;
        commanded_rpm = commanded_rpm + clamp(raw_target_rpm-commanded_rpm, -step, step);
        velocity_error = commanded_rpm - rpm_meas;
        proportional = p.velocity_kp_v_per_rpm * velocity_error;
        candidate = clamp(velocity_integrator + ...
                          p.velocity_ki_v_per_rpm_s * velocity_error * p.position_dt, ...
                          -p.voltage_limit, p.voltage_limit);
        unsaturated = proportional + candidate;
        winding_up = (unsaturated > p.voltage_limit && velocity_error > 0) || ...
                     (unsaturated < -p.voltage_limit && velocity_error < 0);
        if ~winding_up, velocity_integrator = candidate; end
        vq = clamp(proportional + velocity_integrator, -p.voltage_limit, p.voltage_limit);
    end

    omega_e = omega * p.pole_pairs;
    iq = iq + p.dt / p.lq * (vq - p.rs*iq - omega_e*p.flux);
    tau_g = p.motor_mass_kg * p.gravity * eccentricity_m * sin(theta_m);
    tau_f = p.viscous * omega;
    if abs(omega) > 1e-3
        tau_f = tau_f + p.coulomb_torque * sign(omega);
    elseif abs(p.kt*iq - tau_g) <= p.coulomb_torque
        tau_f = p.kt*iq - tau_g;
    else
        tau_f = p.coulomb_torque * sign(p.kt*iq - tau_g);
    end
    omega = omega + p.dt / p.inertia * (p.kt*iq - tau_f - tau_g);
    theta_m = theta_m + omega*p.dt;

    position_deg(k) = theta_m * 180/pi;
    rpm(k) = omega * 60/(2*pi);
    rpm_meas_log(k) = rpm_meas;
    rpm_command_log(k) = commanded_rpm;
    vq_log(k) = vq;
    gravity_torque(k) = tau_g;
end
log = struct('t', t, 'target_deg', target, 'position_deg', position_deg, ...
             'rpm', rpm, 'rpm_meas', rpm_meas_log, ...
             'rpm_command', rpm_command_log, 'vq', vq_log, ...
             'gravity_torque_nm', gravity_torque, ...
             'eccentricity_m', eccentricity_m);
end

function checks = evaluate_unloaded(log, p)
n_segment = round(p.segment_s/p.dt);
checks = repmat(struct('name','','pass',false,'value',0,'limit',0), ...
                numel(p.commands_deg)+2, 1);
for i = 1:numel(p.commands_deg)
    idx = (i-1)*n_segment+1:i*n_segment;
    idx = idx(round(0.8*numel(idx)):end);
    err = max(abs(log.position_deg(idx)-p.commands_deg(i)));
    checks(i) = check(sprintf('目标 %g deg 稳态误差', p.commands_deg(i)), err <= 0.75, err, 0.75);
end
checks(end-1) = check('位置输出不超过0.35 V', max(abs(log.vq)) <= 0.350001, max(abs(log.vq)), 0.35);
checks(end) = check('运动速度不超过保守范围', max(abs(log.rpm)) <= 3.5, max(abs(log.rpm)), 3.5);
end

function c = check(name, pass, value, limit)
c = struct('name',name,'pass',logical(pass),'value',value,'limit',limit);
end

function write_report(path, result)
fid = fopen(path,'w'); cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
p = result.parameters;
fprintf(fid,'2804 position + eccentric gravity SIL\n');
fprintf(fid,'J=%.3g kg.m^2 mass=%.3f kg position loop=%.0f Hz\n', ...
        p.inertia,p.motor_mass_kg,1/p.position_dt);
fprintf(fid,'position kp=%.3f rpm/deg max=%.1f rpm velPI=%.3f/%.3f Vlimit=%.2f V\n\n', ...
        p.position_kp_rpm_per_deg,p.maximum_rpm,p.velocity_kp_v_per_rpm, ...
        p.velocity_ki_v_per_rpm_s,p.voltage_limit);
for i=1:numel(result.checks)
    c=result.checks(i); fprintf(fid,'%s  %-32s value=%g limit=%g\n',tf(c.pass),c.name,c.value,c.limit);
end
fprintf(fid,'\n偏心敏感性（非实测偏心）：\n');
for i=1:numel(result.cases)
    q=result.cases{i};
    fprintf(fid,'ecc=%5.2f mm  gravity_peak=%8.6f Nm  max_steady_error=%7.3f deg\n', ...
        result.eccentricity_mm(i),max(abs(q.gravity_torque_nm)), ...
        maximum_steady_error(q,p));
end
end

function e = maximum_steady_error(log,p)
n_segment=round(p.segment_s/p.dt);
e=0.0;
for j=1:numel(p.commands_deg)
    idx=(j-1)*n_segment+1:j*n_segment;
    idx=idx(round(0.8*numel(idx)):end);
    e=max(e,max(abs(log.position_deg(idx)-p.commands_deg(j))));
end
end

function s=tf(x)
if x, s='PASS'; else, s='FAIL'; end
end

function make_plots(out_dir,result)
fig=figure('Name','2804 position gravity SIL','NumberTitle','off');
tiledlayout(3,1);
nexttile; plot(result.cases{1}.t,result.cases{1}.target_deg,':k','LineWidth',1.1); hold on;
for i=1:numel(result.cases), plot(result.cases{i}.t,result.cases{i}.position_deg,'LineWidth',1.0); end
grid on; ylabel('位置 (deg)');
legend(['目标',compose('偏心 %.2f mm',result.eccentricity_mm)],'Location','best');
title('2804位置环：偏心重力扰动敏感性');
nexttile;
plot(result.cases{1}.t,result.cases{1}.rpm_command,':k','LineWidth',1.1); hold on;
plot(result.cases{1}.t,result.cases{1}.rpm,'b','LineWidth',1.0);
plot(result.cases{1}.t,result.cases{1}.rpm_meas,'c','LineWidth',1.0);
grid on; ylabel('RPM');
legend('目标速度','真实速度','PLL速度','Location','best');
nexttile;
for i=1:numel(result.cases), plot(result.cases{i}.t,result.cases{i}.vq,'LineWidth',1.0); hold on; end
grid on; ylabel('Vq (V)'); xlabel('时间 (s)');
print(fig,fullfile(out_dir,'sim_2804_position_gravity_sil.png'),'-dpng','-r140');
if ~usejava('desktop'),close(fig);end
end

function d=encoder_delta(now,previous,cpr)
d=now-previous;
if d>cpr/2,d=d-cpr;elseif d<-cpr/2,d=d+cpr;end
end

function d=wrap_count(d,cpr)
if d>cpr/2,d=d-cpr;elseif d<-cpr/2,d=d+cpr;end
end

function y=clamp(x,lo,hi)
y=min(max(x,lo),hi);
end
