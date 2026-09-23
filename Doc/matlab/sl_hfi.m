function [th_hfi_deg, rpm_hfi, va_inj, vb_inj, vc_inj] = sl_hfi(ialpha, ibeta, seed_rad, hold_angle)
%SL_HFI  对齐 motor_hfi.c 脉振方波跟踪：估计 d 轴 ±Vh，ΔIq 解调凸极误差。
%
%  影子模式时 I/F 基波约 0.6 A 会进估计 q 轴。先把 αβ 经 200 Hz 高通再 Park，
%  只留 7.5 kHz 注入响应，否则 ΔIq 被起步电流淹没、角卡在 Seed。
%  增益按真凸极 ΔIq 缩小；转速再经 10 Hz 低通，避免起步尖峰把 RPM 坐标拉飞。
persistent inited running sample_valid polarity angle speed_e prev_iq ...
    ia_lp ib_lp err_f rpm_f th_unw th_prev
c = foc.constants();
dt = c.DT;
pp = c.POLE_PAIRS;
vh = c.HFI_VOLTAGE;
gain = c.HFI_GAIN;
v_bus = c.VBUS;
ratio = c.HFI_BUS_RATIO;
a_w = c.HFI_SPEED_LPF;
a_hp = dt * 2.0 * pi * 200.0;
if a_hp > 1.0
    a_hp = 1.0;
end
a_rpm = dt * 2.0 * pi * c.SL_PLL_SPD_HZ;
if a_rpm > 1.0
    a_rpm = 1.0;
end
if nargin < 3
    seed_rad = c.HFI_SEED;
end
if nargin < 4
    hold_angle = 0.0;
end

if isempty(inited)
    running = false;
    sample_valid = false;
    polarity = 1.0;
    angle = foc.wrap_two_pi(seed_rad);
    speed_e = 0.0;
    prev_iq = 0.0;
    ia_lp = 0.0;
    ib_lp = 0.0;
    err_f = 0.0;
    rpm_f = 0.0;
    th_unw = angle * 180.0 / pi;
    th_prev = foc.wrap_pi(angle);
    inited = true;
end

ia_lp = ia_lp + a_hp * (ialpha - ia_lp);
ib_lp = ib_lp + a_hp * (ibeta - ib_lp);
ia_hf = ialpha - ia_lp;
ib_hf = ibeta - ib_lp;

cs = cos(angle);
sn = sin(angle);
iq = -ia_hf * sn + ib_hf * cs;

if ~running
    running = true;
    sample_valid = false;
    polarity = 1.0;
    prev_iq = 0.0;
    speed_e = 0.0;
end

if hold_angle > 0.5
    angle = foc.wrap_two_pi(seed_rad);
    speed_e = 0.0;
    rpm_f = 0.0;
    if sample_valid
        polarity = -polarity;
    else
        polarity = 1.0;
    end
elseif sample_valid
    % 离散植物实测：Ld<Lq 时此符号使真实 d 轴稳定；负号会锁到 ±90°。
    err = (iq - prev_iq) * polarity;
    err_f = err_f + 0.08 * (err - err_f);
    dth_corr = gain * err_f;
    if dth_corr > 0.04
        dth_corr = 0.04;
    elseif dth_corr < -0.04
        dth_corr = -0.04;
    end
    prev_angle = angle;
    angle = foc.wrap_two_pi(angle + dth_corr);
    dth = foc.wrap_pi(angle - prev_angle);
    if dt > 0.0 && isfinite(dt)
        speed_e = speed_e + a_w * (dth / dt - speed_e);
    end
    polarity = -polarity;
else
    polarity = 1.0;
end

prev_iq = iq;
sample_valid = true;

voltage = vh;
if v_bus > 0.0 && voltage > v_bus * ratio
    voltage = v_bus * ratio;
end
vd_inj = polarity * voltage;

[va_h, vb_h] = foc.inverse_park(vd_inj, 0.0, angle);
% foc.inverse_park 为对齐固件功率板已内含负号；数学植物前在这里抵消。
[va_inj, vb_inj, vc_inj] = foc.inverse_clarke(-va_h, -vb_h);

dth_sc = foc.wrap_pi(foc.wrap_pi(angle) - th_prev);
th_unw = th_unw + dth_sc * 180.0 / pi;
th_prev = foc.wrap_pi(angle);
th_hfi_deg = th_unw;
rpm_raw = speed_e / pp * 60.0 / (2.0 * pi);
rpm_f = rpm_f + a_rpm * (rpm_raw - rpm_f);
rpm_hfi = rpm_f;
end
