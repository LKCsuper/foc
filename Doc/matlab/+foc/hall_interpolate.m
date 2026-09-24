function theta = hall_interpolate(sector_center, rpm, pole_pairs, sector_timer_s, direction)
%HALL_INTERPOLATE  扇区内插值，150 RPM 钉中心，300 RPM 满插值。对齐 motor_hall_angle.c。
c = foc.constants();
theta = foc.wrap_pi(sector_center);
if direction == 0 || abs(rpm) < 1.0
    return
end
rpm_abs = abs(rpm);
if rpm_abs <= c.INTERP_START_RPM
    return
end
blend = (rpm_abs - c.INTERP_START_RPM) / (c.INTERP_FULL_RPM - c.INTERP_START_RPM);
if blend > 1.0
    blend = 1.0;
end
half = pi / 6.0;
progress = rpm_abs * pole_pairs * (2.0 * pi / 60.0) * sector_timer_s;
if progress > (2.0 * half)
    progress = 2.0 * half;
end
delta = double(direction) * (progress - half);
theta = foc.wrap_pi(sector_center + blend * delta);
end
