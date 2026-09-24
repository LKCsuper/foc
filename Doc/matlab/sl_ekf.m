function [th_ekf_deg, rpm_ekf, speed_ok] = sl_ekf(valpha, vbeta, ialpha, ibeta)
%SL_EKF  与固件一致：上一拍 inverse_park 的 vαvβ + 本拍 Clarke 后的 iαiβ。
persistent x p inited th_unw th_prev
c = foc.constants();
dt = c.DT;
R = c.R_LINE;
L = c.L_LINE;
flux = c.FLUX;
pp = c.POLE_PAIRS;
if isempty(inited)
    x = [ialpha; ibeta; 0.0; 0.0];
    p = diag([1.0e-4, 1.0e-4, 25.0, 0.02]);
    inited = true;
    th_unw = 0.0;
    th_prev = 0.0;
end
om = x(3);
th = x(4);
s_th = sin(th);
c_th = cos(th);
inv_l = 1.0 / L;
F = zeros(4, 4);
F(1, 1) = 1.0 - dt * R * inv_l;
F(1, 3) = dt * flux * inv_l * s_th;
F(1, 4) = dt * flux * inv_l * om * c_th;
F(2, 2) = 1.0 - dt * R * inv_l;
F(2, 3) = -dt * flux * inv_l * c_th;
F(2, 4) = dt * flux * inv_l * om * s_th;
F(3, 3) = 1.0;
F(4, 3) = dt;
F(4, 4) = 1.0;
x(1) = x(1) + dt * inv_l * (valpha - R * x(1) + om * flux * s_th);
x(2) = x(2) + dt * inv_l * (vbeta - R * x(2) - om * flux * c_th);
x(4) = foc.wrap_pi(x(4) + om * dt);
ppd = F * p * F.';
ppd(1, 1) = ppd(1, 1) + 1.0e-3;
ppd(2, 2) = ppd(2, 2) + 1.0e-3;
ppd(3, 3) = ppd(3, 3) + 80.0;
ppd(4, 4) = ppd(4, 4) + 1.0e-4;
r_i = 5.0e-4;
s00 = ppd(1, 1) + r_i;
s01 = ppd(1, 2);
s11 = ppd(2, 2) + r_i;
det = s00 * s11 - s01 * s01;
if ~(det > 1.0e-12)
    x = [ialpha; ibeta; 0.0; 0.0];
    p = diag([1.0e-4, 1.0e-4, 25.0, 0.02]);
    th_unw = 0.0;
    th_prev = 0.0;
    th_ekf_deg = 0.0;
    rpm_ekf = 0.0;
    speed_ok = 0.0;
    return
end
inv_det = 1.0 / det;
K = zeros(4, 2);
for i = 1:4
    p0 = ppd(i, 1);
    p1 = ppd(i, 2);
    K(i, 1) = (p0 * s11 - p1 * s01) * inv_det;
    K(i, 2) = (-p0 * s01 + p1 * s00) * inv_det;
end
x = x + K * [ialpha - x(1); ibeta - x(2)];
A = eye(4);
A(:, 1) = A(:, 1) - K(:, 1);
A(:, 2) = A(:, 2) - K(:, 2);
p = A * ppd * A.' + r_i * (K * K.');
p = 0.5 * (p + p.');
x(4) = foc.wrap_pi(x(4));
x(3) = max(min(x(3), 1500.0), -1500.0);
dth = foc.wrap_pi(x(4) - th_prev);
th_unw = th_unw + dth * 180.0 / pi;
th_prev = x(4);
th_ekf_deg = th_unw;
rpm_ekf = x(3) / pp * 60.0 / (2.0 * pi);
speed_ok = double(abs(x(3)) > 50.0);
end
