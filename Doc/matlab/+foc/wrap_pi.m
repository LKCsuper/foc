function x = wrap_pi(x)
%WRAP_PI  对齐 utils_norm_angle_rad：映射到 [-pi, pi)。
x = double(x);
if ~isfinite(x)
    x = 0.0;
    return
end
two_pi = 2.0 * pi;
x = mod(x + pi, two_pi) - pi;
end
