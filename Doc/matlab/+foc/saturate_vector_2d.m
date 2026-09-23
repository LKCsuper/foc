function [x, y, limited] = saturate_vector_2d(x, y, max_mag)
%SATURATE_VECTOR_2D  对齐 utils_saturate_vector_2d。
max_mag = abs(double(max_mag));
mag = hypot(double(x), double(y));
if mag < 1e-10
    mag = 1e-10;
end
limited = mag > max_mag;
if limited
    f = max_mag / mag;
    x = x * f;
    y = y * f;
end
end
