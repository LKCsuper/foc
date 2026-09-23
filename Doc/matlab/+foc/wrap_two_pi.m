function x = wrap_two_pi(x)
%WRAP_TWO_PI  对齐 Motor_Hfi_WrapTwoPi：映射到 [0, 2π)。
x = double(x);
if ~isfinite(x)
    x = 0.0;
    return
end
two_pi = 2.0 * pi;
x = mod(x, two_pi);
if x < 0.0
    x = x + two_pi;
end
end
