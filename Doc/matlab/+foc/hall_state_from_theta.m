function state = hall_state_from_theta(theta, table, seq)
%HALL_STATE_FROM_THETA  真电角落入哪个标定扇区（邻中心中点为边界）。
n = numel(seq);
centers = zeros(n, 1);
for i = 1:n
    centers(i) = foc.hall_decode(table, seq(i));
end
best = 1;
best_abs = inf;
for i = 1:n
    prev = centers(mod(i - 2, n) + 1);
    nxt = centers(mod(i, n) + 1);
    half_neg = 0.5 * abs(foc.wrap_pi(centers(i) - prev));
    half_pos = 0.5 * abs(foc.wrap_pi(nxt - centers(i)));
    d = foc.wrap_pi(theta - centers(i));
    if d >= -half_neg && d < half_pos
        state = seq(i);
        return
    end
    if abs(d) < best_abs
        best_abs = abs(d);
        best = i;
    end
end
state = seq(best);
end
