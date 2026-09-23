function [tA, tB, tC, sector] = foc_svm(alpha, beta, period)
%FOC_SVM  VESC 风格 SVPWM，输出 CCR。零电压为 ARR/2。对齐 motor_foc.c。
c = foc.constants();
period = uint32(period);
sector = pick_sector(alpha, beta, c);

switch sector
    case 1
        t1 = foc.to_uint32((alpha - c.ONE_BY_SQRT3 * beta) * double(period));
        t2 = foc.to_uint32((c.TWO_BY_SQRT3 * beta) * double(period));
        tA = u32_mid(period, t1, t2);
        tB = u32_sub(tA, t1);
        tC = u32_sub(tB, t2);
    case 2
        t2 = foc.to_uint32((alpha + c.ONE_BY_SQRT3 * beta) * double(period));
        t3 = foc.to_uint32((-alpha + c.ONE_BY_SQRT3 * beta) * double(period));
        tB = u32_mid(period, t2, t3);
        tA = u32_sub(tB, t3);
        tC = u32_sub(tA, t2);
    case 3
        t3 = foc.to_uint32((c.TWO_BY_SQRT3 * beta) * double(period));
        t4 = foc.to_uint32((-alpha - c.ONE_BY_SQRT3 * beta) * double(period));
        tB = u32_mid(period, t3, t4);
        tC = u32_sub(tB, t3);
        tA = u32_sub(tC, t4);
    case 4
        t4 = foc.to_uint32((-alpha + c.ONE_BY_SQRT3 * beta) * double(period));
        t5 = foc.to_uint32((-c.TWO_BY_SQRT3 * beta) * double(period));
        tC = u32_mid(period, t4, t5);
        tB = u32_sub(tC, t5);
        tA = u32_sub(tB, t4);
    case 5
        t5 = foc.to_uint32((-alpha - c.ONE_BY_SQRT3 * beta) * double(period));
        t6 = foc.to_uint32((alpha - c.ONE_BY_SQRT3 * beta) * double(period));
        tC = u32_mid(period, t5, t6);
        tA = u32_sub(tC, t5);
        tB = u32_sub(tA, t6);
    otherwise
        t6 = foc.to_uint32((-c.TWO_BY_SQRT3 * beta) * double(period));
        t1 = foc.to_uint32((alpha + c.ONE_BY_SQRT3 * beta) * double(period));
        tA = u32_mid(period, t6, t1);
        tC = u32_sub(tA, t1);
        tB = u32_sub(tC, t6);
end
end

function sector = pick_sector(alpha, beta, c)
if beta >= 0.0
    if alpha >= 0.0
        if c.ONE_BY_SQRT3 * beta > alpha
            sector = 2;
        else
            sector = 1;
        end
    else
        if -c.ONE_BY_SQRT3 * beta > alpha
            sector = 3;
        else
            sector = 2;
        end
    end
else
    if alpha >= 0.0
        if -c.ONE_BY_SQRT3 * beta > alpha
            sector = 5;
        else
            sector = 6;
        end
    else
        if c.ONE_BY_SQRT3 * beta > alpha
            sector = 4;
        else
            sector = 5;
        end
    end
end
end

function y = u32_mid(period, t1, t2)
s = mod(double(period) + double(t1) + double(t2), 2^32);
y = uint32(fix(s / 2.0));
end

function y = u32_sub(a, b)
y = uint32(mod(double(a) - double(b), 2^32));
end
