function [tA, tB, tC, sector] = svm_hqd(valpha, vbeta, udc, tpwm)
%SVM_HQD  慧驱动时间合成（含再除 2）。零电压约 ARR/4。对齐 motor_foc.c。
c = foc.constants();
tpwm = uint32(tpwm);
if udc <= 0.0 || tpwm == 0
    tA = uint32(0);
    tB = uint32(0);
    tC = uint32(0);
    sector = 0;
    return
end

vref1 = vbeta;
vref2 = (c.ONE_BY_SQRT3 * valpha - vbeta) / 2.0;
vref3 = (-c.ONE_BY_SQRT3 * valpha - vbeta) / 2.0;
sector = 0;
if vref1 > 0.0
    sector = sector + 1;
end
if vref2 > 0.0
    sector = sector + 2;
end
if vref3 > 0.0
    sector = sector + 4;
end

x = c.TWO_BY_SQRT3 * vbeta * double(tpwm) / udc;
y = double(tpwm) / udc * (valpha + c.ONE_BY_SQRT3 * vbeta);
z = double(tpwm) / udc * (-valpha + c.ONE_BY_SQRT3 * vbeta);

t1 = 0.0;
t2 = 0.0;
switch sector
    case 1
        t1 = z;
        t2 = y;
    case 2
        t1 = y;
        t2 = -x;
    case 3
        t1 = -z;
        t2 = x;
    case 4
        t1 = -x;
        t2 = z;
    case 5
        t1 = x;
        t2 = -y;
    case 6
        t1 = -y;
        t2 = -z;
end

if (t1 + t2) > double(tpwm)
    s = t1 + t2;
    t1 = t1 / s * double(tpwm);
    t2 = t2 / s * double(tpwm);
end

ta = (double(tpwm) - (t1 + t2)) / 4.0;
tb = ta + t1 / 2.0;
tc = tb + t2 / 2.0;

tA = uint32(0);
tB = uint32(0);
tC = uint32(0);
switch sector
    case 1
        tA = foc.to_uint32(tb);
        tB = foc.to_uint32(ta);
        tC = foc.to_uint32(tc);
    case 2
        tA = foc.to_uint32(ta);
        tB = foc.to_uint32(tc);
        tC = foc.to_uint32(tb);
    case 3
        tA = foc.to_uint32(ta);
        tB = foc.to_uint32(tb);
        tC = foc.to_uint32(tc);
    case 4
        tA = foc.to_uint32(tc);
        tB = foc.to_uint32(tb);
        tC = foc.to_uint32(ta);
    case 5
        tA = foc.to_uint32(tc);
        tB = foc.to_uint32(ta);
        tC = foc.to_uint32(tb);
    case 6
        tA = foc.to_uint32(tb);
        tB = foc.to_uint32(tc);
        tC = foc.to_uint32(ta);
end
end
