function [tA, tB, tC, sector] = foc_svm_from_voltage(v_alpha, v_beta, v_bus, max_duty, period)
%FOC_SVM_FROM_VOLTAGE  VESC mcpwm_foc：mod=1.5*v/Vbus，限幅后 foc_svm。
c = foc.constants();
period = uint32(period);
mid = idivide(period, uint32(2));
if v_bus <= 0.0 || period == 0
    tA = mid;
    tB = mid;
    tC = mid;
    sector = 1;
    return
end
if max_duty <= 0.0
    max_duty = 1.0;
elseif max_duty > 1.0
    max_duty = 1.0;
end
mod_alpha = v_alpha * c.VOLTAGE_SCALE / v_bus;
mod_beta = v_beta * c.VOLTAGE_SCALE / v_bus;
[mod_alpha, mod_beta] = foc.saturate_vector_2d(mod_alpha, mod_beta, c.SQRT3_BY_2 * max_duty);
[tA, tB, tC, sector] = foc.foc_svm(mod_alpha, mod_beta, period);
end
