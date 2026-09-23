function [rpm_cmd, align_on, id_align] = sl_split_command(t, rpm_target, rpm_ramp)
%SL_SPLIT_COMMAND  对齐阶段与转速斜坡给定。
persistent rpm_z
c = foc.constants();
if isempty(rpm_z), rpm_z = 0.0; end
align_on = double(t < c.HFI_FLUX_ALIGN_S);
id_align = align_on * c.IQ_IF;
if align_on > 0.5
    rpm_z = 0.0;
else
    step = abs(rpm_ramp) * c.DT;
    if rpm_z < rpm_target, rpm_z = min(rpm_z + step, rpm_target);
    elseif rpm_z > rpm_target, rpm_z = max(rpm_z - step, rpm_target);
    end
end
rpm_cmd = rpm_z;
end
