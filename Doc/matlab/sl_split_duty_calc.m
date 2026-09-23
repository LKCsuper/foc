function duty = sl_split_duty_calc(vd_req, vq_req, vbus)
%SL_SPLIT_DUTY_CALC  将dq电压请求换算为VESC式调制率绝对值。
c=foc.constants();
vlim=max(vbus,0.1)/sqrt(3)*c.MAX_DUTY;
duty=min(hypot(vd_req,vq_req)/max(vlim,1.0e-6),1.0)*c.MAX_DUTY;
end
