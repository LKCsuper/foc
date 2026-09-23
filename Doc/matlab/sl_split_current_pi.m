function [va,vb,vc,vd_req,vq_req,id,iq] = sl_split_current_pi(ia,ib,ic,theta_deg,id_ref,iq_ref,vbus)
%SL_SPLIT_CURRENT_PI  Park、d/q PI、矢量限幅与逆Park。
persistent vd_int vq_int
c=foc.constants(); if isempty(vd_int), vd_int=0.0; vq_int=0.0; end
[ialpha,ibeta]=foc.clarke(ia,ib,ic); th=foc.wrap_pi(theta_deg*pi/180);
[id,iq]=foc.park(ialpha,ibeta,th); vlim=max(vbus,0.1)/sqrt(3)*c.MAX_DUTY;
[vd_req,vd_int]=foc.current_pi(id_ref,id,vd_int,c.KP,c.KI,c.DT,vlim);
[vq_req,vq_int]=foc.current_pi(iq_ref,iq,vq_int,c.KP,c.KI,c.DT,vlim);
[vd,vq]=foc.saturate_vector_2d(vd_req,vq_req,vlim);
[a,b]=foc.inverse_park(vd,vq,th); [va,vb,vc]=foc.inverse_clarke(-a,-b);
end
