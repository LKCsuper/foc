function build_hfi_flux_split_simulink()
%BUILD_HFI_FLUX_SPLIT_SIMULINK  将输入、估算、PI、弱磁和植物拆成独立块。
this_dir=fileparts(mfilename('fullpath')); addpath(this_dir);
mdl='hfi_flux_split_pmsm'; slx=fullfile(this_dir,[mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist(slx,'file'), delete(slx); end
new_system(mdl);
set_param(mdl,'Solver','FixedStepDiscrete','FixedStep','1/15000', ...
    'StartTime','0','StopTime','16.0', ...
    'PreLoadFcn',sprintf('addpath(''%s'');',strrep(this_dir,'''','''''')), ...
    'StartFcn',['clear sl_split_command sl_split_angle_manager sl_split_speed_pi sl_split_speed_pi_stable ' ...
    'sl_split_fw sl_split_duty_calc sl_split_duty_lpf sl_split_fw_duty_map ' ...
    'sl_split_fw_backoff sl_split_fw_ramp sl_split_current_circle sl_split_hfi_observer_sync sl_split_angle_errors ' ...
    'sl_split_encoder_feedback sl_split_hall_feedback sl_split_feedback_select ' ...
    'sl_split_current_pi sl_hfi sl_flux sl_pmsm_discrete;']);

% 可直接双击修改的外部输入。
constant(mdl,'Vbus_24V','24.0',[20 35 90 65]);
constant(mdl,'Target_RPM','3000.0',[20 85 90 115]);
constant(mdl,'Ramp_RPM_s','800.0',[20 135 90 165]);
constant(mdl,'FW_Mode_VESC','0.0',[20 235 90 265]);
constant(mdl,'Speed_Mode_VESC','0.0',[20 285 90 315]);
constant(mdl,'HFI_Sync_Enable','0.0',[20 335 90 365]);
constant(mdl,'Encoder_Enable','0.0',[20 385 90 415]);
constant(mdl,'Encoder_Mode','0.0',[20 435 90 465]);
constant(mdl,'Hall_Enable','0.0',[20 485 90 515]);
constant(mdl,'Encoder_Fault','0.0',[20 535 90 565]);
constant(mdl,'Hall_Fault','0.0',[20 585 90 615]);
add_block('simulink/Sources/Digital Clock',[mdl '/Time'], ...
    'SampleTime','1/15000','Position',[20 185 90 215]);

mf(mdl,'Command_Ramp',[
    "function [rpm_cmd,align,id_align]=fcn(t,target,ramp)" newline ...
    "[rpm_cmd,align,id_align]=sl_split_command(t,target,ramp);" newline "end"], ...
    [145 80 285 155],'yellow');
mf(mdl,'Clarke_Current',[
    "function [ialpha,ibeta]=fcn(ia,ib,ic)" newline ...
    "[ialpha,ibeta]=foc.clarke(ia,ib,ic);" newline "end"], ...
    [145 280 285 345],'lightBlue');
mf(mdl,'HFI_Estimator',[
    "function [th,rpm,va,vb,vc]=fcn(ialpha,ibeta,align)" newline ...
    "[th,rpm,va,vb,vc]=sl_hfi(ialpha,ibeta,0.0,align);" newline "end"], ...
    [345 240 500 345],'cyan');
mf(mdl,'Flux_Observer',[
    "function [th,rpm,ok]=fcn(va,vb,ialpha,ibeta,rpm_seed,th_seed)" newline ...
    "[th,rpm,ok]=sl_flux(va,vb,ialpha,ibeta,rpm_seed,th_seed);" newline "end"], ...
    [345 390 515 475],'green');
mf(mdl,'HFI_Observer_Sync',[
    "function [th,rpm,err,ofs]=fcn(th_h,rpm_h,th_f,rpm_f,ok,align,en)" newline ...
    "[th,rpm,err,ofs]=sl_split_hfi_observer_sync(th_h,rpm_h,th_f,rpm_f,ok,align,en);" newline "end"], ...
    [555 400 735 500],'lightBlue');
mf(mdl,'Angle_Error_Diagnostics',[
    "function [eh,ef,es]=fcn(th_h,th_f,th_s,th_true)" newline ...
    "[eh,ef,es]=sl_split_angle_errors(th_h,th_f,th_s,th_true);" newline "end"], ...
    [555 535 735 610],'yellow');
mf(mdl,'Angle_Manager',[
    "function [th,rpm,use_flux,inj_on]=fcn(th_h,rpm_h,th_f,rpm_f,ok,align,fw_id)" newline ...
    "[th,rpm,use_flux,inj_on]=sl_split_angle_manager(th_h,rpm_h,th_f,rpm_f,ok,align,fw_id);" newline "end"], ...
    [570 245 750 355],'magenta');
mf(mdl,'Encoder_Feedback',[
    "function [th,rpm]=fcn(th_true,rpm_true)" newline ...
    "[th,rpm]=sl_split_encoder_feedback(th_true,rpm_true);" newline "end"], ...
    [805 535 955 600],'green');
mf(mdl,'Hall_Feedback',[
    "function [th,rpm]=fcn(th_true,rpm_true)" newline ...
    "[th,rpm]=sl_split_hall_feedback(th_true,rpm_true);" newline "end"], ...
    [805 625 955 690],'cyan');
mf(mdl,'Feedback_Manager',[
    "function [th,rpm,src,fb,w]=fcn(th_sl,rpm_sl,th_e,rpm_e,th_h,rpm_h,en_e,en_h,flt_e,flt_h,align,enc_mode)" newline ...
    "[th,rpm,src,fb,w]=sl_split_feedback_select(th_sl,rpm_sl,th_e,rpm_e,th_h,rpm_h,en_e,en_h,flt_e,flt_h,align,enc_mode);" newline "end"], ...
    [1000 510 1190 690],'yellow');
mf(mdl,'Speed_PI_Stable',[
    "function iq_ref=fcn(cmd,rpm,align)" newline ...
    "iq_ref=sl_split_speed_pi_stable(cmd,rpm,align);" newline "end"], ...
    [805 75 930 130],'green');
mf(mdl,'Speed_PID_VESC',[
    "function iq_ref=fcn(cmd,rpm,align)" newline ...
    "iq_ref=sl_split_speed_pi(cmd,rpm,align);" newline "end"], ...
    [805 5 930 60],'orange');
add_block('simulink/Signal Routing/Switch',[mdl '/Speed_Mode_Select'], ...
    'Threshold','0.5','Criteria','u2 >= Threshold', ...
    'Position',[965 65 1015 125],'BackgroundColor','yellow');
mf(mdl,'Duty_Calculation',[
    "function duty=fcn(vd_z,vq_z,vbus)" newline ...
    "duty=sl_split_duty_calc(vd_z,vq_z,vbus);" newline "end"], ...
    [805 190 930 245],'lightBlue');
mf(mdl,'Duty_LPF',[
    "function duty_f=fcn(duty)" newline ...
    "duty_f=sl_split_duty_lpf(duty);" newline "end"], ...
    [965 190 1070 235],'green');
mf(mdl,'FW_Duty_Map',[
    "function i_fw=fcn(duty_f,use_flux)" newline ...
    "i_fw=sl_split_fw_duty_map(duty_f,use_flux);" newline "end"], ...
    [1105 190 1235 250],'red');
mf(mdl,'FW_Iq_Backoff',[
    "function i_fw=fcn(cmd,iq,iq_ref,rpm)" newline ...
    "i_fw=sl_split_fw_backoff(cmd,iq,iq_ref,rpm);" newline "end"], ...
    [1270 190 1415 275],'orange');
mf(mdl,'FW_Ramp',[
    "function id_fw=fcn(i_fw)" newline ...
    "id_fw=sl_split_fw_ramp(i_fw);" newline "end"], ...
    [1450 205 1560 250],'red');
mf(mdl,'FW_Voltage_PI_Stable',[
    "function id_fw=fcn(vd_z,vq_z,iq_ref,use_flux,vbus)" newline ...
    "id_fw=sl_split_fw(vd_z,vq_z,iq_ref,use_flux,vbus);" newline "end"], ...
    [1105 455 1280 540],'green');
add_block('simulink/Signal Routing/Switch',[mdl '/FW_Mode_Select'], ...
    'Threshold','0.5','Criteria','u2 >= Threshold', ...
    'Position',[1510 315 1560 375],'BackgroundColor','yellow');
add_block('simulink/Math Operations/Add',[mdl '/Id_Align_plus_FW'], ...
    'Inputs','++','Position',[1590 205 1620 250]);
mf(mdl,'IdIq_Current_Circle',[
    "function [id,iq]=fcn(id_raw,iq_raw)" newline ...
    "[id,iq]=sl_split_current_circle(id_raw,iq_raw);" newline "end"], ...
    [1650 175 1790 245],'yellow');
mf(mdl,'DQ_Current_PI',[
    "function [va,vb,vc,vd_req,vq_req,id,iq]=fcn(ia,ib,ic,theta,id_ref,iq_ref,vbus)" newline ...
    "[va,vb,vc,vd_req,vq_req,id,iq]=sl_split_current_pi(ia,ib,ic,theta,id_ref,iq_ref,vbus);" newline "end"], ...
    [1830 130 2010 290],'lightBlue');

% HFI注入门控和基波叠加。
for k=1:3
    product(mdl,sprintf('HFI_Gate_%d',k),[2050 300+45*k 2080 330+45*k]);
    add_block('simulink/Math Operations/Add',[mdl sprintf('/Voltage_Sum_%d',k)], ...
        'Inputs','++','Position',[2120 145+55*k 2150 175+55*k]);
    delay(mdl,sprintf('z_Vphase_%d',k),[2190 145+55*k 2230 175+55*k]);
end
mf(mdl,'Voltage_Clarke',[
    "function [valpha,vbeta]=fcn(va,vb,vc)" newline ...
    "[valpha,vbeta]=foc.clarke(va,vb,vc);" newline "end"], ...
    [2270 350 2400 415],'lightBlue');
delay(mdl,'z_Valpha',[2440 350 2480 380]); delay(mdl,'z_Vbeta',[2440 395 2480 425]);
delay(mdl,'z_Vd_req',[2050 80 2090 110]); delay(mdl,'z_Vq_req',[2050 120 2090 150]);
% 控制器之间的状态必须显式延迟一拍，MATLAB Function 持久状态不能处于代数环中。
delay(mdl,'z_RPM_used',[770 105 800 135]);
delay(mdl,'z_Iq_ref_FW',[965 120 995 150]);
delay(mdl,'z_Use_flux',[770 300 800 330]);
delay(mdl,'z_FW_id_angle',[1590 285 1630 315]);
delay(mdl,'z_Iq_meas_FW',[1270 300 1310 330]);
mf(mdl,'PMSM_Discrete',[
    "function [ia,ib,ic,rpm,th,te]=fcn(va,vb,vc)" newline ...
    "[ia,ib,ic,rpm,th,te]=sl_pmsm_discrete(va,vb,vc);" newline "end"], ...
    [2280 150 2440 285],'gray');

% 输入和主控制链。
wire(mdl,'Time/1','Command_Ramp/1'); wire(mdl,'Target_RPM/1','Command_Ramp/2');
wire(mdl,'Ramp_RPM_s/1','Command_Ramp/3');
wire(mdl,'PMSM_Discrete/1','Clarke_Current/1'); wire(mdl,'PMSM_Discrete/2','Clarke_Current/2'); wire(mdl,'PMSM_Discrete/3','Clarke_Current/3');
wire(mdl,'Clarke_Current/1','HFI_Estimator/1'); wire(mdl,'Clarke_Current/2','HFI_Estimator/2'); wire(mdl,'Command_Ramp/2','HFI_Estimator/3');
wire(mdl,'z_Valpha/1','Flux_Observer/1'); wire(mdl,'z_Vbeta/1','Flux_Observer/2');
wire(mdl,'Clarke_Current/1','Flux_Observer/3'); wire(mdl,'Clarke_Current/2','Flux_Observer/4');
wire(mdl,'HFI_Estimator/2','Flux_Observer/5'); wire(mdl,'HFI_Estimator/1','Flux_Observer/6');
wire(mdl,'HFI_Estimator/1','HFI_Observer_Sync/1'); wire(mdl,'HFI_Estimator/2','HFI_Observer_Sync/2');
wire(mdl,'Flux_Observer/1','HFI_Observer_Sync/3'); wire(mdl,'Flux_Observer/2','HFI_Observer_Sync/4'); wire(mdl,'Flux_Observer/3','HFI_Observer_Sync/5'); wire(mdl,'Command_Ramp/2','HFI_Observer_Sync/6');
wire(mdl,'HFI_Sync_Enable/1','HFI_Observer_Sync/7');
wire(mdl,'HFI_Estimator/1','Angle_Error_Diagnostics/1'); wire(mdl,'Flux_Observer/1','Angle_Error_Diagnostics/2');
wire(mdl,'HFI_Observer_Sync/1','Angle_Error_Diagnostics/3'); wire(mdl,'PMSM_Discrete/5','Angle_Error_Diagnostics/4');
wire(mdl,'HFI_Estimator/1','Angle_Manager/1'); wire(mdl,'HFI_Estimator/2','Angle_Manager/2');
wire(mdl,'HFI_Observer_Sync/1','Angle_Manager/3'); wire(mdl,'HFI_Observer_Sync/2','Angle_Manager/4'); wire(mdl,'Flux_Observer/3','Angle_Manager/5');
wire(mdl,'Command_Ramp/2','Angle_Manager/6');
wire(mdl,'FW_Mode_Select/1','z_FW_id_angle/1'); wire(mdl,'z_FW_id_angle/1','Angle_Manager/7');
wire(mdl,'PMSM_Discrete/5','Encoder_Feedback/1'); wire(mdl,'PMSM_Discrete/4','Encoder_Feedback/2');
wire(mdl,'PMSM_Discrete/5','Hall_Feedback/1'); wire(mdl,'PMSM_Discrete/4','Hall_Feedback/2');
wire(mdl,'Angle_Manager/1','Feedback_Manager/1'); wire(mdl,'Angle_Manager/2','Feedback_Manager/2');
wire(mdl,'Encoder_Feedback/1','Feedback_Manager/3'); wire(mdl,'Encoder_Feedback/2','Feedback_Manager/4');
wire(mdl,'Hall_Feedback/1','Feedback_Manager/5'); wire(mdl,'Hall_Feedback/2','Feedback_Manager/6');
wire(mdl,'Encoder_Enable/1','Feedback_Manager/7'); wire(mdl,'Hall_Enable/1','Feedback_Manager/8');
wire(mdl,'Encoder_Fault/1','Feedback_Manager/9'); wire(mdl,'Hall_Fault/1','Feedback_Manager/10'); wire(mdl,'Command_Ramp/2','Feedback_Manager/11');
wire(mdl,'Encoder_Mode/1','Feedback_Manager/12');
wire(mdl,'Command_Ramp/1','Speed_PI_Stable/1'); wire(mdl,'Command_Ramp/1','Speed_PID_VESC/1');
wire(mdl,'Feedback_Manager/2','z_RPM_used/1'); wire(mdl,'z_RPM_used/1','Speed_PI_Stable/2'); wire(mdl,'z_RPM_used/1','Speed_PID_VESC/2');
wire(mdl,'Command_Ramp/2','Speed_PI_Stable/3'); wire(mdl,'Command_Ramp/2','Speed_PID_VESC/3');
wire(mdl,'Speed_PID_VESC/1','Speed_Mode_Select/1'); wire(mdl,'Speed_Mode_VESC/1','Speed_Mode_Select/2'); wire(mdl,'Speed_PI_Stable/1','Speed_Mode_Select/3');
wire(mdl,'z_Vd_req/1','Duty_Calculation/1'); wire(mdl,'z_Vq_req/1','Duty_Calculation/2'); wire(mdl,'Vbus_24V/1','Duty_Calculation/3');
wire(mdl,'Duty_Calculation/1','Duty_LPF/1'); wire(mdl,'Duty_LPF/1','FW_Duty_Map/1');
wire(mdl,'Speed_Mode_Select/1','z_Iq_ref_FW/1');
wire(mdl,'Angle_Manager/3','z_Use_flux/1'); wire(mdl,'z_Use_flux/1','FW_Duty_Map/2');
wire(mdl,'FW_Duty_Map/1','FW_Iq_Backoff/1'); wire(mdl,'z_Iq_meas_FW/1','FW_Iq_Backoff/2');
wire(mdl,'z_Iq_ref_FW/1','FW_Iq_Backoff/3'); wire(mdl,'z_RPM_used/1','FW_Iq_Backoff/4');
wire(mdl,'FW_Iq_Backoff/1','FW_Ramp/1');
wire(mdl,'z_Vd_req/1','FW_Voltage_PI_Stable/1'); wire(mdl,'z_Vq_req/1','FW_Voltage_PI_Stable/2');
wire(mdl,'z_Iq_ref_FW/1','FW_Voltage_PI_Stable/3'); wire(mdl,'z_Use_flux/1','FW_Voltage_PI_Stable/4'); wire(mdl,'Vbus_24V/1','FW_Voltage_PI_Stable/5');
wire(mdl,'FW_Ramp/1','FW_Mode_Select/1'); wire(mdl,'FW_Mode_VESC/1','FW_Mode_Select/2'); wire(mdl,'FW_Voltage_PI_Stable/1','FW_Mode_Select/3');
wire(mdl,'Command_Ramp/3','Id_Align_plus_FW/1'); wire(mdl,'FW_Mode_Select/1','Id_Align_plus_FW/2');
wire(mdl,'Id_Align_plus_FW/1','IdIq_Current_Circle/1'); wire(mdl,'Speed_Mode_Select/1','IdIq_Current_Circle/2');
for k=1:3, wire(mdl,sprintf('PMSM_Discrete/%d',k),sprintf('DQ_Current_PI/%d',k)); end
wire(mdl,'Feedback_Manager/1','DQ_Current_PI/4'); wire(mdl,'IdIq_Current_Circle/1','DQ_Current_PI/5'); wire(mdl,'IdIq_Current_Circle/2','DQ_Current_PI/6'); wire(mdl,'Vbus_24V/1','DQ_Current_PI/7');
wire(mdl,'DQ_Current_PI/4','z_Vd_req/1'); wire(mdl,'DQ_Current_PI/5','z_Vq_req/1');
wire(mdl,'DQ_Current_PI/7','z_Iq_meas_FW/1');
for k=1:3
    wire(mdl,sprintf('HFI_Estimator/%d',k+2),sprintf('HFI_Gate_%d/1',k)); wire(mdl,'Angle_Manager/4',sprintf('HFI_Gate_%d/2',k));
    wire(mdl,sprintf('DQ_Current_PI/%d',k),sprintf('Voltage_Sum_%d/1',k)); wire(mdl,sprintf('HFI_Gate_%d/1',k),sprintf('Voltage_Sum_%d/2',k));
    wire(mdl,sprintf('Voltage_Sum_%d/1',k),sprintf('z_Vphase_%d/1',k)); wire(mdl,sprintf('z_Vphase_%d/1',k),sprintf('PMSM_Discrete/%d',k));
    wire(mdl,sprintf('Voltage_Sum_%d/1',k),sprintf('Voltage_Clarke/%d',k));
end
wire(mdl,'Voltage_Clarke/1','z_Valpha/1'); wire(mdl,'Voltage_Clarke/2','z_Vbeta/1');

% 示波器：与整体版含义一致。
mux(mdl,'Mux_RPM',3,[2520 80 2550 135]); scope(mdl,'Scope_RPM',[2590 80 2670 135]);
wire(mdl,'Command_Ramp/1','Mux_RPM/1'); wire(mdl,'PMSM_Discrete/4','Mux_RPM/2'); wire(mdl,'Feedback_Manager/2','Mux_RPM/3'); wire(mdl,'Mux_RPM/1','Scope_RPM/1');
mux(mdl,'Mux_IdIq',2,[2520 165 2550 210]); scope(mdl,'Scope_IdIq',[2590 165 2670 210]);
wire(mdl,'DQ_Current_PI/6','Mux_IdIq/1'); wire(mdl,'DQ_Current_PI/7','Mux_IdIq/2'); wire(mdl,'Mux_IdIq/1','Scope_IdIq/1');
scope(mdl,'Scope_Te',[2590 250 2670 295]); wire(mdl,'PMSM_Discrete/6','Scope_Te/1');
mux(mdl,'Mux_FW',3,[2520 350 2550 415]); scope(mdl,'Scope_FW',[2590 350 2670 415]);
wire(mdl,'Duty_LPF/1','Mux_FW/1'); wire(mdl,'FW_Duty_Map/1','Mux_FW/2'); wire(mdl,'FW_Mode_Select/1','Mux_FW/3'); wire(mdl,'Mux_FW/1','Scope_FW/1');
mux(mdl,'Mux_Theta',3,[2520 455 2550 510]); scope(mdl,'Scope_Theta',[2590 455 2670 510]);
wire(mdl,'Feedback_Manager/1','Mux_Theta/1'); wire(mdl,'PMSM_Discrete/5','Mux_Theta/2'); wire(mdl,'HFI_Observer_Sync/1','Mux_Theta/3'); wire(mdl,'Mux_Theta/1','Scope_Theta/1');
mux(mdl,'Mux_UVW',3,[2520 550 2550 605]); scope(mdl,'Scope_UVW',[2590 550 2670 605]);
wire(mdl,'PMSM_Discrete/1','Mux_UVW/1'); wire(mdl,'PMSM_Discrete/2','Mux_UVW/2'); wire(mdl,'PMSM_Discrete/3','Mux_UVW/3'); wire(mdl,'Mux_UVW/1','Scope_UVW/1');
mux(mdl,'Mux_Sync',2,[2520 645 2550 690]); scope(mdl,'Scope_Sync',[2590 645 2670 690]);
wire(mdl,'HFI_Observer_Sync/3','Mux_Sync/1'); wire(mdl,'HFI_Observer_Sync/4','Mux_Sync/2'); wire(mdl,'Mux_Sync/1','Scope_Sync/1');
mux(mdl,'Mux_Angle_Error',3,[2520 735 2550 790]); scope(mdl,'Scope_Angle_Error',[2590 735 2670 790]);
wire(mdl,'Angle_Error_Diagnostics/1','Mux_Angle_Error/1'); wire(mdl,'Angle_Error_Diagnostics/2','Mux_Angle_Error/2'); wire(mdl,'Angle_Error_Diagnostics/3','Mux_Angle_Error/3'); wire(mdl,'Mux_Angle_Error/1','Scope_Angle_Error/1');
mux(mdl,'Mux_Feedback_State',3,[2520 825 2550 880]); scope(mdl,'Scope_Feedback_State',[2590 825 2670 880]);
wire(mdl,'Feedback_Manager/3','Mux_Feedback_State/1'); wire(mdl,'Feedback_Manager/4','Mux_Feedback_State/2'); wire(mdl,'Feedback_Manager/5','Mux_Feedback_State/3'); wire(mdl,'Mux_Feedback_State/1','Scope_Feedback_State/1');

try
    a=Simulink.Annotation(mdl,'默认Speed/FW Mode=0且HFI_Sync_Enable=0：已验证稳定路径。Encoder_Enable=1时 Encoder_Mode=0 纯编码器，=1 低速编码器/高速无感'); a.Position=[20 5];
catch
end
save_system(mdl,slx); fprintf('已生成拆分模型：%s\n',slx);
end

function mf(m,n,s,p,c), add_block('simulink/User-Defined Functions/MATLAB Function',[m '/' n],'Position',p,'BackgroundColor',c); cfg=get_param([m '/' n],'MATLABFunctionConfiguration'); cfg.FunctionScript=char(s); try, set_param([m '/' n],'SimulateUsing','Interpreted execution'); catch, end, end
function constant(m,n,v,p), add_block('simulink/Sources/Constant',[m '/' n],'Value',v,'Position',p,'BackgroundColor','yellow'); end
function delay(m,n,p), add_block('simulink/Discrete/Unit Delay',[m '/' n],'SampleTime','1/15000','Position',p); end
function product(m,n,p), add_block('simulink/Math Operations/Product',[m '/' n],'Position',p); end
function mux(m,n,k,p), add_block('simulink/Signal Routing/Mux',[m '/' n],'Inputs',num2str(k),'Position',p); end
function scope(m,n,p), add_block('simulink/Sinks/Scope',[m '/' n],'Position',p,'OpenAtSimulationStart','on'); end
function wire(m,a,b), add_line(m,a,b,'autorouting','on'); end
