function p = load_2804_profile()
%LOAD_2804_PROFILE 从固件2804头文件读取SIL使用的数值参数。
%  只解析形如 (1.23f) 的直接数值宏；表达式宏仍由仿真显式计算。
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
header = fullfile(root, 'App', 'motor_param', 'inc', 'motor_2804_220kv.h');
text = fileread(header);
p.header = header;
p.pole_pairs = value(text, 'MOTOR_2804_220KV_POLE_PAIRS');
p.rs = value(text, 'MOTOR_2804_220KV_PHASE_RESISTANCE_OHM');
p.inductance = value(text, 'MOTOR_2804_220KV_PHASE_INDUCTANCE_H');
p.nameplate_flux = value(text, 'MOTOR_2804_220KV_FLUX_LINKAGE_WB');
p.inertia = value(text, 'MOTOR_2804_220KV_ROTOR_INERTIA_KGM2');
p.current_kp = value(text, 'MOTOR_CURRENT_KP_DEFAULT');
p.current_ki = value(text, 'MOTOR_CURRENT_KI_DEFAULT');
p.speed_kp = value(text, 'MOTOR_SPEED_KP_DEFAULT');
p.speed_ki = value(text, 'MOTOR_SPEED_KI_DEFAULT');
p.speed_iq_limit = value(text, 'MOTOR_SPEED_IQ_LIMIT_DEFAULT');
p.speed_voltage_per_amp = value(text, 'MOTOR_ENCODER_SPEED_VOLT_PER_AMP');
p.speed_voltage_limit = value(text, 'MOTOR_ENCODER_SPEED_VOLT_LIMIT_V');
p.speed_voltage_max_rpm = value(text, 'MOTOR_ENCODER_SPEED_VOLTAGE_MAX_RPM');
p.speed_ff_boost = value(text, 'MOTOR_ENCODER_SPEED_FF_BOOST_V');
p.speed_ff_slope = value(text, 'MOTOR_ENCODER_SPEED_FF_V_PER_RPM');
p.encoder_pll_bw = value(text, 'MOTOR_ENCODER_PLL_BW_RAD_S');
p.position_kp = value(text, 'MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG');
p.position_max_rpm = value(text, 'MOTOR_POSITION_MAX_RPM_DEFAULT');
p.position_vel_kp = value(text, 'MOTOR_POSITION_VEL_KP_DEFAULT');
p.position_vel_ki = value(text, 'MOTOR_POSITION_VEL_KI_DEFAULT');
p.position_voltage_limit = value(text, 'MOTOR_POSITION_VOLTAGE_LIMIT_DEFAULT');
p.position_accel = value(text, 'MOTOR_POSITION_ACCEL_RPM_S_DEFAULT');
end

function x = value(text, name)
pattern = ['(?m)^\s*#define\s+', regexptranslate('escape', name), ...
           '\s+\(\s*([-+0-9.eE]+)f?\s*\)'];
token = regexp(text, pattern, 'tokens', 'once');
if isempty(token), error('load_2804_profile:missingMacro', 'Cannot parse %s', name); end
x = str2double(token{1});
end
