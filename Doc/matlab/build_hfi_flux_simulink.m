function build_hfi_flux_simulink()
%BUILD_HFI_FLUX_SIMULINK  HFI 闭环低速 + Ortega 磁链稳速交接，生成 hfi_flux_pmsm.slx。
this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
logf = fullfile(this_dir, 'hfi_flux_slx_build.log');
fid = fopen(logf, 'w');
if fid < 0
    fid = 1;
end
c = onCleanup(@() close_log(fid, logf));
try
    load_system('simulink');
catch me
    error('build_hfi_flux_simulink:noSL', ...
          '需要 Simulink（%s）。没有的话用脚本看图。', me.message);
end

mdl = 'hfi_flux_pmsm';
slx = fullfile(this_dir, [mdl '.slx']);
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
if exist(slx, 'file')
    delete(slx);
end

new_system(mdl);
set_param(mdl, ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '1/15000', ...
    'StartTime', '0', ...
    'StopTime', '8.0', ...
    'PreLoadFcn', sprintf('addpath(''%s'');', strrep(this_dir, '''', '''''')), ...
    'StartFcn', 'clear sl_hfi_flux sl_hfi sl_flux sl_pmsm_discrete;', ...
    'Description', '闭环：VESC 风格按 ERPM 回差选择 HFI/Ortega 磁链角');

add_mf(mdl, 'HFI_FLUX', ...
    ['function [va,vb,vc,rpm_cmd,th_hfi,th_flux,th_used,rpm_used,id,iq] = fcn(ia,ib,ic,t)' newline ...
     '[va,vb,vc,rpm_cmd,th_hfi,th_flux,th_used,rpm_used,id,iq] = sl_hfi_flux(ia,ib,ic,t);' newline ...
     'end'], [20 50 180 250]);
set_param([mdl '/HFI_FLUX'], 'BackgroundColor', 'cyan');

add_mf(mdl, 'PMSM_Discrete', ...
    ['function [ia,ib,ic,rpm,th,te] = fcn(va,vb,vc)' newline ...
     '[ia,ib,ic,rpm,th,te] = sl_pmsm_discrete(va,vb,vc);' newline ...
     'end'], [470 70 650 230]);

add_block('simulink/Sources/Digital Clock', [mdl '/Clock'], 'Position', [40 280 90 310], 'SampleTime', '1/15000');
add_block('simulink/Discrete/Unit Delay', [mdl '/z_Va'], 'Position', [200 40 235 70], 'SampleTime', '1/15000');
add_block('simulink/Discrete/Unit Delay', [mdl '/z_Vb'], 'Position', [200 90 235 120], 'SampleTime', '1/15000');
add_block('simulink/Discrete/Unit Delay', [mdl '/z_Vc'], 'Position', [200 140 235 170], 'SampleTime', '1/15000');

add_mux(mdl, 'Mux_Iabc', 3, [880 40 910 90]);
add_mux(mdl, 'Mux_RPM', 3, [880 150 910 200]);
add_mux(mdl, 'Mux_Theta', 3, [880 260 910 310]);
add_mux(mdl, 'Mux_IdIq', 2, [880 430 910 470]);

add_scope(mdl, 'Scope_UVW', [960 35 1040 95]);
add_scope(mdl, 'Scope_RPM', [960 145 1040 205]);
add_scope(mdl, 'Scope_Theta', [960 255 1040 315]);
add_scope(mdl, 'Scope_Te', [960 360 1040 410]);
add_scope(mdl, 'Scope_IdIq', [960 430 1040 480]);

add_line(mdl, 'Clock/1', 'HFI_FLUX/4', 'autorouting', 'on');
add_line(mdl, 'HFI_FLUX/1', 'z_Va/1', 'autorouting', 'on');
add_line(mdl, 'HFI_FLUX/2', 'z_Vb/1', 'autorouting', 'on');
add_line(mdl, 'HFI_FLUX/3', 'z_Vc/1', 'autorouting', 'on');
add_line(mdl, 'z_Va/1', 'PMSM_Discrete/1', 'autorouting', 'on');
add_line(mdl, 'z_Vb/1', 'PMSM_Discrete/2', 'autorouting', 'on');
add_line(mdl, 'z_Vc/1', 'PMSM_Discrete/3', 'autorouting', 'on');
add_line(mdl, 'PMSM_Discrete/1', 'HFI_FLUX/1', 'autorouting', 'on');
add_line(mdl, 'PMSM_Discrete/2', 'HFI_FLUX/2', 'autorouting', 'on');
add_line(mdl, 'PMSM_Discrete/3', 'HFI_FLUX/3', 'autorouting', 'on');
add_line(mdl, 'PMSM_Discrete/1', 'Mux_Iabc/1', 'autorouting', 'on');
add_line(mdl, 'PMSM_Discrete/2', 'Mux_Iabc/2', 'autorouting', 'on');
add_line(mdl, 'PMSM_Discrete/3', 'Mux_Iabc/3', 'autorouting', 'on');
add_line(mdl, 'HFI_FLUX/4', 'Mux_RPM/1', 'autorouting', 'on');
add_line(mdl, 'PMSM_Discrete/4', 'Mux_RPM/2', 'autorouting', 'on');
add_line(mdl, 'HFI_FLUX/8', 'Mux_RPM/3', 'autorouting', 'on');
add_line(mdl, 'HFI_FLUX/7', 'Mux_Theta/1', 'autorouting', 'on');
add_line(mdl, 'PMSM_Discrete/5', 'Mux_Theta/2', 'autorouting', 'on');
add_line(mdl, 'HFI_FLUX/6', 'Mux_Theta/3', 'autorouting', 'on');
add_line(mdl, 'Mux_Iabc/1', 'Scope_UVW/1', 'autorouting', 'on');
add_line(mdl, 'Mux_RPM/1', 'Scope_RPM/1', 'autorouting', 'on');
add_line(mdl, 'Mux_Theta/1', 'Scope_Theta/1', 'autorouting', 'on');
add_line(mdl, 'PMSM_Discrete/6', 'Scope_Te/1', 'autorouting', 'on');
add_line(mdl, 'HFI_FLUX/9', 'Mux_IdIq/1', 'autorouting', 'on');
add_line(mdl, 'HFI_FLUX/10', 'Mux_IdIq/2', 'autorouting', 'on');
add_line(mdl, 'Mux_IdIq/1', 'Scope_IdIq/1', 'autorouting', 'on');

try
    a1 = Simulink.Annotation(mdl, '离散 PMSM：Ld=0.5mH Lq=0.8mH；统一 Clarke/Park 符号');
    a1.Position = [430 8];
    a2 = Simulink.Annotation(mdl, '黄=给定/Park角  蓝=转子  橙=磁链  RPM橙=闭环反馈；IdIq 黄=Id 蓝=Iq');
    a2.Position = [820 8];
    a3 = Simulink.Annotation(mdl, '青=HFI；3000 ERPM ±5% 回差切换磁链；80% 调制度后启动弱磁');
    a3.Position = [20 8];
catch
end

save_system(mdl, slx);
log_msg(fid, '已生成 %s', slx);
fprintf('已生成 %s\n', slx);
fprintf('桌面打开：open_hfi_flux_simulink   然后点运行，示波器会自己弹出\n');
end

function close_log(fid, logf)
if fid > 2
    fclose(fid);
end
fprintf('搭建日志：%s\n', logf);
end

function log_msg(fid, fmt, varargin)
msg = sprintf(fmt, varargin{:});
fprintf('%s\n', msg);
if fid > 2
    fprintf(fid, '%s\n', msg);
end
end

function add_mf(mdl, name, script, pos)
blk = [mdl '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
try
    cfg = get_param(blk, 'MATLABFunctionConfiguration');
    cfg.FunctionScript = script;
catch
    error('build_hfi_flux_simulink:mf', '无法写入 MATLAB Function，请确认已安装 Simulink。');
end
try
    set_param(blk, 'SimulateUsing', 'Interpreted execution');
catch
end
end

function add_mux(mdl, name, n, pos)
add_block('simulink/Signal Routing/Mux', [mdl '/' name], 'Inputs', num2str(n), 'Position', pos);
end

function add_scope(mdl, name, pos)
blk = [mdl '/' name];
add_block('simulink/Sinks/Scope', blk, 'Position', pos);
set_param(blk, 'OpenAtSimulationStart', 'on');
end

function add_ee_pmsm(mdl, fid)
c = foc.constants();
libs = {'ee_lib', 'fl_lib', 'nesl_utility'};
for i = 1:numel(libs)
    try
        load_system(libs{i});
        log_msg(fid, '已加载库 %s', libs{i});
    catch me
        log_msg(fid, '加载 %s 失败：%s', libs{i}, me.message);
    end
end

pmsm_src = first_block(mdl, fid, { ...
    'ee_lib/Electromechanical/Permanent Magnet/PMSM'});
if isempty(pmsm_src)
    error('build_hfi_flux_simulink:noPMSM', ...
          ['没有找到 MATLAB 自带 PMSM。R2026a 已去掉 Specialized Power Systems，\n' ...
           '需要 Simscape Electrical。库浏览器：Simscape → Electrical → Electromechanical → Permanent Magnet → PMSM。']);
end
log_msg(fid, '使用电机库：%s', pmsm_src);

cvs_src = first_block(mdl, fid, {'fl_lib/Electrical/Electrical Sources/Controlled Voltage Source'});
isens_src = first_block(mdl, fid, {'fl_lib/Electrical/Electrical Sensors/Current Sensor'});
eref_src = first_block(mdl, fid, {'fl_lib/Electrical/Electrical Elements/Electrical Reference'});
solver_src = first_block(mdl, fid, {'nesl_utility/Solver Configuration'});
s2ps_src = first_block(mdl, fid, {'nesl_utility/Simulink-PS Converter'});
ps2s_src = first_block(mdl, fid, {'nesl_utility/PS-Simulink Converter'});
mref_src = first_block(mdl, fid, {'fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference'});
wsens_src = first_block(mdl, fid, { ...
    'fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor', ...
    'fl_lib/Mechanical/Rotational Sensors/Ideal Rotational Motion Sensor'});
if any(cellfun(@isempty, {cvs_src, isens_src, eref_src, solver_src, s2ps_src, ps2s_src, mref_src, wsens_src}))
    error('build_hfi_flux_simulink:noSimscape', '缺 Simscape 源/传感器/参考地。请确认已装 Simscape + Simscape Electrical。');
end

add_block(pmsm_src, [mdl '/PMSM'], 'Position', [470 40 630 210]);
set_if(mdl, 'PMSM', {'port_option'}, 'ee.enum.threePhasePort.expanded');
set_if(mdl, 'PMSM', {'Rs'}, num2str(c.R_LINE));
set_if(mdl, 'PMSM', {'Ld'}, num2str(c.HFI_LD));
set_if(mdl, 'PMSM', {'Lq'}, num2str(c.HFI_LQ));
set_if(mdl, 'PMSM', {'L0'}, num2str(0.5 * (c.HFI_LD + c.HFI_LQ)));
set_if(mdl, 'PMSM', {'pm_flux_linkage'}, num2str(c.FLUX));
set_if(mdl, 'PMSM', {'nPolePairs'}, num2str(c.POLE_PAIRS));
set_if(mdl, 'PMSM', {'J'}, num2str(c.J));
set_if(mdl, 'PMSM', {'lam'}, num2str(c.B_VISC));
try
    log_msg(fid, 'PMSM stator_param=%s Ld=%s Lq=%s', ...
            get_param([mdl '/PMSM'], 'stator_param'), ...
            get_param([mdl '/PMSM'], 'Ld'), ...
            get_param([mdl '/PMSM'], 'Lq'));
catch me
    log_msg(fid, '读 PMSM Ld/Lq 失败：%s', me.message);
end
dump_block(fid, [mdl '/PMSM']);

add_block(cvs_src, [mdl '/CVS_A'], 'Position', [310 30 360 70]);
add_block(cvs_src, [mdl '/CVS_B'], 'Position', [310 95 360 135]);
add_block(cvs_src, [mdl '/CVS_C'], 'Position', [310 160 360 200]);
add_block(isens_src, [mdl '/ISens_A'], 'Position', [390 30 430 70]);
add_block(isens_src, [mdl '/ISens_B'], 'Position', [390 95 430 135]);
add_block(isens_src, [mdl '/ISens_C'], 'Position', [390 160 430 200]);
add_block(eref_src, [mdl '/ElecRef'], 'Position', [330 230 360 250]);
add_block(solver_src, [mdl '/SolverCfg'], 'Position', [30 500 90 540]);
add_block(mref_src, [mdl '/MechRef'], 'Position', [520 250 560 270]);
add_block(wsens_src, [mdl '/WSens'], 'Position', [500 290 560 340]);

add_block(s2ps_src, [mdl '/S2PS_A'], 'Position', [250 40 290 70]);
add_block(s2ps_src, [mdl '/S2PS_B'], 'Position', [250 105 290 135]);
add_block(s2ps_src, [mdl '/S2PS_C'], 'Position', [250 170 290 200]);
add_block(ps2s_src, [mdl '/PS2S_IA'], 'Position', [390 360 440 380]);
add_block(ps2s_src, [mdl '/PS2S_IB'], 'Position', [390 390 440 410]);
add_block(ps2s_src, [mdl '/PS2S_IC'], 'Position', [390 420 440 440]);
add_block(ps2s_src, [mdl '/PS2S_W'], 'Position', [580 300 630 320]);
add_block(ps2s_src, [mdl '/PS2S_Th'], 'Position', [580 330 630 350]);
set_if(mdl, 'S2PS_A', {'Unit'}, 'V');
set_if(mdl, 'S2PS_B', {'Unit'}, 'V');
set_if(mdl, 'S2PS_C', {'Unit'}, 'V');
set_if(mdl, 'PS2S_IA', {'Unit'}, 'A');
set_if(mdl, 'PS2S_IB', {'Unit'}, 'A');
set_if(mdl, 'PS2S_IC', {'Unit'}, 'A');
set_if(mdl, 'PS2S_W', {'Unit'}, 'rad/s');
set_if(mdl, 'PS2S_Th', {'Unit'}, 'rad');

try
    set_param([mdl '/SolverCfg'], 'UseLocalSolver', 'on', ...
        'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', ...
        'LocalSolverSampleTime', '1/15000');
catch me
    log_msg(fid, 'SolverCfg 局部求解器设置失败：%s', me.message);
end

dump_block(fid, [mdl '/CVS_A']);
dump_block(fid, [mdl '/ISens_A']);
dump_block(fid, [mdl '/S2PS_A']);
dump_block(fid, [mdl '/PS2S_IA']);
dump_block(fid, [mdl '/WSens']);
dump_block(fid, [mdl '/ElecRef']);
dump_block(fid, [mdl '/MechRef']);
dump_block(fid, [mdl '/SolverCfg']);
wire_machine(mdl, fid);
end

function src = first_block(mdl, fid, cands)
src = '';
probe = [mdl '/_probe_tmp'];
for i = 1:numel(cands)
    try
        load_system(strtok(cands{i}, '/'));
        if ~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', '_probe_tmp'))
            delete_block(probe);
        end
        add_block(cands{i}, probe);
        delete_block(probe);
        src = cands{i};
        log_msg(fid, '找到模块 %s', src);
        return
    catch me
        if ~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', '_probe_tmp'))
            delete_block(probe);
        end
        log_msg(fid, '候选失败 %s：%s', cands{i}, me.message);
    end
end
end

function dump_block(fid, blk)
log_msg(fid, '---- %s ----', blk);
try
    dp = get_param(blk, 'DialogParameters');
    log_msg(fid, 'DialogParameters: %s', strjoin(fieldnames(dp).', ', '));
catch me
    log_msg(fid, 'DialogParameters 失败：%s', me.message);
end
try
    ph = get_param(blk, 'PortHandles');
    fn = fieldnames(ph);
    for i = 1:numel(fn)
        v = ph.(fn{i});
        log_msg(fid, 'PortHandles.%s n=%d', fn{i}, numel(v));
    end
catch me
    log_msg(fid, 'PortHandles 失败：%s', me.message);
end
end

function wire_machine(mdl, fid)
sc(fid, mdl, 'CVS_A', 'p', 'ISens_A', 'p');
sc(fid, mdl, 'CVS_B', 'p', 'ISens_B', 'p');
sc(fid, mdl, 'CVS_C', 'p', 'ISens_C', 'p');
sc(fid, mdl, 'ISens_A', 'n', 'PMSM', 'a');
sc(fid, mdl, 'ISens_B', 'n', 'PMSM', 'b');
sc(fid, mdl, 'ISens_C', 'n', 'PMSM', 'c');
sc(fid, mdl, 'CVS_A', 'n', 'ElecRef', 'V');
sc(fid, mdl, 'CVS_B', 'n', 'ElecRef', 'V');
sc(fid, mdl, 'CVS_C', 'n', 'ElecRef', 'V');
sc(fid, mdl, 'PMSM', 'C', 'MechRef', 'W');
sc(fid, mdl, 'PMSM', 'R', 'WSens', 'R');
sc(fid, mdl, 'WSens', 'C', 'MechRef', 'W');
sc_try(fid, mdl, 'S2PS_A', 'RConn', 'CVS_A', 'vT');
sc_try(fid, mdl, 'S2PS_B', 'RConn', 'CVS_B', 'vT');
sc_try(fid, mdl, 'S2PS_C', 'RConn', 'CVS_C', 'vT');
ps_unused(fid, mdl, 'S2PS_A', 'CVS_A');
ps_unused(fid, mdl, 'S2PS_B', 'CVS_B');
ps_unused(fid, mdl, 'S2PS_C', 'CVS_C');
sc_try(fid, mdl, 'ISens_A', 'I', 'PS2S_IA', 'LConn');
sc_try(fid, mdl, 'ISens_B', 'I', 'PS2S_IB', 'LConn');
sc_try(fid, mdl, 'ISens_C', 'I', 'PS2S_IC', 'LConn');
ps_unused(fid, mdl, 'ISens_A', 'PS2S_IA');
ps_unused(fid, mdl, 'ISens_B', 'PS2S_IB');
ps_unused(fid, mdl, 'ISens_C', 'PS2S_IC');
sc_try(fid, mdl, 'WSens', 'W', 'PS2S_W', 'LConn');
sc_try(fid, mdl, 'WSens', 'A', 'PS2S_Th', 'LConn');
ps_unused(fid, mdl, 'WSens', 'PS2S_W');
ps_unused(fid, mdl, 'WSens', 'PS2S_Th');
sc_try(fid, mdl, 'SolverCfg', 'RConn', 'ElecRef', 'V');
if ~already_linked(mdl, 'SolverCfg', 'ElecRef')
    try
        add_line(mdl, 'SolverCfg/RConn1', 'ElecRef/LConn1', 'autorouting', 'on');
        log_msg(fid, 'al OK SolverCfg/RConn1 -> ElecRef/LConn1');
    catch me
        log_msg(fid, 'SolverCfg 连地失败：%s', me.message);
        rethrow(me);
    end
end
end

function sc(fid, mdl, a, pa, b, pb)
try
    simscape.addConnection([mdl '/' a], pa, [mdl '/' b], pb);
    log_msg(fid, 'sc OK %s.%s -> %s.%s', a, pa, b, pb);
catch me
    log_msg(fid, 'sc 失败 %s.%s -> %s.%s：%s', a, pa, b, pb, me.message);
    rethrow(me);
end
end

function ok = sc_try(fid, mdl, a, pa, b, pb)
ok = false;
try
    simscape.addConnection([mdl '/' a], pa, [mdl '/' b], pb);
    log_msg(fid, 'sc OK %s.%s -> %s.%s', a, pa, b, pb);
    ok = true;
catch me
    log_msg(fid, 'sc 跳过 %s.%s -> %s.%s：%s', a, pa, b, pb, me.message);
end
end

function ps_unused(fid, mdl, src, dst)
if already_linked(mdl, src, dst)
    log_msg(fid, '已有连线 %s -> %s', src, dst);
    return
end
hs = get_param([mdl '/' src], 'PortHandles');
hd = get_param([mdl '/' dst], 'PortHandles');
src_ports = [hs.RConn(:); hs.LConn(:)];
dst_ports = [hd.LConn(:); hd.RConn(:)];
for i = 1:numel(src_ports)
    if port_busy(src_ports(i))
        continue
    end
    for j = 1:numel(dst_ports)
        if port_busy(dst_ports(j))
            continue
        end
        try
            add_line(mdl, src_ports(i), dst_ports(j), 'autorouting', 'on');
            log_msg(fid, 'ps OK %s(%d) -> %s(%d)', src, i, dst, j);
            return
        catch me
            log_msg(fid, 'ps 跳过 %s(%d) -> %s(%d)：%s', src, i, dst, j, me.message);
        end
    end
end
error('build_hfi_flux_simulink:ps', '无法连接物理信号 %s -> %s', src, dst);
end

function busy = port_busy(h)
busy = false;
try
    lh = get_param(h, 'Line');
    busy = isnumeric(lh) && any(lh(:) ~= -1);
catch
end
end

function tf = already_linked(mdl, src, dst)
tf = false;
lh = find_system(mdl, 'FindAll', 'on', 'Type', 'line');
srcb = get_param([mdl '/' src], 'Handle');
dstb = get_param([mdl '/' dst], 'Handle');
for i = 1:numel(lh)
    try
        s = get_param(lh(i), 'SrcBlockHandle');
        d = get_param(lh(i), 'DstBlockHandle');
        if (s == srcb && d == dstb) || (s == dstb && d == srcb)
            tf = true;
            return
        end
    catch
    end
end
end

function set_if(mdl, name, fields, val)
blk = [mdl '/' name];
try
    dp = get_param(blk, 'DialogParameters');
catch
    return
end
for i = 1:numel(fields)
    if isfield(dp, fields{i})
        try
            set_param(blk, fields{i}, val);
            return
        catch
        end
    end
end
end
