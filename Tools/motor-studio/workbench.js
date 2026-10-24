import catalog from './catalog.js';
import {validateCommand, numericDraft, SerialDecoder, plotChannels} from './protocol.mjs';
import {commandHelp, actionHelp, describeCommand} from './command-help.mjs';
import {StudioDecoder} from './studio-link.mjs';

const q = selector => document.querySelector(selector);
const el = (tag, attrs={}, text='') => {
  const node=document.createElement(tag);
  for (const [key,value] of Object.entries(attrs)) {
    if (key==='class') node.className=value;
    else node.setAttribute(key,value);
  }
  node.textContent=text; return node;
};
const labels={
  ol_max_rpm:'最大目标转速',ol_v_boost:'V/F 启动补偿',ol_kv_f:'V/F 斜率',ol_ramp_rpm_s:'转速斜坡',ol_vq_limit:'电压限幅',ol_foc_freq:'ADC-DMA 基准频率',ol_pole_pairs:'极对数',ol_min_rpm:'默认起步转速',ol_current_kp:'电流环 Kp',ol_current_ki:'电流环 Ki',ol_start_phase_boost:'旧启动相位偏置',ol_align_enable:'启动预对齐使能',ol_align_current:'预对齐 Id',ol_align_time:'预对齐保持时间',ol_align_angle:'预对齐电角度',hfi_start_enable:'HFI 启动使能',hfi_start_voltage:'HFI 注入幅值',hfi_start_cycles:'HFI 每轴周期',hfi_start_min_confidence:'HFI 最小置信度',speed_enable:'速度环使能',speed_allow_braking:'速度环制动',speed_kp:'速度环 Kp',speed_ki:'速度环 Ki',speed_iq_limit:'速度环电流限幅',speed_loop_hz:'速度环频率',sensorless_enable:'观测器使能',sensorless_takeover:'观测器角度接管',sensorless_motor_r:'电机模型电阻',sensorless_motor_l:'电机模型电感',sensorless_flux_linkage:'电机模型磁链',sensorless_observer_gain:'观测器增益',sensorless_pll_kp:'PLL Kp',sensorless_pll_ki:'PLL Ki',sensorless_transition_rpm:'观测器接管转速',sensorless_fallback_rpm:'观测器回退转速',sensorless_openloop_max_rpm:'未锁定开环护栏',sensorless_lock_time:'锁定判定时间',sensorless_loss_time:'失锁判定时间',sensorless_blend_time:'角度混合时间',sensorless_phase_tolerance:'接管相位容差',sensorless_phase_offset:'观测器相位偏置',sensorless_speed_filter:'速度滤波系数',hall_extra_samples:'霍尔额外采样次数',current_offset:'三相电流偏移',voltage_offset:'三相电压偏移',undriven_offset:'关桥相电压偏移',hall_table:'霍尔标定表',encoder_offset_rad:'编码器电角零点',encoder_inverted:'编码器方向反转',encoder_calibrated:'编码器标定有效',fw_current_max:'弱磁电流上限',fw_duty_start:'弱磁进入利用率',fw_ramp_a_s:'弱磁变化斜坡',ol_iq_set:'目标 Iq',ol_id_set:'目标 Id',ol_target_rpm:'目标转速',ol_mode:'V/F 与 I/F 模式',ol_enable:'运行使能',feedback:'反馈源',sldt:'观测电压死区补偿比例',position_kp:'位置环 Kp',position_max:'位置最大速度',joint_kp:'关节 Kp',joint_kd:'关节 Kd',joint_ki:'关节 Ki',joint_lim:'关节电压上限',
};
const units={ol_max_rpm:'rpm',ol_v_boost:'V',ol_kv_f:'V/Hz',ol_ramp_rpm_s:'rpm/s',ol_vq_limit:'V',ol_foc_freq:'Hz',ol_pole_pairs:'对',ol_min_rpm:'rpm',ol_current_kp:'V/A',ol_current_ki:'V/(A·s)',ol_start_phase_boost:'rad',ol_align_current:'A',ol_align_time:'s',ol_align_angle:'rad',hfi_start_voltage:'V',speed_kp:'A/rpm',speed_ki:'A/(rpm·s)',speed_iq_limit:'A',speed_loop_hz:'Hz',sensorless_motor_r:'Ω',sensorless_motor_l:'H',sensorless_flux_linkage:'Wb',sensorless_transition_rpm:'rpm',sensorless_fallback_rpm:'rpm',sensorless_openloop_max_rpm:'rpm',sensorless_lock_time:'s',sensorless_loss_time:'s',sensorless_blend_time:'s',sensorless_phase_tolerance:'rad',sensorless_phase_offset:'rad',encoder_offset_rad:'rad',fw_current_max:'A',fw_ramp_a_s:'A/s',ol_iq_set:'A',ol_id_set:'A',ol_target_rpm:'rpm',position_kp:'rpm/deg',position_max:'rpm',joint_kp:'V/deg',joint_kd:'V/rpm',joint_ki:'V/(deg·s)',joint_lim:'V'};
const single={ol_max_rpm:'olmax',ol_v_boost:'olboost',ol_kv_f:'olkvf',ol_ramp_rpm_s:'olramp',ol_vq_limit:'olvlimit',ol_min_rpm:'olmin',ol_current_kp:'olkp',ol_current_ki:'olki',ol_start_phase_boost:'olphase',speed_enable:'spenable',speed_allow_braking:'spbrake',speed_kp:'spkp',speed_ki:'spki',speed_iq_limit:'spiq',sensorless_enable:'slenable',sensorless_takeover:'sltakeover',sensorless_phase_offset:'slphase',ol_iq_set:'oliq',ol_id_set:'olid',ol_target_rpm:'olspeed',ol_mode:'olmode',feedback:'olfb',sldt:'sldt',position_kp:'pos kp',position_max:'pos max',joint_kp:'joint kp',joint_kd:'joint kd',joint_ki:'joint ki',joint_lim:'joint lim'};
const bundles=[
  ['olalign',['ol_align_enable','ol_align_current','ol_align_time','ol_align_angle']],
  ['hfistart',['hfi_start_enable','hfi_start_voltage','hfi_start_cycles','hfi_start_min_confidence']],
  ['slrpm',['sensorless_transition_rpm','sensorless_fallback_rpm']],
  ['slguard',['sensorless_openloop_max_rpm','sensorless_phase_tolerance','sensorless_blend_time']],
  ['slmotor',['sensorless_motor_r','sensorless_motor_l','sensorless_flux_linkage']],
  ['slobs',['sensorless_observer_gain','sensorless_pll_kp','sensorless_pll_ki']],
  ['fw',['fw_current_max','fw_duty_start','fw_ramp_a_s']],
];
const examples={
  detect:['detect','detect offset ram','detect offset','detect hall','detect phase','detect rl','detect motor','detect fluxscan','detect show','detect abort'],
  encoder:['encoder show','encoder reset','encoder detect ram','encoder watch 2','encoder compare 0','encoder compare 1','encoder cog status','encoder cog learn 1','encoder cog on','encoder cog off','encoder cog stop','encoder cog clear'],
  bldc:['bldc status','bldc stop','bldc exit','bldc start','bldc open','bldc sample up','bldc sample down','bldc tune'],
  pos:['pos status','pos zero','pos hold','pos on','pos off','pos set','pos move','pos test','pos kp','pos max'],
  joint:['joint status','joint zero','joint on','joint off','joint set','joint test','joint slow','joint hybridtest','joint iqtest','joint timeouttest','joint kp','joint kd','joint ki','joint lim'],
  speed:['speed','speed test','speed ff detect'],plot:['plot help','plot create current','plot create speed','plot create pos','plot create sine','plot stop'],
  duty:['duty create','duty set','duty start','duty stop','duty destory'],task:['task','task watch 1000'],olfault:['olfault','olfault clear'],
};
const helpOverrides={olphase:'olphase <radians>（弧度；command.c 的 deg 帮助文本与实现不同，以实现为准）',slphase:'slphase <rad>（弧度）',slmotor:'slmotor <R_ohm> <L_mH> <flux_mWb>（表格 H、Wb 在生成命令时转换为 mH、mWb）',sldt:'sldt [fraction 0..0.02]（停机，RAM 临时值）',fw:'fw [max_A [duty_start [ramp_A/s]]]（运行命令，不写入持久配置）'};
const known=catalog.commands.map(c=>c.name);
let active=catalog.commands.find(c=>c.name==='olstatus'), serialPort=null, reader=null, readTask=null, serialConnected=false, closing=false, sending=false;
let transport='demo', rxBytes=0, txBytes=0, lastFrameTime=0, plotMode=null, currentTab='config', textTail='', logDirty=false;
let profile=catalog.profiles[0], draft={}, terminal=[], points=[];
let structuredReady=false;
const deviceValues=new Map(), packetListeners=new Set(), stateListeners=new Set(), textListeners=new Set();
let batchBusy=false;
const valueText=value=>Array.isArray(value)?value.map(v=>typeof v==='number'?Number(v.toPrecision(7)):'—').join(', '):typeof value==='number'?String(Number(value.toPrecision(8))):String(value);
function receiveValue(name,value){deviceValues.set(name,{value,at:Date.now()});for(const cell of document.querySelectorAll('.device-readback'))if(cell.dataset.field===name){cell.textContent=valueText(value)+' · 已回读';cell.classList.add('status-received');}}
const macroMap=()=>Object.fromEntries(profile.macros.map(x=>[x.name,x.value]));
function resolve(value,depth=0) {
  if(value==null || depth>15)return null;
  let v=String(value).trim(); if(v==='true')return 1;if(v==='false')return 0;
  if(v==='MOTOR_OL_MODE_IF')return 1;if(v==='MOTOR_OL_MODE_VF')return 0;
  const macros=macroMap();
  if(macros[v]!=null)return resolve(macros[v],depth+1);
  v=v.replace(/\bM_PI\b/g,String(Math.PI)).replace(/\bMOTOR_\w+\b/g,name=>String(resolve(macros[name],depth+1)??'unknown'));
  v=v.replace(/(\d)[fFuU]\b/g,'$1');
  if(!/^[\d\s.eE+*/()\-]+$/.test(v))return null;
  try { const n=Function('"use strict";return ('+v+')')();return Number.isFinite(n)?Number(n.toPrecision(10)):null; }catch{return null;}
}
function resetDraft(){draft={};for(const [key,value]of Object.entries(catalog.defaults))draft[key]=resolve(value);}
resetDraft();

const dashboard=q('main'); dashboard.classList.add('studio-view');dashboard.dataset.view='dashboard';
const hardwareNotice=el('section',{class:'panel transport-panel'});hardwareNotice.hidden=true;
hardwareNotice.append(el('h2',{},'真实设备控制'),el('p',{class:'studio-note'},'上方持续显示设备仪表与波形。使用下方命令和参数操作设备；连接不等于电机已运行。'));
const hardwareLink=el('button',{class:'primary'},'打开设备工作台');hardwareLink.onclick=()=>navigate('commands');hardwareNotice.append(hardwareLink);dashboard.prepend(hardwareNotice);
const workspace=q('.workspace');const commandView=el('main',{'class':'studio-view',id:'command-view'});commandView.hidden=true;
commandView.innerHTML=`<div class="studio-header"><div><h1>命令工作台<span style="color:var(--green)">.</span></h1><div class="studio-note">按当前固件源码整理，保留全部命令入口与子功能。选好参数，预览后再发送。</div></div><input class="studio-input studio-search" id="command-search" placeholder="搜索功能、命令或子命令" aria-label="搜索命令"></div>
<div class="transport-panel panel"><div class="studio-toolbar"><span class="local-chip" id="transport-chip">演示模式 · 不发送硬件指令</span><label>连接方式 <select id="transport-mode" class="studio-select"><option value="demo">演示 / 命令预览</option><option value="serial">真实串口 · Web Serial</option></select></label><label>波特率 <select id="serial-baud" class="studio-select"><option>2000000</option><option>921600</option><option>115200</option></select></label><button id="serial-connect">选择串口并连接</button><button id="serial-disconnect" disabled>断开串口</button></div><p class="transport-status" id="transport-status">默认不会连接设备。真实串口需要浏览器支持 Web Serial；连接成功也不会自动启动电机。</p></div>
<div class="summary-strip"><span><strong>${catalog.commands.length}</strong>命令入口</span><span><strong>${catalog.config.length}</strong>配置字段</span><span><strong>4</strong>电机源码档案</span><span>UART4 · 2,000,000 · 8N1</span><span>模块可由固件编译开关裁剪</span></div>
<div class="quick-actions" id="quick-actions"></div><div class="catalog-layout"><section class="panel command-menu" id="command-menu"></section><section class="panel command-detail" id="command-detail"></section></div>
<section class="panel plot-card"><div class="studio-toolbar"><h2>设备波形</h2><span class="dim" id="stream-status">等待串口数据，未使用模拟信号</span><button id="stream-export" style="margin-left:auto">导出设备波形</button></div><div class="real-metrics" id="stream-values">发送 plot create current / speed / pos / sine 后显示对应数据</div><canvas id="real-scope" aria-label="设备 JustFloat 波形"></canvas></section>
<section class="panel terminal-panel"><div class="panel-heading"><h2>串口终端 / 命令记录</h2><small>发送 ≠ 固件执行成功，请核对 RX 回复</small></div><div class="terminal-toolbar"><button class="terminal-key" id="terminal-clear">清空显示</button><button class="terminal-key" id="terminal-export">导出日志</button><button class="terminal-key" id="watch-break">发送空格 · 退出 watch</button><span id="traffic" class="console-counter">RX 0 B / TX 0 B</span></div><pre class="terminal-output" id="terminal-output" aria-label="串口输出" tabindex="0"></pre><form class="terminal-line" id="terminal-form"><input class="studio-input" id="terminal-input" placeholder="输入命令，例如 olstatus；↑↓ 浏览发送历史" aria-label="手动命令" autocomplete="off"><button type="submit" class="primary">预览并发送</button></form></section>`;
const paramsView=el('main',{'class':'studio-view',id:'parameter-view'});paramsView.hidden=true;
paramsView.innerHTML=`<div class="studio-header"><div><h1>电机参数<span style="color:var(--green)">.</span></h1><div class="studio-note">源码默认 / 本地草稿与设备值分开。当前没有通用参数读写协议，未读回的设备值一律显示“未知”。</div></div><button id="draft-export">导出参数草稿 JSON</button></div><div class="studio-toolbar"><label>参考电机 <select id="profile-select" class="studio-select"></select></label><input id="param-search" class="studio-input studio-search" placeholder="搜索字段或中文名称" aria-label="搜索参数"></div><div class="workbench-warning" id="profile-note"></div><div class="parameter-tabs" id="parameter-tabs"></div><div class="panel"><div class="draft-toolbar"><span class="studio-note" id="parameter-count"></span><div class="studio-toolbar"><button id="query-parameters">预览参数查询命令</button><button id="flash-save">预览 Flash 保存</button></div></div><div class="param-table-wrap" id="parameter-table"></div></div><div class="profile-grid" id="profile-grid" hidden></div>`;
const scopeView=el('section',{class:'persistent-monitor',id:'live-monitor','aria-label':'仪表与实时示波器'});
scopeView.innerHTML='<div class="monitor-heading"><h2>仪表与实时示波器</h2><span class="studio-note" id="scope-source">演示信号</span><button id="scope-connection">串口与采集设置</button></div>';
const connectionDrawer=el('div',{class:'monitor-connection',id:'monitor-connection'});connectionDrawer.hidden=true;connectionDrawer.append(commandView.querySelector('.transport-panel'));scopeView.append(connectionDrawer,q('.cards'));
const demoScopeHost=el('section',{id:'demo-scope-host'});demoScopeHost.append(q('#wave'));
const deviceScopeHost=el('section',{id:'device-scope-host'});deviceScopeHost.hidden=true;
const streamActions=el('div',{class:'studio-toolbar stream-actions'});
for(const [label,args]of [['三相电流','create current'],['转速','create speed'],['位置','create pos'],['正余弦 / d-q','create sine'],['停止采集','stop']]){const b=el('button',{},label);b.onclick=()=>sendLine('plot '+args);streamActions.append(b);}
deviceScopeHost.append(streamActions,commandView.querySelector('.plot-card'));scopeView.append(demoScopeHost,deviceScopeHost);
const operationZone=el('div',{class:'operation-zone',id:'operation-zone'});operationZone.append(dashboard,commandView,paramsView);workspace.append(scopeView,operationZone);
q('#scope-connection').onclick=()=>{connectionDrawer.hidden=!connectionDrawer.hidden;q('#scope-connection').setAttribute('aria-expanded',String(!connectionDrawer.hidden));};
q('[data-nav="wave"]').hidden=true;q('[data-nav="overview"]').innerHTML='<i>▷</i>运行控制';
const extraNav=el('button',{'class':'nav studio-nav','data-view-button':'commands'},'⌘ 命令工作台');q('.sidebar-bottom').before(extraNav);
const detectView=el('main',{class:'studio-view',id:'detection-view'});detectView.hidden=true;operationZone.append(detectView);
const detectNav=el('button',{class:'nav','data-view-button':'detection'},'◇ 电机参数检测');extraNav.before(detectNav);detectNav.onclick=()=>navigate('detection');
function navigate(view){if(view==='scope')return;document.body.classList.toggle('detection-active',view==='detection');for(const n of document.querySelectorAll('.studio-view'))n.hidden=n!==({dashboard,commands:commandView,parameters:paramsView,detection:detectView}[view]);q('#breadcrumb').textContent={dashboard:'运行控制',commands:'命令工作台',parameters:'电机参数',detection:'电机参数检测'}[view];for(const b of document.querySelectorAll('.nav')){b.classList.remove('active');b.removeAttribute('aria-current');}const selected={dashboard:q('[data-nav="overview"]'),parameters:q('[data-nav="params"]'),commands:extraNav,detection:detectNav}[view];selected.classList.add('active');selected.setAttribute('aria-current','page');operationZone.scrollTop=0;}
extraNav.onclick=()=>navigate('commands');q('[data-nav="params"]').onclick=()=>navigate('parameters');q('[data-nav="overview"]').onclick=()=>navigate('dashboard');q('[data-nav="logs"]').onclick=()=>{navigate('commands');q('.terminal-panel').scrollIntoView({behavior:'smooth',block:'end'});};
q('#connection').onclick=()=>{connectionDrawer.hidden=false;q('#transport-mode').focus();};
q('.device').innerHTML='<span class="dot"></span>本地调试台<br><b>BM75 · STM32F4</b><br><span class="dim">命令与参数 · 源码同步</span>';
q('#save').textContent='全部参数 ↗';q('#save').onclick=()=>navigate('parameters');
const homeDefaults={speed_kp:q('[aria-label="速度环 Kp"]'),speed_ki:q('[aria-label="速度环 Ki"]'),speed_iq_limit:q('[aria-label="电流限制"]')};
for(const [name,input]of Object.entries(homeDefaults)){input.value=draft[name]??'';input.readOnly=true;}
q('.param-note').textContent='3508 源码默认值预览，并非设备回读。完整编辑请打开“电机参数”。';

function appendLog(type,text){terminal.push(`${new Date().toLocaleTimeString('zh-CN',{hour12:false})} ${type} ${text}`);if(terminal.length>1000)terminal.splice(0,terminal.length-1000);logDirty=true;}
setInterval(()=>{if(!logDirty)return;const pre=q('#terminal-output');const nearEnd=pre.scrollHeight-pre.scrollTop-pre.clientHeight<70;pre.textContent=terminal.join('\n');if(nearEnd)pre.scrollTop=pre.scrollHeight;q('#traffic').textContent=`RX ${rxBytes.toLocaleString()} B / TX ${txBytes.toLocaleString()} B`;logDirty=false;},100);
function notify(text){q('#toast').textContent=text;q('#toast').style.display='block';clearTimeout(window.workbenchToast);window.workbenchToast=setTimeout(()=>q('#toast').style.display='none',3500);}
function renderMenu(){const menu=q('#command-menu');menu.replaceChildren();const term=q('#command-search').value.toLowerCase();for(const cat of [...new Set(catalog.commands.map(c=>c.category))]){const list=catalog.commands.filter(c=>c.category===cat && [c.name,c.title,commandHelp[c.name]?.purpose,...c.subcommands,...Object.entries(actionHelp).filter(([key])=>key.startsWith(c.name+' ')).map(([,help])=>help.title)].join(' ').toLowerCase().includes(term));if(!list.length)continue;menu.append(el('h3',{},cat));for(const c of list){const b=el('button',{class:'command-item'+(active===c?' selected':'')});b.append(el('span',{},c.title),el('code',{},c.name));b.onclick=()=>selectCommand(c.name);menu.append(b);}}if(!menu.children.length)menu.append(el('p',{class:'studio-note'},'没有匹配的命令'));}
function selectCommand(name,args=''){active=catalog.commands.find(c=>c.name===name);if(!active)return;renderMenu();const detail=q('#command-detail');const intro=commandHelp[name];detail.replaceChildren(el('h2',{},intro.title),el('div',{class:'command-purpose'},intro.purpose));
  const choices=el('div',{class:'subcommands'});for(const sub of active.subcommands){const help=describeCommand(name+' '+sub);const b=el('button',{title:help.purpose});b.append(el('span',{},help.title),el('code',{},sub));b.onclick=()=>{q('#command-args').value=sub;updatePreview();};choices.append(b);}detail.append(choices);
  const ex=el('div',{class:'command-examples'});for(const example of examples[active.name]||[]){const help=describeCommand(example);const b=el('button',{class:'explained-example',title:help.purpose});b.append(el('strong',{},help.title),el('span',{},help.purpose),el('code',{},example));b.onclick=()=>{q('#command-args').value=example.split(' ').slice(1).join(' ');updatePreview();};ex.append(b);}detail.append(ex);
  detail.append(el('section',{class:'command-explanation',id:'command-explanation','aria-live':'polite'}));
  const raw=el('details',{class:'command-raw'});raw.append(el('summary',{},'展开原始命令用法与源码位置'),el('div',{class:'source-note'},`${active.name} · ${active.file}:${active.line} · 是否启用由设备固件编译决定`),el('pre',{class:'usage'},[helpOverrides[active.name],...active.usage].filter(Boolean).join('\n') || active.description));detail.append(raw);
  const label=el('label',{class:'command-arg'},'参数输入（点击中文功能可填入命令；下方说明会随输入变化）');const input=el('input',{class:'studio-input',id:'command-args',placeholder:intro.parameters,autocomplete:'off'});input.value=args;input.oninput=updatePreview;label.append(input);detail.append(label,el('div',{class:'cmd-preview',id:'cmd-preview'}));
  const row=el('div',{class:'command-detail-actions'});const copy=el('button',{},'复制命令');copy.onclick=async()=>{try{await navigator.clipboard.writeText(commandLine());notify('命令已复制');}catch{notify('复制不可用，请从预览区选中命令复制');}};const send=el('button',{class:'primary',id:'command-send'},'记录演示命令');send.onclick=()=>sendLine(commandLine());row.append(send,copy);detail.append(row,el('div',{id:'operation-message',class:'operation-message'}));
  if(['detect','saveparam','reset','bldc','sltest','curtest','duty','joint','pos'].includes(name))detail.append(el('div',{class:'workbench-warning'},'此入口包含运动、测试、校准或持久化操作。实机模式下只在明确点击发送后下发；检测结果和写 Flash 的语义按固件回复判断。'));
  updatePreview();
}
function commandLine(){return active.name+(q('#command-args').value.trim()?' '+q('#command-args').value.trim():'');}
function updatePreview(){q('#cmd-preview').textContent=commandLine();q('#command-send').textContent=transport==='demo'?'记录演示命令':'发送至设备';q('#operation-message').textContent=transport==='demo'?'点击功能卡片只填入参数；仅记录命令预览，不执行设备操作。':'点击功能卡片只填入参数；发送后请在终端核对设备回复。';const help=describeCommand(commandLine());const box=q('#command-explanation');box.replaceChildren(el('h3',{},help.title),el('p',{class:'explanation-purpose'},help.purpose));for(const [label,text]of [['参数怎么填',help.parameters],['使用条件',help.conditions],['执行后影响',help.effect]]){const row=el('div',{class:'explanation-row'});row.append(el('strong',{},label),el('span',{},text));box.append(row);}for(const b of document.querySelectorAll('.explained-example')){const selected=b.querySelector('code').textContent===commandLine();b.classList.toggle('selected',selected);b.setAttribute('aria-pressed',String(selected));}}
q('#command-search').oninput=renderMenu;
for(const [text,line]of [['状态','olstatus'],['停止电机','olstop'],['故障','olfault'],['参数检测','detect'],['编码器','encoder show'],['波形','plot help'],['任务','task']]){const b=el('button',{},text);b.onclick=()=>{const [name,...args]=line.split(' ');selectCommand(name,args.join(' '));};q('#quick-actions').append(b);}
selectCommand('olstatus');

const decoder=new SerialDecoder(text=>{
  for(const listener of textListeners)listener(text);
  textTail+=text.replace(/\x1b\[[0-9;]*[a-zA-Z]/g,'');const lines=textTail.split(/\r?\n/);textTail=lines.pop().slice(-4096);for(const line of lines)if(line.trim())appendLog('RX',line);
},(mode,values)=>{lastFrameTime=performance.now();if(mode==='current')q('.cards .card:nth-child(3) .metric').textContent=values[3].toFixed(2)+' V';if(mode==='speed'){q('#rpm').textContent=Math.round(values[1]).toLocaleString();q('#targetLabel').textContent=Math.round(values[0]).toLocaleString();}if(mode==='pos'){q('#iq').textContent=values[3].toFixed(2);q('#id').textContent=values[4].toFixed(2);}if(mode==='sine'){q('#id').textContent=values[3].toFixed(2);q('#iq').textContent=values[4].toFixed(2);}points.push({time:Date.now(),mode,values});if(points.length>1500)points.shift();logDirty=true;});
const wireDecoder=new StudioDecoder(packet=>{for(const listener of packetListeners)listener(packet);},bytes=>decoder.push(bytes));
function modeState(){window.focHardwareMode=transport==='serial';scopeView.classList.toggle('monitor-hardware',transport==='serial');q('#transport-chip').textContent=transport==='demo'?'演示模式 · 不发送硬件指令':serialConnected?'串口已连接 · 实机模式':'实机模式 · 未连接';q('#serial-connect').disabled=transport!=='serial'||serialConnected||closing;q('#serial-disconnect').disabled=!serialConnected;q('#serial-baud').disabled=serialConnected;q('#transport-mode').disabled=serialConnected||closing;q('#linkState').textContent=transport==='demo'?'模拟连接已建立':serialConnected?'串口已连接 · 状态待查询':'实机串口未连接';q('.badge').textContent=transport==='demo'?'LOCAL DEMO':'SERIAL MODE';
  demoScopeHost.hidden=transport==='serial';deviceScopeHost.hidden=transport!=='serial';q('#scope-source').textContent=transport==='serial'?'设备信号 · JustFloat 串口采集':'演示信号 · 始终可见';
  dashboard.classList.toggle('hardware-active',transport==='serial');hardwareNotice.hidden=transport!=='serial';
  for(const id of ['run','stop','target','slider','disconnect'])q('#'+id).disabled=transport==='serial';for(const b of document.querySelectorAll('[data-mode]'))b.disabled=transport==='serial';
  q('#status').textContent=transport==='serial'?'设备状态未知':'演示模式';q('#capture').textContent=transport==='serial'?'设备波形采集中':'● 正在采集模拟数据';
  if(transport==='serial'){q('#targetLabel').textContent='—';q('.cards .card:nth-child(3) .card-foot').textContent='等待设备电压数据';for(const id of ['rpm','iq','id'])q('#'+id).textContent='—';q('.cards .card:nth-child(3) .metric').textContent='— V';q('#statusFoot').textContent='请查询 olstatus，未由发送推断状态';q('#tracking').textContent='等待设备反馈';q('#scope').getContext('2d').clearRect(0,0,q('#scope').width,q('#scope').height);}
  updatePreview();for(const listener of stateListeners)listener({transport,connected:serialConnected});}
q('#transport-mode').onchange=()=>{transport=q('#transport-mode').value;modeState();if(transport==='demo'){q('.cards .card:nth-child(3) .metric').innerHTML='24.08<small>V</small>';q('.cards .card:nth-child(3) .card-foot').textContent='演示电压 · DC BUS';}};
async function disconnect(){if(closing)return;if(sending)return notify('串口仍在发送，请完成后再断开');closing=true;modeState();try{if(reader)await reader.cancel();if(readTask)await readTask;if(serialPort)await serialPort.close();}catch(error){appendLog('ERR','关闭串口：'+error.message);}finally{reader=null;serialPort=null;serialConnected=false;closing=false;decoder.setMode(null);q('#transport-status').textContent='串口已断开。断开通信不等于电机已经停机。';modeState();}}
q('#serial-disconnect').onclick=disconnect;
q('#serial-connect').onclick=async()=>{
  if(!('serial'in navigator)){q('#transport-status').textContent='当前浏览器不支持 Web Serial。请在桌面 Chrome / Edge 打开本地地址。';return;}
  q('#serial-connect').disabled=true;let candidate;
  try{candidate=await navigator.serial.requestPort();await candidate.open({baudRate:Number(q('#serial-baud').value),dataBits:8,stopBits:1,parity:'none',flowControl:'none',bufferSize:65536});serialPort=candidate;serialConnected=true;rxBytes=0;txBytes=0;plotMode=null;points=[];deviceValues.clear();wireDecoder.buffer=new Uint8Array();wireDecoder.badCrc=0;decoder.setMode(null);q('#transport-status').textContent='串口已连接，正在探测只读监视接口；不会自动启动电机。';appendLog('LINK','串口连接成功 · '+q('#serial-baud').value+' 8N1');readTask=readSerial();}
  catch(error){q('#transport-status').textContent=error.name==='NotFoundError'?'已取消串口选择':'连接失败：'+error.message;serialPort=null;serialConnected=false;}
  finally{modeState();}
};
async function readSerial(){
  try{while(serialPort?.readable&&!closing){reader=serialPort.readable.getReader();try{for(;;){const {value,done}=await reader.read();if(done)break;if(value){rxBytes+=value.byteLength;wireDecoder.push(value);logDirty=true;}}}finally{reader.releaseLock();reader=null;}if(!closing)break;}}
  catch(error){if(!closing)appendLog('ERR','接收中断：'+error.message);}
  finally{if(!closing){const old=serialPort;serialPort=null;serialConnected=false;try{await old?.close();}catch{}q('#transport-status').textContent='串口读取结束 / 设备断开，电机状态未知。';modeState();}}
}
const history=[];let historyIndex=0;
async function sendLine(input,{batch=false}={}){
  if(batchBusy&&!batch){notify('组合正在执行，请先中止组合');return false;}
  let line;try{line=validateCommand(input,known);}catch(error){notify(error.message);return;}
  if(sending){notify('上一条命令仍在写入，请稍后');return;}
  if(transport==='demo'){appendLog('PREVIEW',line+'（未发送）');q('#operation-message').textContent='已记录演示预览，未连接或执行硬件。';notify('已记录命令预览，未发送');remember(line);return;}
  if(!serialConnected||!serialPort?.writable){notify('请先选择串口并连接设备');return;}
  sending=true;let writer;const prevMode=plotMode;
  try{
    const args=line.split(/\s+/);if(args[0]==='plot'&&args[1]==='create'&&plotChannels[args[2]==='sin'?'sine':args[2]]){plotMode=args[2]==='sin'?'sine':args[2];decoder.setMode(plotMode);points=[];}
    writer=serialPort.writable.getWriter();const data=new TextEncoder().encode(line+'\r');await writer.write(data);txBytes+=data.length;appendLog('TX',line+'（已写入串口，等待设备回复）');remember(line);
    if(args[0]==='plot'&&['destory','destroy','stop'].includes(args[1])){plotMode=null;decoder.setMode(null);}
    q('#operation-message').textContent='已写入串口；是否成功以 RX 回复为准。';return true;
  }catch(error){plotMode=prevMode;decoder.setMode(prevMode);appendLog('ERR','发送失败：'+error.message);notify('发送失败：'+error.message);}
  finally{writer?.releaseLock();sending=false;}
}
function remember(line){history.push(line);if(history.length>100)history.shift();historyIndex=history.length;}
q('#terminal-form').onsubmit=e=>{e.preventDefault();const line=q('#terminal-input').value;const [name,...args]=line.trim().split(/\s+/);if(known.includes(name))selectCommand(name,args.join(' '));sendLine(line);};
q('#terminal-input').onkeydown=e=>{if(e.key==='ArrowUp'||e.key==='ArrowDown'){e.preventDefault();historyIndex=Math.max(0,Math.min(history.length,historyIndex+(e.key==='ArrowUp'?-1:1)));e.target.value=history[historyIndex]||'';}};
q('#watch-break').onclick=async()=>{if(transport==='demo'){appendLog('PREVIEW','空格（未发送）');return;}if(!serialConnected||sending)return notify('串口未连接或正在发送');let writer;sending=true;try{writer=serialPort.writable.getWriter();await writer.write(new Uint8Array([32]));txBytes++;appendLog('TX','[空格] · 请求退出 watch');}catch(error){notify(error.message);}finally{writer?.releaseLock();sending=false;}};
function download(name,data,type){const url=URL.createObjectURL(new Blob([data],{type}));const a=el('a',{href:url,download:name});a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
q('#terminal-clear').onclick=()=>{terminal=[];logDirty=true;};q('#terminal-export').onclick=()=>download('foc-terminal.txt',terminal.join('\n')+'\n'+textTail,'text/plain;charset=utf-8');

const operational=[...['fw_current_max','fw_duty_start','fw_ramp_a_s','ol_iq_set','ol_id_set','ol_target_rpm','ol_mode'].map(name=>({name,type:'float',default:catalog.defaults[name],description:'运行命令 / 临时状态，不作为持久化配置'})),
  ...['feedback','sldt','position_kp','position_max','joint_kp','joint_kd','joint_ki','joint_lim'].map(name=>({name,type:'float',default:null,description:'使用专用命令设置；设备当前值未知'}))];
const tabs={config:['持久配置',catalog.config],operational:['命令与临时参数',operational],calibration:['校准数据',catalog.calibration],runtime:['实时字段',catalog.runtime],profiles:['电机档案',[]]};
for(const [key,[label]]of Object.entries(tabs)){const b=el('button',{'data-param-tab':key},label);b.onclick=()=>{currentTab=key;renderParams();};q('#parameter-tabs').append(b);}
for(const [i,p]of catalog.profiles.entries())q('#profile-select').append(el('option',{value:i},p.name));
q('#profile-select').onchange=()=>{profile=catalog.profiles[Number(q('#profile-select').value)];resetDraft();renderParams();notify('已切换源码参考档案；未切换实际电机、未发送命令');};q('#param-search').oninput=renderParams;
function previewLine(line){const [name,...args]=line.split(' ');navigate('commands');selectCommand(name,args.join(' '));q('#cmd-preview').scrollIntoView({behavior:'smooth',block:'center'});}
function buildParameterCommand(name){
  const checkValue=key=>{const value=Number(numericDraft(draft[key]));const type=catalog.config.find(f=>f.name===key)?.type;if(type==='bool'&&value!==0&&value!==1)throw Error((labels[key]||key)+' 只能为 0 或 1');if(type==='uint16_t'&&(!Number.isInteger(value)||value<0||value>65535))throw Error((labels[key]||key)+' 需要有效的 16 位无符号整数');return String(value);};
  if(structuredReady&&currentTab==='config'){const index=catalog.config.findIndex(f=>f.name===name);if(index>=0&&name!=='ol_foc_freq')return 'studio set '+index+' '+checkValue(name);}
  if(single[name]){if(name==='feedback'){const value=String(draft[name]??'');if(!['openloop','sensorless','encoder','hall'].includes(value))throw Error('反馈源请填写 openloop / sensorless / encoder / hall');return 'olfb '+value;}
    return single[name]+' '+checkValue(name);}
  const group=bundles.find(x=>x[1].includes(name));if(!group)throw Error('该字段没有独立 Shell 写入入口');
  return group[0]+' '+group[1].map(key=>{const n=Number(checkValue(key));return group[0]==='slmotor'&&['sensorless_motor_l','sensorless_flux_linkage'].includes(key)?String(Number((n*1000).toPrecision(10))):String(n);}).join(' ');
}
function renderParams(){
  q('#profile-note').textContent='参考档案：'+profile.name+'。以下是源码默认值和本地草稿，不是设备读回值。切换档案仅改变参考。组合参数生成命令时会带上同组全部草稿值，请逐项核对。3508 的 KV 与 R/L/磁链来源仍有待核对项，详见电机档案。';
  for(const b of document.querySelectorAll('[data-param-tab]'))b.classList.toggle('active',b.dataset.paramTab===currentTab);
  q('#profile-grid').hidden=currentTab!=='profiles';q('#parameter-table').hidden=currentTab==='profiles';q('#parameter-count').textContent=currentTab==='profiles'?'编译期电机档案 · 只读':tabs[currentTab][0]+' · '+tabs[currentTab][1].length+' 项';
  if(currentTab==='profiles'){renderProfiles();return;}
  const term=q('#param-search').value.toLowerCase();const fields=tabs[currentTab][1].filter(f=>[f.name,labels[f.name],f.description].join(' ').toLowerCase().includes(term));
  const table=el('table',{class:'param-table'}),head=el('thead'),row=el('tr');for(const label of ['参数 / 单位','源码默认值','本地草稿','设备实际值','操作'])row.append(el('th',{},label));head.append(row);table.append(head);const body=el('tbody');
  for(const field of fields){const tr=el('tr'),name=el('td');name.append(el('div',{},labels[field.name]||field.name),el('code',{},field.name+(field.array||'')),el('small',{},units[field.name]||field.description||field.type));tr.append(name);
    const defaultCell=el('td');const v=resolve(field.default);defaultCell.append(el('span',{class:'param-value'},v==null?'—':String(v)),el('small',{class:'source-default'},field.default||'源码未直接提供标量默认值'));tr.append(defaultCell);
    const editable=['config','operational'].includes(currentTab)&&!!(single[field.name]||bundles.find(x=>x[1].includes(field.name))||structuredReady&&currentTab==='config'&&field.name!=='ol_foc_freq');
    const draftCell=el('td');if(editable){const input=el('input',{class:'studio-input',type:field.name==='feedback'?'text':'number','aria-label':labels[field.name]||field.name,'data-param':field.name,step:field.type==='bool'||field.type==='uint16_t'?'1':'any'});if(field.type==='bool'){input.min='0';input.max='1';}input.value=draft[field.name]??'';input.oninput=()=>draft[field.name]=input.value;draftCell.append(input);}else draftCell.append(el('span',{class:'readonly-pill'},'只读 / 未提供写入命令'));const actual=el('td',{class:'dim device-readback','data-field':field.name},deviceValues.has(field.name)?valueText(deviceValues.get(field.name).value)+(structuredReady?' · 已回读':' · 离线缓存'):'未知 · 未读回');tr.append(draftCell,actual);
    const action=el('td');if(editable){const b=el('button',{},'预览修改');b.onclick=()=>{try{previewLine(buildParameterCommand(field.name));}catch(error){notify(error.message);}};action.append(b);}else if(currentTab==='calibration'){const command=field.name.startsWith('encoder')?'encoder show':field.name==='hall_table'?'detect hall':'detect';const b=el('button',{},'查看专用命令');b.onclick=()=>previewLine(command);action.append(b);}else action.append(el('small',{},currentTab==='runtime'?'通过诊断 / plot 查看':'需固件配置接口支持'));
    tr.append(action);body.append(tr);
  }
  if(!fields.length){const tr=el('tr');tr.append(el('td',{colspan:5},'没有匹配的参数'));body.append(tr);}table.append(body);q('#parameter-table').replaceChildren(table);
}
function renderProfiles(){q('#profile-grid').replaceChildren();for(const p of catalog.profiles){const card=el('section',{class:'panel profile-card'});card.append(el('h2',{},p.name),el('div',{class:'source-note'},p.file),el('pre',{},p.note));const details=el('details'),summary=el('summary',{},'查看全部铭牌、控制默认值与编译宏（'+p.macros.length+' 项）'),list=el('div',{class:'macro-list'});for(const macro of p.macros){const row=el('div',{class:'macro-row'});row.append(el('code',{},macro.name),el('span',{},macro.value),el('div',{},macro.description));list.append(row);}details.append(summary,list);card.append(details);q('#profile-grid').append(card);}}
q('#draft-export').onclick=()=>download('foc-parameter-draft.json',JSON.stringify({kind:'local-draft-and-device-readback',reference:profile.file,draft,device:Object.fromEntries(deviceValues)},null,2),'application/json');
q('#query-parameters').textContent='读取设备参数';q('#query-parameters').onclick=()=>{if(structuredReady)sendLine('studio params');else{previewLine('studio on');notify('请先连接支持 studio 接口的新固件');}};
q('#flash-save').onclick=()=>previewLine('saveparam');renderParams();

function paintPlot(){const canvas=q('#real-scope');if(!canvas.clientWidth)return;const ctx=canvas.getContext('2d');const w=canvas.clientWidth,h=220,dpr=Math.min(devicePixelRatio||1,2);if(canvas.width!==Math.round(w*dpr)||canvas.height!==h*dpr){canvas.width=Math.round(w*dpr);canvas.height=h*dpr;}ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);const list=points.filter(p=>p.mode===plotMode).slice(-300);const colors=['#5be2b3','#66a8ff','#f2b86b','#c59bff','#ef889e'];ctx.font='10px Consolas';for(let i=0;i<5;i++){let y=15+i*45;ctx.strokeStyle='#25303b';ctx.beginPath();ctx.moveTo(40,y);ctx.lineTo(w-10,y);ctx.stroke();}
  if(!list.length){ctx.fillStyle='#7d94aa';ctx.fillText('等待设备数据',45,108);q('#stream-status').textContent='尚未收到设备波形';q('#stream-values').textContent=structuredReady?'等待设备状态帧；曲线切换只改变显示，不重复发送采集命令':'连接新版固件后自动显示设备数据；旧固件可使用 plot 命令';return;}
  const max=Math.max(1,...list.flatMap(p=>p.values.map(Math.abs)));const channels=plotChannels[plotMode];ctx.fillStyle='#7d94aa';ctx.fillText(max.toPrecision(3),0,15);ctx.fillText('0',20,105);ctx.fillText('-'+max.toPrecision(3),0,195);
  for(let c=0;c<channels.length;c++){ctx.strokeStyle=colors[c];ctx.beginPath();list.forEach((p,i)=>{const x=40+i/Math.max(1,list.length-1)*(w-50),y=105-p.values[c]/max*90;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.stroke();}
  q('#stream-values').replaceChildren(...channels.map((name,i)=>el('span',{style:'color:'+colors[i]},name+': '+list.at(-1).values[i].toPrecision(5))));q('#stream-status').textContent=`${plotMode} · ${points.length} 帧 · ${Math.round(performance.now()-lastFrameTime)} ms 前收到 · 混合单位自动量程`;
}
setInterval(()=>{if(!scopeView.hidden&&transport==='serial')paintPlot();},100);
new ResizeObserver(()=>{if(!scopeView.hidden&&transport==='serial')paintPlot();}).observe(q('#real-scope'));
q('#stream-export').onclick=()=>{if(!points.length)return notify('还没有收到实机波形，未导出模拟数据');const mode=points[0].mode;download('foc-device-'+mode+'.csv','\ufeffhost_time_ms,device_time_us,'+plotChannels[mode].join(',')+'\n'+points.filter(p=>p.mode===mode).map(p=>[p.time,p.deviceUs??'',...p.values].join(',')).join('\n'),'text/csv;charset=utf-8');};
navigate('commands');
appendLog('INFO',`目录已加载：${catalog.commands.length} 条固件命令 / ${catalog.config.length} 项配置。演示模式不发送设备指令。`);
modeState();
async function writeControlBytes(bytes){if(!serialConnected||!serialPort?.writable||closing)throw Error('串口未连接');if(sending)throw Error('串口正在处理手动命令');sending=true;let writer;try{writer=serialPort.writable.getWriter();await writer.write(bytes);txBytes+=bytes.length;logDirty=true;}finally{writer?.releaseLock();sending=false;}}
async function exchangeShell(line,expected){
  if(transport!=='serial'||!serialConnected)throw Error('串口未连接');
  let reply='',timer,listener;
  const answer=new Promise((resolve,reject)=>{listener=text=>{reply=(reply+text.replace(/\x1b\[[0-9;]*[a-zA-Z]/g,'')).slice(-12000);if(/\$\s*$/.test(reply)&&expected.test(reply)){if(/invalid|failed|unknown|not found|stop motor before|requires|need speed|clear motor fault|control lease|studio off first/i.test(reply))reject(Error(reply.trim()));else resolve(reply);}};textListeners.add(listener);timer=setTimeout(()=>reject(Error('回复未确认，已中止：'+line+'\n'+reply.trim())),3000);});
  try{const sent=await sendLine(line,{batch:true});if(!sent){clearTimeout(timer);throw Error('命令未写入串口：'+line);}return await answer;}finally{clearTimeout(timer);textListeners.delete(listener);}
}
const fullAPI={q,el,catalog,notify,appendLog,sendLine,navigate,previewLine,onText:listener=>textListeners.add(listener),exchangeShell,setBatch:value=>batchBusy=value,write:writeControlBytes,getState:()=>({transport,connected:serialConnected,sending,batch:batchBusy}),onPacket:listener=>packetListeners.add(listener),onState:listener=>stateListeners.add(listener),readback:receiveValue,values:deviceValues,setReady:value=>{structuredReady=value;renderParams();},download,
  copyDeviceDraft:()=>{for(const field of catalog.config)if(deviceValues.has(field.name))draft[field.name]=deviceValues.get(field.name).value;renderParams();},
  pushTelemetryPoint:(mode,values,timeUs)=>{if(plotMode!==mode)points=[];plotMode=mode;lastFrameTime=performance.now();points.push({time:Date.now(),deviceUs:timeUs,mode,values});if(points.length>1500)points.shift();},badCrc:()=>wireDecoder.badCrc};
import('./full-studio.js').then(({mount})=>mount(fullAPI)).catch(error=>{appendLog('ERR','完整通信模块加载失败：'+error.message);notify('通信模块加载失败，请刷新页面');});
