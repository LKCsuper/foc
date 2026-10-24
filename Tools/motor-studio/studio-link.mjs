export const STUDIO_PERIOD_MS=20;
export const STUDIO_TIMEOUT_MS=300;
export function detectionCommand(kind,fields=[]){
  const calibration={offset:'detect offset ram',encoder:'encoder detect ram',hall:'detect hall',phase:'detect phase'};
  if(calibration[kind]){if(fields.some(v=>String(v??'').trim()))throw Error('此校准项目不接受 R/L 检测参数');return calibration[kind];}
  if(!['rl','motor','fluxscan'].includes(kind))throw Error('未知检测项目');
  const max=kind==='rl'?1:3;if(fields.length>max)throw Error('检测参数过多');let empty=false;const args=[];
  for(let i=0;i<fields.length;i++){const raw=String(fields[i]??'').trim();if(!raw){empty=true;continue;}if(empty)throw Error('自定义后面的参数时，请先填写前面的测试电流');const n=Number(raw);if(!Number.isFinite(n)||n<=0)throw Error('检测参数必须为正有限数值');if(kind==='fluxscan'&&i===1&&(n<900||n>3000))throw Error('磁链扫描最大电转速为 900～3000 eRPM');args.push(n);}
  return 'detect '+kind+(args.length?' '+args.join(' '):'');
}
export function detectionCompletion(kind,text){
  if(/Encoder detect rejected:|Encoder detect FAILED:|Invalid detection arguments|Failed to create|already running|Unknown command|not found|Detect End:\s*FAILED/i.test(text))return false;
  if(kind==='encoder'){const done=text.match(/Encoder detect:\s*count=[^\r\n]*zero_ok=(\d)/);return done?done[1]==='1':null;}
  const done=text.match(['offset','hall','phase'].includes(kind)?/Detect End:\s*(SUCCESS|FAILED)/:/Detect motor finished:\s*(SUCCESS|FAILED)/);return done?done[1]==='SUCCESS':null;
}
export function parseDetectionResult(text){
  const start=text.lastIndexOf('===== Motor Parameter Detect');if(start<0)return null;const body=text.slice(start);
  const scalar=(name,unit)=>{const match=body.match(new RegExp(name+'\\s*:\\s*(VALID|INVALID)\\s+([-+\\deE.]+)\\s*'+unit));if(!match)return null;const value=Number(match[2]);return {valid:match[1]==='VALID'&&Number.isFinite(value)&&value>0,value:Number.isFinite(value)?value:null};};
  return {status:body.match(/Status\s*:\s*([^\r\n]+)/)?.[1]?.trim()??'未知',resistance:scalar('Resistance','ohm'),inductance:scalar('Inductance','uH'),flux:scalar('Flux','mWb')};
}
export function buildModeRecipe({mode='open',target=300,feedback='encoder',hold=true,device={}}){
  const value=Number(target),limit=Number(device.speed_iq_limit),max=Number(device.ol_max_rpm);
  if(String(target).trim()===''||!Number.isFinite(value))throw Error('目标必须是有限数值');
  if(!['open','current','sensored','sensorless','position'].includes(mode))throw Error('未知电机模式');
  if(!Number.isFinite(max)||max<=0||!Number.isFinite(limit)||limit<=0)throw Error('请先完成设备参数回读');
  if(mode!=='current'&&mode!=='position'&&(value===0||Math.abs(value)>max))throw Error('目标必须非零且不超过设备最大转速 '+max+' rpm');
  if(mode==='current'&&(value<=0||value>limit))throw Error('当前快捷电流模式支持正向 Iq，须大于 0 且不超过设备电流预算 '+limit+' A');
  if(['current','sensored','position'].includes(mode)){
    if(!['encoder','hall'].includes(feedback)||mode==='position'&&feedback!=='encoder')throw Error('位置模式需要编码器，电流/速度模式需要编码器或霍尔');
    if(feedback==='encoder'&&device.encoder_calibrated!==1)throw Error('编码器尚未回读到有效标定，请先检测/标定');
    if(feedback==='hall'&&(!Array.isArray(device.hall_table)||![1,2,3,4,5,6].every(i=>Number.isFinite(device.hall_table[i])&&device.hall_table[i]>=0&&device.hall_table[i]<200)))throw Error('请先回读有效霍尔标定表');
  }
  if(mode==='open')return buildOpenLoopRecipe({mode:'if',rpm:value,start:true,device,options:{iq:Math.min(.3,limit),id:0}});
  if(mode==='sensorless')return buildOpenLoopRecipe({mode:'slrun',rpm:value,start:true,speedLoop:true,device,options:{iq:Math.min(.3,limit),id:0}});
  const steps=[],add=(line,title,expected)=>steps.push({line,title,expected});
  add('olstop','先停机，退出检测及位置控制',/Open-loop stopped/);
  add('spenable 0','复位外环选择',/Speed loop disabled/);
  add('sltakeover 0','关闭无感接管',/Sensorless takeover blocked/);
  add('olmode 1','选择电流控制路径',/Open-loop mode set to 1/);
  add('olfb '+feedback,'选择'+(feedback==='encoder'?'编码器':'霍尔')+'角度反馈',new RegExp('feedback set to '+(feedback==='encoder'?2:3)));
  if(mode==='position'){add(hold?'pos hold':'pos set '+value,hold?'保持启动时当前位置':'设置相对已有位置零点的角度 '+value+'°',/Position: enabled=1/);}
  else if(mode==='current'){add('olid 0','Id 默认 0 A',/Id set to/);add('oliq '+value,'设置 Iq '+value+' A，速度环关闭',/Iq set to/);add('olspeed 1','设置当前固件的非零使能参考；不作为实际转速目标或限速',/Target RPM set to/);}
  else {add('olspeed '+value,'设置目标 '+value+' rpm',/Target RPM set to/);add('spenable 1','启用有感速度环',/Speed loop enabled/);}
  add('olstart','启动所选模式',/Open-loop started/);add('olstatus','查询运行结果',/Open-Loop Status/i);return steps;
}
/** 按当前时间窗口内的有效数据计算量程；速度不强制包含零点。 */
export function scopeRange(rows,key){
  let min=key==='speed'?Infinity:0,max=key==='speed'?-Infinity:0;
  for(const row of rows)for(const value of row)if(Number.isFinite(value)){min=Math.min(min,value);max=Math.max(max,value);}
  if(!Number.isFinite(min)||!Number.isFinite(max)){min=0;max=0;}
  const pad=Math.max((max-min)*.12,key==='speed'?10:.05);
  return {min:min-pad,max:max+pad};
}
export function startAvailability({connected,ready,batch,pending,active,telemetry,age}){
  let reason='';
  if(!connected)reason='请先连接真实串口';else if(batch)reason='组合正在执行';else if(!ready||!telemetry)reason='等待设备握手与状态';else if(age>150)reason='设备状态已过期，等待新遥测';else if(pending)reason='正在等待周期会话授权';else if(telemetry.faultLatched)reason='设备故障锁存，请先排查';else if(telemetry.enabled)reason='电机已在运行';else if(active&&(!telemetry.owned||!telemetry.appliedSequence))reason='等待设备确认周期目标';else if(!active&&telemetry.owned)reason='请先退出设备原有周期会话';else if(!active&&(!Number.isFinite(telemetry.targetRpm)||telemetry.targetRpm===0))reason='请先通过组合操作设置非零目标';
  return {enabled:!reason,reason,line:active?'studio start':'olstart',label:active?'启动周期目标':'启动设备当前目标'};
}
/** 组合为有依赖的命令步骤，所有输入先校验，启动固定置于最后配置之后。 */
export function buildOpenLoopRecipe({mode='if',rpm=300,options={},start=false,speedLoop=false,device={}}){
  if(!['if','vf','slobserve','slrun'].includes(mode))throw Error('请选择开环或无感场景');
  const sensorless=mode.startsWith('sl'),currentMode=mode!=='vf';
  function number(value,label,min,max){if(value==null||String(value).trim()==='')throw Error(label+'不能为空');const n=Number(value);if(!Number.isFinite(n)||n<min||n>max)throw Error(label+'范围为 '+min+'～'+max);return n;}
  rpm=number(rpm,'目标转速',-100000,100000);if(rpm===0&&start)throw Error('目标为零时请勿组合启动，固件会使用默认起步转速');
  const steps=[],add=(line,title,expected)=>steps.push({line,title,expected});
  add('olstop','先停机，退出位置控制及检测',/Open-loop stopped/);
  add('spenable 0','关闭速度闭环，让转速按开环角度推进',/Speed loop disabled/);
  add('sltakeover 0','禁止无感观测器接管角度',/Sensorless takeover blocked/);
  add('olfb openloop','选择开环电角度，不使用编码器或霍尔闭环',/feedback set to 0/);
  add('olmode '+(currentMode?1:0),currentMode?'I/F 电流模式：用于开环起步或无感控制':'V/F：按电频率设置驱动电压',/Open-loop mode set to/);
  const fields={max:['olmax','最大转速',1,100000,/Max RPM set/],ramp:['olramp','转速斜坡 RPM/s',0,100000,/Ramp set/],limit:['olvlimit','电压限幅 V',0,1000,/Vq_limit set/],boost:['olboost','V/F 启动补偿 V',0,1000,/V_boost set/],kvf:['olkvf','V/F 斜率 V/Hz',0,1000,/Kv_f set/],kp:['olkp','电流环 Kp',0,100000,/Kp set/],ki:['olki','电流环 Ki',0,100000,/Ki set/],iq:['oliq','目标 Iq A',-1000,1000,/Iq set to/],id:['olid','目标 Id A',-1000,1000,/Id set to/]};
  for(const [key,definition]of Object.entries(fields)){if(options[key]==null)continue;if(!currentMode&&['iq','id','kp','ki'].includes(key)||currentMode&&['boost','kvf'].includes(key))continue;const [cmd,label,min,max,expected]=definition,value=number(options[key],label,min,max);if(key==='max'&&Math.abs(rpm)>value)throw Error('目标转速超过所选最大转速');add(cmd+' '+value,'设置'+label,expected);}
  if(sensorless){
    for(const [key,cmd,label,expected]of [['spkp','spkp','速度环 Kp',/Speed Kp set/],['spki','spki','速度环 Ki',/Speed Ki set/],['spiq','spiq','速度环电流上限',/Speed Iq limit set/]])if(options[key]!=null)add(cmd+' '+number(options[key],label,0,100000),'设置'+label,expected);
    const groups={model:['slmotor','电机模型（Ω / mH / mWb）',[1e-9,1e-9,1e-9],[1000,100000,100000],/Sensorless motor set/],threshold:['slrpm','接管与回退转速',[1,0],[100000,100000],/Sensorless transition\/fallback set/],observer:['slobs','观测器增益与 PLL',[1e-9,0,0],[1e15,1e9,1e9],/Observer gain=/],guard:['slguard','开环护栏、相位容差和融合时间',[1,.1,.05],[100000,1.5,1],/Sensorless guard set/]};
    for(const [key,[cmd,label,mins,maxs,expected]]of Object.entries(groups)){if(options[key]==null)continue;if(!Array.isArray(options[key])||options[key].length!==mins.length)throw Error(label+'需要完整填写同组参数');const values=options[key].map((v,i)=>number(v,label,mins[i],maxs[i]));if(key==='threshold'&&values[1]>=values[0])throw Error('回退转速必须小于接管转速');add(cmd+' '+values.join(' '),'设置'+label,expected);}
    if(options.phase!=null)add('slphase '+number(options.phase,'无感相位 rad',-100,100),'设置无感相位偏置 rad',/Sensorless phase offset set/);
    if(mode==='slrun'&&start){const transition=options.threshold?.[0]??device.sensorless_transition_rpm,fallback=options.threshold?.[1]??device.sensorless_fallback_rpm;number(transition,'接管转速（请回读或勾选填写）',1,100000);number(fallback,'回退转速（请回读或勾选填写）',0,100000);if(Number(fallback)>=Number(transition))throw Error('回退转速必须小于接管转速');if(Math.abs(rpm)<Number(transition))throw Error('目标低于接管转速，无法进入无感接管；可选择只观察');const model=options.model??[device.sensorless_motor_r,device.sensorless_motor_l==null?null:device.sensorless_motor_l*1000,device.sensorless_flux_linkage==null?null:device.sensorless_flux_linkage*1000];model.forEach(v=>number(v,'电机 R/L/磁链（请回读或勾选填写）',1e-9,100000));}
    add('slenable 1','启用无感观测器',/Sensorless observer enabled/);
    if(mode==='slrun'){add('olfb sensorless','选择无感反馈，由固件判断起步和接管条件',/feedback set to 1/);add('sltakeover 1','授权无感接管；不代表已经锁定或接管',/Sensorless takeover armed/);if(speedLoop)add('spenable 1','启用无感速度环，等待有效反馈',/Speed loop enabled/);}
  }
  add('olspeed '+rpm,'设置目标 '+rpm+' rpm（此步不启动）',/Target RPM set to/);
  if(start)add('olstart','启动电机：按上述参数运行',/Open-loop started/);
  add('olstatus','查询最终状态，核对实际结果',/Open-loop|Control|Target|Mode|RPM/i);
  return steps;
}
export function crc16(bytes){let crc=65535;for(const byte of bytes){crc^=byte<<8;for(let bit=0;bit<8;bit++)crc=((crc&32768)?(crc<<1)^0x1021:crc<<1)&65535;}return crc;}
export function encodeTarget(token,sequence,target){
  if(!Number.isInteger(token)||token<=0||token>0xffffffff||!Number.isInteger(sequence)||sequence<0||sequence>0xffffffff)throw Error('控制会话或序号无效');
  const values=[target.rpm,target.iq,target.id];if(values.some(v=>typeof v!=='number'||!Number.isFinite(v)))throw Error('目标必须是有限数值');
  const body=`@C1,${token},${sequence},${values.join(',')}`;
  return new TextEncoder().encode(body+'*'+crc16(new TextEncoder().encode(body)).toString(16).padStart(4,'0').toUpperCase()+'\r');
}
export function encodePacket(type,sequence,payload){const bytes=new Uint8Array(payload.length+10),v=new DataView(bytes.buffer);bytes.set([70,83,1,type]);v.setUint16(4,payload.length,true);v.setUint16(6,sequence,true);bytes.set(payload,8);v.setUint16(bytes.length-2,crc16(bytes.slice(0,-2)),true);return bytes;}
export class StudioDecoder{
  constructor(onPacket,onTextBytes){this.onPacket=onPacket;this.onTextBytes=onTextBytes;this.buffer=new Uint8Array();this.badCrc=0;}
  push(bytes){const merged=new Uint8Array(this.buffer.length+bytes.length);merged.set(this.buffer);merged.set(bytes,this.buffer.length);this.buffer=merged;
    for(;;){let start=-1;for(let i=0;i<this.buffer.length-1;i++)if(this.buffer[i]===70&&this.buffer[i+1]===83){start=i;break;}
      if(start<0){const keep=this.buffer.at(-1)===70?1:0;if(this.buffer.length>keep)this.onTextBytes(this.buffer.slice(0,this.buffer.length-keep));this.buffer=this.buffer.slice(this.buffer.length-keep);return;}
      if(start){this.onTextBytes(this.buffer.slice(0,start));this.buffer=this.buffer.slice(start);}
      if(this.buffer.length<8)return;const v=new DataView(this.buffer.buffer,this.buffer.byteOffset,this.buffer.length),length=v.getUint16(4,true),type=this.buffer[3];
      if(this.buffer[2]!==1||type<1||type>4||length>86){this.onTextBytes(this.buffer.slice(0,1));this.buffer=this.buffer.slice(1);continue;}
      if(this.buffer.length<length+10)return;
      const packet=this.buffer.slice(0,length+10),pv=new DataView(packet.buffer);
      if(crc16(packet.slice(0,-2))!==pv.getUint16(packet.length-2,true)){this.badCrc++;this.buffer=this.buffer.slice(1);continue;}
      this.buffer=this.buffer.slice(length+10);this.onPacket({type,sequence:pv.getUint16(6,true),payload:packet.slice(8,-2)});
    }
  }
}
export function decodeTelemetry(payload){
  if(payload.length!==72)throw Error('遥测长度不匹配');const v=new DataView(payload.buffer,payload.byteOffset,payload.length);
  const result={timeUs:v.getUint32(0,true),token:v.getUint32(4,true),appliedSequence:v.getUint32(8,true),flags:v.getUint32(12,true),fault:v.getInt32(16,true),result:v.getInt32(20,true)};
  const keys=['targetRpm','rpm','rampRpm','ia','ib','ic','id','iq','vbus','theta','idTarget','iqTarget'];keys.forEach((key,i)=>{result[key]=v.getFloat32(24+i*4,true);if(!Number.isFinite(result[key]))throw Error('遥测包含非有限数据');});
  result.enabled=!!(result.flags&1);result.speedEnabled=!!(result.flags&2);result.faultLatched=!!(result.flags&4);result.speedValid=!!(result.flags&8);result.owned=!!(result.flags&16);result.expired=!!(result.flags&32);result.mode=(result.flags>>8)&255;result.feedback=(result.flags>>16)&255;return result;
}

// 每次 tick 只尝试写最新目标。写入未结束时丢弃本次机会，不排队、不补发历史目标。
export class TargetPump{
  constructor({write,now=()=>performance.now(),onStop=()=>{},onWrite=()=>{}}){this.write=write;this.now=now;this.onStop=onStop;this.onWrite=onWrite;this.active=false;this.busy=false;this.sequence=0;this.generation=0;this.lastRx=0;this.sent=0;this.skipped=0;}
  start(token,target){if(this.busy)throw Error('上一条写入尚未结束');encodeTarget(token,1,target);this.generation++;this.token=token;this.target={...target};this.sequence=0;this.lastRx=this.now();this.active=true;this.sent=0;this.skipped=0;}
  setTarget(target){encodeTarget(this.token,1,target);this.target={...target};}
  observe(telemetry){if(!this.active)return;if(telemetry.token!==this.token||telemetry.expired||!telemetry.owned||telemetry.faultLatched||telemetry.result<0){this.stop('设备已撤销周期控制或拒绝目标');return;}this.lastRx=this.now();}
  stop(reason='周期控制已停止'){const was=this.active;this.active=false;this.generation++;if(was)this.onStop(reason);}
  async tick(){if(!this.active)return;if(this.now()-this.lastRx>150){this.stop('超过 150 ms 未收到有效遥测，已暂停下发');return;}if(this.busy){this.skipped++;return;}
    const generation=this.generation;const sequence=(this.sequence+1)>>>0;const bytes=encodeTarget(this.token,sequence,this.target);this.busy=true;
    try{await this.write(bytes);if(this.active&&generation===this.generation){this.sequence=sequence;this.sent++;this.onWrite(sequence);}}
    catch(error){if(generation===this.generation)this.stop('串口写入失败：'+error.message);}
    finally{this.busy=false;}
  }
}
