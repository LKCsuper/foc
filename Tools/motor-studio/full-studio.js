import {TargetPump,decodeTelemetry,STUDIO_PERIOD_MS,buildOpenLoopRecipe,startAvailability,scopeRange,buildModeRecipe,detectionCommand,parseDetectionResult,detectionCompletion} from './studio-link.mjs';

export function mount(api){
  const {q,el}=api;
  mountMonitorFeed(api);
  document.body.classList.add('control-page');
  const monitorLink=el('a',{href:'http://127.0.0.1:4175/',target:'_blank',rel:'noopener',class:'monitor-open'},'打开独立波形页 ↗');q('#scope-connection').before(monitorLink);
  let ready=false,lastRx=0,lastTelemetry=null,wasConnected=false,pendingArm=0,armAt=0,freeze=false,ending=false,receivedParams=new Set(),lastPacket=null,lost=0;
  const scopes=mountParallelScopes(api);
  const panel=el('section',{class:'realtime-controls','aria-label':'20 ms 周期控制'});
  panel.innerHTML='<div class="realtime-title"><strong>周期目标 · 20 ms</strong><span id="studio-state">未连接 · 兼容手动命令</span><span id="studio-health">50 Hz 遥测 / 显式启动</span></div><div class="realtime-inputs"><label>目标转速 <input id="live-rpm" type="number" step="1" value="0" aria-label="周期目标转速"> rpm</label><label>Iq <input id="live-iq" type="number" step="0.01" value="0.3" aria-label="周期目标 Iq"> A</label><label>Id <input id="live-id" type="number" step="0.01" value="0" aria-label="周期目标 Id"> A</label><button id="live-arm">启用周期目标</button><button id="live-start" class="primary">启动电机</button><button id="live-stop" class="stop">停止并退出周期</button><button id="live-params">回读参数</button></div>';
  q('#live-monitor .cards').after(panel);
  const startHint=el('div',{class:'studio-note',id:'start-hint','aria-live':'polite'});panel.append(startHint);
  const curveBar=q('.stream-actions');curveBar.replaceChildren();
  for(const [name,label]of [['current','三相电流'],['speed','速度'],['dq','d / q'],['sine','电角度']]){
    const b=el('button',{'data-live-curve':name},label);b.onclick=()=>scopes.focus(name);curveBar.append(b);
  }
  curveBar.firstChild.classList.add('primary');
  const copy=el('button',{},'设备值复制为草稿');copy.onclick=()=>{if(!ready)return api.notify('设备尚未连接新版协议');api.copyDeviceDraft();api.notify('仅更新本地草稿，未发送或保存参数');};q('#draft-export').before(copy);
  const paramSubtitle=q('#parameter-view .studio-header .studio-note');paramSubtitle.textContent='支持新版固件时可回读 43 项配置和 20 项校准标量。草稿与设备回读值分开；每次修改先预览，再单独发送。';
  const controlPump=new TargetPump({write:bytes=>api.write(bytes),onStop:reason=>{api.appendLog('CONTROL',reason);q('#studio-state').textContent=reason;if(!ending)releaseSoon();},onWrite:sequence=>q('#studio-health').textContent=`目标帧 ${sequence} · 已确认 ${lastTelemetry?.appliedSequence??0} · 丢弃机会 ${controlPump.skipped}`});
  mountRecipeBuilder(api,()=>controlPump.active||pendingArm||lastTelemetry?.owned);
  function setReady(value){ready=value;api.setReady(value);q('#live-monitor').classList.toggle('structured-scopes',value);if(!value){q('#studio-state').textContent='未确认新版协议 · 手动命令仍可使用';}updateButtons();}
  function startState(){return startAvailability({...api.getState(),ready,pending:pendingArm,active:controlPump.active,telemetry:lastTelemetry,age:performance.now()-lastRx});}
  function updateButtons(){const start=startState();q('#live-arm').disabled=api.getState().batch||!ready||controlPump.active||!!pendingArm;q('#live-start').disabled=!start.enabled;q('#live-start').textContent=start.label;q('#live-start').title=start.reason;startHint.textContent=start.reason|| (controlPump.active?'使用已确认的周期目标启动':`将启动设备已配置的 ${lastTelemetry.targetRpm} rpm；上方输入框仅用于周期目标。`);q('#live-stop').disabled=!api.getState().connected;q('#live-params').disabled=!ready||api.getState().batch;}
  function target(){const values=['live-rpm','live-iq','live-id'].map(id=>{const raw=q('#'+id).value;if(raw.trim()==='')throw Error('周期目标不能留空');const value=Number(raw);if(!Number.isFinite(value))throw Error('周期目标必须是有限数值');return value;});const max=api.values.get('ol_max_rpm')?.value,limit=api.values.get('speed_iq_limit')?.value;
    if(max==null||limit==null)throw Error('请先回读转速及电流限制');if(Math.abs(values[0])>max)throw Error('目标超过设备最大转速 '+max+' rpm');if(values[1]**2+values[2]**2>limit**2)throw Error('Id/Iq 合成目标超过设备电流预算 '+limit+' A');return {rpm:values[0],iq:values[1],id:values[2]};}
  async function sendWhenIdle(line){for(let i=0;i<15&&api.getState().sending;i++)await new Promise(r=>setTimeout(r,5));if(api.getState().sending){api.notify('串口仍忙，设备将按租约超时停机');return;}await api.sendLine(line);}
  function releaseSoon(){if(api.getState().connected&&api.getState().transport==='serial')void sendWhenIdle('studio release');}
  for(const id of ['live-rpm','live-iq','live-id'])q('#'+id).oninput=()=>{if(controlPump.active){try{controlPump.setTarget(target());q('#studio-state').textContent='周期目标运行中 · 输入已校验';}catch(error){q('#studio-state').textContent='输入未应用：'+error.message;}}};
  q('#live-arm').onclick=async()=>{
    try{if(!ready||performance.now()-lastRx>150)throw Error('请等待新鲜的设备状态');if(document.hidden)throw Error('请保持网页在前台');if(lastTelemetry.enabled)throw Error('启用周期目标前请先停机');if(lastTelemetry.mode!==1)throw Error('请先通过 olmode 1 切换 I/F 模式');if(lastTelemetry.faultLatched)throw Error('请先排除并清除故障');target();pendingArm=crypto.getRandomValues(new Uint32Array(1))[0]||1;armAt=performance.now();updateButtons();await sendWhenIdle('studio control '+pendingArm);q('#studio-state').textContent='等待设备授权周期会话，不会自动启动';}
    catch(error){pendingArm=0;api.notify(error.message);updateButtons();}
  };
  q('#live-start').onclick=async()=>{freeze=true;try{let start=startState();if(!start.enabled)throw Error(start.reason);if(controlPump.active){const started=performance.now();while(lastTelemetry?.appliedSequence!==controlPump.sequence&&performance.now()-started<120)await new Promise(r=>setTimeout(r,5));if(lastTelemetry?.appliedSequence!==controlPump.sequence)throw Error('设备尚未确认最新目标，请重试');}start=startState();if(!start.enabled)throw Error(start.reason);await sendWhenIdle(start.line);}catch(error){api.notify(error.message);}finally{freeze=false;updateButtons();}};
  q('#live-stop').onclick=async()=>{if(api.getState().batch){if(!api.cancelDetection?.()&&!api.cancelUnified?.())q('#recipe-cancel').click();return;}ending=true;try{controlPump.stop('用户停止周期控制');pendingArm=0;await sendWhenIdle(ready?'studio release':'olstop');}finally{ending=false;updateButtons();}};
  q('#live-params').onclick=()=>sendWhenIdle('studio params');
  function resetConnection(){controlPump.stop('串口连接已结束');pendingArm=0;ready=false;lastTelemetry=null;lastRx=0;lastPacket=null;lost=0;receivedParams.clear();setReady(false);for(const cell of document.querySelectorAll('.device-readback'))if(cell.textContent.includes('已回读'))cell.textContent=cell.textContent.replace('已回读','离线缓存');}
  api.onState(state=>{
    if(!state.connected){if(wasConnected)resetConnection();wasConnected=false;return;}
    if(state.connected&&!wasConnected){wasConnected=true;lastTelemetry=null;ready=false;lastRx=0;receivedParams.clear();q('#studio-state').textContent='探测新版接口中…';const connectedAt=Date.now();setTimeout(()=>{if(api.getState().connected)void sendWhenIdle('studio on');},30);
      setTimeout(()=>{if(api.getState().connected&&!ready&&Date.now()-connectedAt>=1900){q('#studio-state').textContent='未收到新版接口；可用手动命令，升级固件后启用周期控制';api.appendLog('LINK','studio 握手未完成：旧固件或 legacy plot 正在运行。');}},2000);
    }
  });
  function setTelemetry(t){lastTelemetry=t;lastRx=performance.now();controlPump.observe(t);q('#rpm').textContent=t.speedValid?Math.round(t.rpm).toLocaleString():'—';q('#iq').textContent=t.iq.toFixed(2);q('#id').textContent=t.id.toFixed(2);q('#targetLabel').textContent=Math.round(t.targetRpm).toLocaleString();q('.cards .card:nth-child(3) .metric').textContent=t.vbus.toFixed(2)+' V';q('.cards .card:nth-child(3) .card-foot').textContent='设备回读 · DC BUS';q('#tracking').textContent=t.speedValid?'反馈有效':'转速反馈无效';q('#status').textContent=t.faultLatched?'故障锁存':t.enabled?'运行中':'已停机';q('#statusFoot').textContent=`故障 ${t.fault} · ${t.mode===1?'I/F':'V/F'} · ${t.speedEnabled?'速度环':'电流/开环'}`;
    const fields={ia:t.ia,ib:t.ib,ic:t.ic,Id:t.id,Iq:t.iq,v_bus:t.vbus,theta:t.theta,ol_current_rpm:t.rampRpm,id_target:t.idTarget,iq_target:t.iqTarget,ol_target_rpm:t.targetRpm};for(const [name,value]of Object.entries(fields))api.readback(name,value);
    scopes.push(t);api.publishTelemetry(t);if(!controlPump.active&&!pendingArm)q('#studio-state').textContent=t.expired?'设备控制超时已停机，需重新启用':`只读监视 · ${t.enabled?'电机运行中':'电机已停机'}`;updateButtons();
  }
  api.onPacket(packet=>{
    if(api.getState().transport!=='serial')return;
    const view=new DataView(packet.payload.buffer,packet.payload.byteOffset,packet.payload.length);
    try{
      if(lastPacket!=null){const difference=(packet.sequence-lastPacket+65536)%65536;if(difference===0||difference>=32768)return;if(difference>1)lost+=difference-1;}lastPacket=packet.sequence;
      if(packet.type===2&&packet.payload.length===20){
        const period=view.getUint32(0,true),timeout=view.getUint32(4,true),token=view.getUint32(8,true),count=view.getUint32(12,true),model=view.getUint32(16,true);
        if(period!==20||timeout!==300||count!==63)throw Error('设备接口版本/字段表不匹配');
        const first=!ready;setReady(true);q('#transport-status').textContent=`新版接口已确认：${period} ms 状态 / ${timeout} ms 控制超时；电机档案 ID ${model}。连接不会自动启动。`;
        if(pendingArm&&token===pendingArm){controlPump.start(token,target());pendingArm=0;void controlPump.tick();q('#studio-state').textContent='20 ms 周期目标已启用，电机尚未自动启动';}
        if(first)setTimeout(()=>{if(ready)void sendWhenIdle('studio params');},30);
      }else if(packet.type===1){setTelemetry(decodeTelemetry(packet.payload));}
      else if(packet.type===3&&packet.payload.length===8){
        const id=view.getUint16(0,true),total=view.getUint16(2,true),value=view.getFloat32(4,true);if(total!==63||id>=63||!Number.isFinite(value))throw Error('设备参数帧无效');receivedParams.add(id);
        if(id<43)api.readback(api.catalog.config[id].name,value);
        else{const groups=[['current_offset',43,3],['voltage_offset',46,3],['undriven_offset',49,3],['hall_table',52,8]];const group=groups.find(([,start,length])=>id>=start&&id<start+length);
          if(group){const existing=api.values.get(group[0])?.value||Array(group[2]).fill(null);const copy=[...existing];copy[id-group[1]]=value;api.readback(group[0],copy);}else api.readback(['encoder_offset_rad','encoder_inverted','encoder_calibrated'][id-60],value);}
        q('#live-params').textContent=`回读参数 ${receivedParams.size}/63`;
      }else if(packet.type===4&&packet.payload.length===8){
        const id=view.getUint16(0,true),code=view.getInt16(2,true),value=view.getFloat32(4,true);api.appendLog(code===0?'ACK':'REJECT',id===65535?`控制请求结果 ${code}`:`字段 ${id} 结果 ${code}，设备值 ${value}`);
        if(id<43&&Number.isFinite(value))api.readback(api.catalog.config[id].name,value);
        if(code<0){pendingArm=0;api.notify(`设备拒绝请求（${code}）：请核对停机、模式、故障与参数范围`);updateButtons();}
        else if(id<43)api.notify('设备已确认参数应用；尚未保存 Flash');
      }
    }catch(error){api.appendLog('PROTOCOL',error.message);controlPump.stop('协议数据异常，周期控制已停止');if(packet.type===2)setReady(false);}
  });
  setInterval(()=>{
    if(controlPump.active&&!freeze){if(api.getState().sending)controlPump.skipped++;else void controlPump.tick();}
    if(pendingArm&&performance.now()-armAt>1500){pendingArm=0;api.notify('设备未确认周期控制，未开始下发');updateButtons();}
  },STUDIO_PERIOD_MS);
  setInterval(()=>{
    if(ready&&lastRx&&performance.now()-lastRx>150){q('#status').textContent='状态数据已过期';q('#studio-state').textContent='遥测已过期 · '+Math.round(performance.now()-lastRx)+' ms';controlPump.stop('遥测已过期，暂停目标下发');}
    q('#scope-source').textContent=api.getState().transport==='demo'?'演示信号 · 始终可见':ready?`设备帧龄 ${lastRx?Math.round(performance.now()-lastRx):'—'} ms · CRC 丢弃 ${api.badCrc()} · 序号缺口 ${lost}`:'串口未确认新版接口';updateButtons();
  },100);
  document.addEventListener('visibilitychange',()=>{if(document.hidden){pendingArm=0;controlPump.stop('页面进入后台，周期目标已暂停；返回后需重新启用');}});
  window.addEventListener('pagehide',()=>controlPump.stop('页面已关闭'));
  setReady(false);
  mountConvenience(api);
  const advanced=el('details',{class:'advanced-periodic'});advanced.append(el('summary',{},'高级：20 ms 周期目标控制'));panel.before(advanced);advanced.append(panel);
  mountUnifiedControl(api,()=>({ready,telemetry:lastTelemetry,age:performance.now()-lastRx,owned:controlPump.active||pendingArm||lastTelemetry?.owned}),advanced);
  mountDetectionPage(api,()=>({ready,telemetry:lastTelemetry,age:performance.now()-lastRx,owned:controlPump.active||pendingArm||lastTelemetry?.owned}));
}

/* 四张图共享同一遥测帧和时间窗口；暂停仅冻结显示，接收及控制继续。 */
function mountDetectionPage(api,context){
  const {q,el}=api,view=q('#detection-view');let running=false,log='',result=null,timeout,startedAt=0;
  view.innerHTML='<div class="studio-header"><div><h1>电机参数检测</h1><div class="studio-note">独立检测工作区 · 检测结果与设备配置分开 · 不自动应用或保存模型</div></div><button id="detect-export">导出检测记录</button></div><section class="panel detection-panel"><h2>① 选择检测项目</h2><div class="detection-kinds"><label><input type="radio" name="detect-kind" value="rl" checked> 电阻 / 电感 R/L</label><label><input type="radio" name="detect-kind" value="motor"> R/L + 磁链</label><label><input type="radio" name="detect-kind" value="fluxscan"> 分档磁链扫描</label></div><p id="detect-effect" class="workbench-warning"></p><div class="detection-inputs"><label>R/L 测试电流 A <input id="detect-current" class="studio-input" type="number" step="any" placeholder="留空沿用固件默认"></label><label id="detect-erpm-label">电转速 eRPM <input id="detect-erpm" class="studio-input" type="number" step="1" placeholder="电转速，不是机械转速"></label><label id="detect-flux-label">磁链测试电流 A <input id="detect-flux-current" class="studio-input" type="number" step="any" placeholder="留空沿用固件默认"></label></div><h2>② 检查条件</h2><p id="detect-conditions" class="studio-note"></p><label class="detection-ack"><input type="checkbox" id="detect-ack"> 已确认接线、供电和测试电流，转子有检测所需的运动空间</label><div class="cmd-preview" id="detect-preview"></div><div class="detection-actions"><button id="detect-run" class="primary">开始检测</button><button id="detect-abort" class="stop">中止检测并停机</button><button id="detect-read">读取最近结果</button><span id="detect-status" role="status">尚未检测</span></div></section><section class="panel detection-panel"><h2>③ 检测结果</h2><div class="detection-results"><div><small>电阻 R / Ω</small><strong id="detected-r">—</strong></div><div><small>电感 L / μH</small><strong id="detected-l">—</strong></div><div><small>磁链 / mWb</small><strong id="detected-flux">—</strong></div></div><p id="detect-result-note" class="studio-note">尚无检测回复。检测不会自动更新参数草稿、设备模型或 Flash。</p><div class="detection-actions"><button id="detect-to-model">将有效结果填入无感模型草稿</button></div><h3>原始检测输出</h3><pre id="detect-output" class="detection-output">等待检测或读取结果…</pre></section><section class="panel detection-panel"><h2>传感器与采样校准入口</h2><p class="studio-note">以下按钮只打开对应命令说明并填入命令，不自动执行。偏移和编码器默认选 RAM 版本；霍尔标定的保存行为由固件决定。</p><div id="detection-calibration" class="detection-actions"></div></section>';
  q('#detection-calibration').closest('section').querySelector('h2').textContent='传感器与采样校准 · 页内执行';
  q('#detection-calibration').previousElementSibling.textContent='选择项目后，直接使用本页的开始、停止和结果区，不跳转。偏移与编码器使用 RAM 版本；霍尔标定可能保存校准，请查看执行说明。';
  for(const [title,key]of [['采样偏移校准','offset'],['编码器零点/方向','encoder'],['霍尔表标定','hall'],['UVW 相序检查','phase']]){const label=el('label',{class:'calibration-choice'});label.append(el('input',{type:'radio',name:'detect-kind',value:key}),document.createTextNode(' '+title));q('#detection-calibration').append(label);}
  const calibrationSection=q('#detection-calibration').closest('section');calibrationSection.className='inline-calibration';q('.detection-kinds').after(calibrationSection);
  const calibrationValues=el('pre',{class:'calibration-values'});q('.detection-results').after(calibrationValues);
  const kind=()=>q('input[name="detect-kind"]:checked').value;
  function command(){return detectionCommand(kind(),['offset','encoder','hall','phase'].includes(kind())?[]:kind()==='rl'?[q('#detect-current').value]:[q('#detect-current').value,q('#detect-erpm').value,q('#detect-flux-current').value]);}
  function finish(message){running=false;clearTimeout(timeout);api.setBatch(false);q('#detect-status').textContent=message;render();}
  function render(){const k=kind(),s=api.getState(),c=context();q('.detection-inputs').hidden=['offset','encoder','hall','phase'].includes(k);q('#detect-erpm-label').hidden=q('#detect-flux-label').hidden=k==='rl';q('#detect-effect').textContent=k==='rl'?'会给电机通电、对齐转子并施加双极性脉冲；可能发生转动。结果只在 RAM，不自动应用模型。':k==='motor'?'会先测 R/L，再开环旋转、关桥滑行估算磁链。eRPM 是电转速；结果只在 RAM。':'会旋转并分档扫描磁链，最大电转速须在 900～3000 eRPM；拟合质量以原始报告为准。';const effects={offset:'采样偏移校准：停机且不应有相电流，结果应用到 RAM，不请求写 Flash。',encoder:'编码器零点/方向标定：会通电、对齐并前后转动一机械圈，应用 RAM，旧齿槽补偿表会清除；需已有合适 Iq（不超过 0.5 A）。固件为同步流程，串口停止请求可能需流程返回后处理。',hall:'霍尔表标定：会通电旋转，校准结果可能保存到 Flash，请确认霍尔接线和运动空间。',phase:'UVW 相序检查：会通电并检查相序与霍尔步进对应关系，请确认霍尔接线。'};if(effects[k])q('#detect-effect').textContent=effects[k];let error='';try{q('#detect-preview').textContent=command();}catch(e){error=e.message;q('#detect-preview').textContent=error;}
    if(s.transport==='serial'){if(!s.connected)error='请先连接串口';else if(!c.ready||!c.telemetry||c.age>150)error='等待设备握手和新鲜状态';else if(c.telemetry.enabled)error='请先停机，再进行参数检测';else if(c.telemetry.faultLatched)error='故障锁存，请先排查';else if(c.owned)error='请先退出周期控制会话';else if(s.batch&&!running)error='其他组合正在执行';}
    q('#detect-conditions').textContent=s.transport==='demo'?'演示预览，未连接真实设备；不会执行检测。':running?'检测进行中 · '+Math.floor((Date.now()-startedAt)/1000)+' 秒 · 等待固件完成回复':error||'条件已就绪；仍需核对电源、接线、台架及测试电流。';q('#detect-run').textContent=s.transport==='demo'?'记录检测预览（不执行）':'开始检测';q('#detect-run').disabled=running||!!error||s.transport==='serial'&&!q('#detect-ack').checked;q('#detect-abort').disabled=!s.connected||s.batch&&!running;q('#detect-read').disabled=running||s.batch;q('#detect-to-model').disabled=running||![result?.resistance,result?.inductance,result?.flux].every(v=>v?.valid);for(const n of view.querySelectorAll('.detection-inputs input,input[name=detect-kind],#detect-ack'))n.disabled=running;
  }
  api.onText(text=>{log=(log+text.replace(/\x1b\[[0-9;]*[a-zA-Z]/g,'')).slice(-40000);if(!running&&!/Motor Parameter Detect/.test(log))return;q('#detect-output').textContent=log;const parsed=parseDetectionResult(log);if(parsed){result=parsed;for(const [id,value]of [['detected-r',result.resistance],['detected-l',result.inductance],['detected-flux',result.flux]])q('#'+id).textContent=value?.valid?String(value.value):value?'无效':'—';q('#detect-result-note').textContent='设备检测状态：'+result.status+'。有效值未自动应用，模型和 Flash 未由此页面修改。';}
    if(running){const done=detectionCompletion(kind(),log);if(done!==null){if(['offset','encoder','hall','phase'].includes(kind()))q('#detect-result-note').textContent=done?'校准/检查完成，实际偏移、方向或霍尔表请核对原始回复。不会自动启动电机。':'校准/检查失败，请核对原始回复。';finish(done?'检测/校准完成，请核对原始报告':'检测/校准失败，请查看原始报告');}else if(/No active motor detection/i.test(log))finish('设备回复没有活动检测，请核对停机状态');else if(/Invalid detection arguments|Failed to create|already running|Unknown command|not found|Detect End:\s*FAILED/i.test(log))finish('设备拒绝或检测失败，请查看原始输出');}render();});
  q('#detect-run').onclick=async()=>{try{render();if(q('#detect-run').disabled)return;const line=command();if(api.getState().transport==='demo'){api.appendLog('PREVIEW',line+'（未发送）');q('#detect-status').textContent='已记录检测预览，未通电或执行';return;}log='';result=null;for(const id of ['detected-r','detected-l','detected-flux'])q('#'+id).textContent='—';running=true;startedAt=Date.now();api.setBatch(true);q('#detect-status').textContent='等待设备接受检测…';q('#detect-output').textContent='等待检测回复…';render();const sent=await api.sendLine(line,{batch:true});if(!sent)throw Error('检测命令未发送');if(running)timeout=setTimeout(()=>{q('#detect-status').textContent='检测超过 180 秒，正在请求中止';void abort();},180000);}catch(error){finish(error.message);}};
  async function abort(){try{if(!await api.sendLine('detect abort',{batch:true}))throw Error('中止命令未发送');if(!await api.sendLine('olstop',{batch:true}))throw Error('停机命令未发送');q('#detect-status').textContent='已发送中止和停机请求，等待检测结束；请核对设备状态';}catch(error){q('#detect-status').textContent='中止未确认：'+error.message;}if(!running)render();}
  api.cancelDetection=()=>{if(!running)return false;void abort();return true;};q('#detect-abort').onclick=()=>void abort();q('#detect-read').onclick=()=>{log='';result=null;void api.sendLine('detect show');};q('#detect-export').onclick=()=>api.download('motor-detection.json',JSON.stringify({at:new Date().toISOString(),result,log},null,2),'application/json');
  q('#detect-to-model').onclick=()=>{if(!result||q('#detect-to-model').disabled)return;for(const [field,value]of [['sensorless_motor_r',result.resistance.value],['sensorless_motor_l',result.inductance.value/1000],['sensorless_flux_linkage',result.flux.value]])q(`[data-sl-field="${field}"]`).value=value;q('[data-sl-use="model"]').checked=true;q('#recipe-mode').value='slrun';q('#recipe-start').checked=false;q('.recipe-builder').dispatchEvent(new Event('input',{bubbles:true}));api.navigate('commands');api.notify('只填入无感模型草稿，未应用设备、未启动或保存 Flash');};
  function renderCalibration(){const calibration=['offset','encoder','hall','phase'].includes(kind());q('.detection-results').hidden=calibration;calibrationValues.hidden=!calibration;q('#detect-to-model').hidden=calibration;q('#detect-read').textContent=calibration?'回读校准数据':'读取最近结果';if(calibration){const fields=kind()==='encoder'?['encoder_offset_rad','encoder_inverted','encoder_calibrated']:kind()==='offset'?['current_offset','voltage_offset','undriven_offset']:kind()==='hall'?['hall_table']:[];const names={encoder_offset_rad:'编码器偏移 rad',encoder_inverted:'方向反转',encoder_calibrated:'已标定',current_offset:'电流采样偏移',voltage_offset:'相电压偏移',undriven_offset:'滑行电压偏移',hall_table:'霍尔表'};calibrationValues.textContent=fields.length?fields.map(field=>{const entry=api.values.get(field);return names[field]+'：'+(entry?JSON.stringify(entry.value)+'（回读 '+new Date(entry.at).toLocaleTimeString()+'）':'尚未回读');}).join('\n'):'相序检查结果见下方原始输出。';}}
  q('#detect-read').onclick=()=>{if(['offset','encoder','hall','phase'].includes(kind())){void api.sendLine('studio params');}else{log='';result=null;void api.sendLine('detect show');}};
  view.addEventListener('input',()=>{render();renderCalibration();});view.addEventListener('change',event=>{if(event.target.name==='detect-kind'){q('#detect-ack').checked=false;q('#detect-result-note').textContent=['offset','encoder','hall','phase'].includes(kind())?'校准/检查结果以设备回复为准；校准可能更新实际校准数据，不等同于只读模型检测。':'检测模型结果仅RAM，不自动应用或保存。';}render();renderCalibration();});api.onState(s=>{if(running&&!s.connected)finish('串口已断开，检测和停机结果未知；重连后查询状态');render();renderCalibration();});setInterval(()=>{render();renderCalibration();},500);render();renderCalibration();
}

function mountUnifiedControl(api,context,before){
  const {q,el}=api;let running=false,cancelled=false,lastMode='',lastResult='';
  const bar=el('section',{class:'unified-control','aria-label':'统一电机控制'});
  bar.innerHTML='<div class="unified-title"><strong>电机控制</strong><span>设备当前档案 · PI、模型与保护参数沿用设备回读值</span></div><div class="unified-fields"><label>模式 <select id="motor-mode" class="studio-select"><option value="open">开环 I/F</option><option value="current">电流 / 转矩</option><option value="sensored">有感速度环</option><option value="sensorless">无感速度环</option><option value="position">位置环</option></select></label><label id="motor-feedback-label">反馈 <select id="motor-feedback" class="studio-select"><option value="encoder">编码器</option><option value="hall">霍尔</option></select></label><label id="motor-target-label"><span id="motor-target-name">目标转速</span> <input id="motor-target" class="studio-input" type="number" value="300" step="1"><span id="motor-unit">rpm</span></label><label id="motor-hold-label" hidden><input id="motor-hold" type="checkbox" checked>保持当前位置</label><button id="motor-run" class="primary">配置并启动</button><button id="motor-stop" class="stop">停止电机</button><button id="motor-prepare">仅预览步骤</button></div><div id="motor-mode-help" class="studio-note"></div><div id="motor-run-state" role="status">尚未执行</div><details><summary>查看将执行的配置</summary><ol id="motor-plan"></ol></details>';
  before.before(bar);
  const values=()=>Object.fromEntries([...api.values].map(([key,entry])=>[key,entry.value]));
  function plan(){return buildModeRecipe({mode:q('#motor-mode').value,target:q('#motor-target').value,feedback:q('#motor-feedback').value,hold:q('#motor-hold').checked,device:values()});}
  function render(){const mode=q('#motor-mode').value,t=context().telemetry;
    if(mode!==lastMode){q('#motor-target').value=mode==='current'?'0.3':mode==='position'?'0':mode==='sensorless'?String(Math.min(Number(api.values.get('ol_max_rpm')?.value)||800,Math.max(800,(Number(api.values.get('sensorless_transition_rpm')?.value)||500)*1.2))):'300';lastMode=mode;}
    q('#motor-feedback-label').hidden=!['current','sensored','position'].includes(mode);if(mode==='position')q('#motor-feedback').value='encoder';q('#motor-feedback').disabled=running||mode==='position';q('#motor-hold-label').hidden=mode!=='position';q('#motor-target').disabled=running||mode==='position'&&q('#motor-hold').checked;q('#motor-target-name').textContent=mode==='current'?'目标 Iq':mode==='position'?'目标角度':'目标转速';q('#motor-unit').textContent=mode==='current'?'A':mode==='position'?'°':'rpm';q('#motor-target').step=mode==='current'?'0.01':'1';
    q('#motor-mode-help').textContent={open:'I/F 开环角度，电流闭环；实际速度不保证等于目标。默认目标 300 rpm，其余沿用设备参数。',current:'正向 Iq 电流控制，Id=0；不保证转速或限速。现有固件用非零参考保持输出使能，转速参考不是速度目标。',sensored:'自动选择传感器并启用速度环；需要对应标定有效，参数沿用设备配置。',sensorless:'I/F 起步并授权无感接管，启用速度环；授权不等于已经锁定，启动后请查看状态。',position:'仅支持已标定编码器。默认保持启动时当前位置；取消保持后，角度相对已有位置零点，不自动清零。'}[mode];
    let reason='';try{plan();}catch(error){reason=error.message;}
    const state=api.getState(),c=context();if(!state.connected||state.transport!=='serial')reason='请连接真实串口并回读参数';else if(!c.ready||!c.telemetry)reason='等待设备握手与状态';else if(c.age>150)reason='等待新鲜设备状态';else if(c.telemetry.faultLatched)reason='设备故障锁存，请先检查';else if(c.owned)reason='请先退出高级周期目标会话';else if(state.batch&&!running)reason='其他组合正在执行';
    q('#motor-run').disabled=running||!!reason;q('#motor-run').title=reason;q('#motor-run').textContent=running?'执行中…':t?.enabled?'停机配置并重启':'配置并启动';q('#motor-stop').disabled=!state.connected;for(const id of ['motor-mode','motor-hold','motor-prepare'])q('#'+id).disabled=running;
    if(!running)q('#motor-run-state').textContent=reason||lastResult|| (t?.enabled?'设备正在运行；点击将先停机，再配置并启动':'就绪 · 点击后自动配置并启动');
  }
  function preview(){const list=q('#motor-plan');list.replaceChildren();try{for(const step of plan()){const li=el('li');li.append(el('span',{},step.title),el('code',{},step.line));list.append(li);}}catch(error){list.append(el('li',{},error.message));}}
  bar.addEventListener('change',()=>{lastResult='';render();preview();});bar.addEventListener('input',()=>{lastResult='';render();preview();});q('#motor-prepare').onclick=()=>{preview();bar.querySelector('details').open=true;};
  api.cancelUnified=()=>{if(!running)return false;cancelled=true;q('#motor-run-state').textContent='正在中止并请求停机…';void api.sendLine('olstop',{batch:true});return true;};
  q('#motor-stop').onclick=()=>{if(!api.cancelUnified())q('#live-stop').click();};
  q('#motor-run').onclick=async()=>{try{render();if(q('#motor-run').disabled)throw Error(q('#motor-run').title);const steps=plan();preview();running=true;cancelled=false;api.setBatch(true);render();for(let i=0;i<steps.length;i++){if(cancelled)throw Error('用户中止');const step=steps[i];q('#motor-run-state').textContent=`${i+1}/${steps.length} · ${step.title}`;if(step.line==='olstart'&&['current','sensored','position'].includes(q('#motor-mode').value)){const until=performance.now()+600;while(!context().telemetry?.speedValid&&performance.now()<until&&!cancelled)await new Promise(resolve=>setTimeout(resolve,10));const c=context();if(cancelled||!c.telemetry?.speedValid||c.age>150||c.telemetry.faultLatched)throw Error('反馈无效、过期或故障，未启动');}await api.exchangeShell(step.line,step.expected);}q('#motor-run-state').textContent='配置和启动命令已确认，请观察实际状态。';}
    catch(error){q('#motor-run-state').textContent='未完成：'+error.message;api.appendLog('MODE',error.message);if(running){try{await api.exchangeShell('olstop',/Open-loop stopped/);q('#motor-run-state').textContent+=' · 已请求停机';}catch{q('#motor-run-state').textContent+=' · 停机未确认';}}}
    finally{lastResult=q('#motor-run-state').textContent;running=false;api.setBatch(false);q('#motor-mode').disabled=false;q('#motor-hold').disabled=false;q('#motor-prepare').disabled=false;q('#motor-feedback').disabled=q('#motor-mode').value==='position';q('#motor-target').disabled=q('#motor-mode').value==='position'&&q('#motor-hold').checked;q('#motor-run').disabled=false;}};
  setInterval(()=>{if(!running)render();},500);render();preview();
}

function mountRecipeBuilder(api,ownsControl){
  const {q,el}=api;let running=false,cancelled=false;
  const box=el('section',{class:'recipe-builder panel','aria-label':'组合操作'});
  box.innerHTML='<div class="recipe-heading"><div><h2>组合操作 · 不用记命令顺序</h2><p class="studio-note">选场景、填目标、勾选参数，自动生成整套步骤。未勾选的参数沿用设备当前值。</p></div><button id="recipe-default">开环 300 rpm 模板</button></div><div class="recipe-fields"><label>运行方式 <select id="recipe-mode" class="studio-select"><option value="if">I/F 开环（电流闭环）</option><option value="vf">V/F 开环（电压驱动）</option></select></label><label>目标转速 <input id="recipe-rpm" type="number" value="300" step="1" class="studio-input"> rpm</label><button id="recipe-readback">填入已回读参数</button></div><p class="studio-note" id="recipe-mode-note"></p><div class="recipe-options"></div><label class="recipe-start-label"><input id="recipe-start" type="checkbox"> 最后启动电机（会转动；不勾选则只配置）</label><div class="recipe-heading"><strong>将按此顺序执行</strong><span id="recipe-summary" class="studio-note"></span></div><ol id="recipe-plan"></ol><div class="recipe-actions"><button id="recipe-send" class="primary">一键发送配置（不启动）</button><button id="recipe-cancel" disabled>中止组合并停机</button><span id="recipe-progress" role="status">尚未发送</span></div><p class="studio-note">真实串口逐条等待固件回复，失败即中止并请求停机；已经生效的参数不会自动回滚。不自动清故障、不写 Flash。开环目标是电角度推进速度，负载下不保证实际转速。</p>';
  q('#command-view .studio-header').after(box);
  q('#recipe-mode').append(el('option',{value:'slobserve'},'无感观察（不接管）'),el('option',{value:'slrun'},'无感接管运行'));
  const slPanel=el('div',{class:'recipe-sensorless'});slPanel.hidden=true;
  slPanel.innerHTML='<p class="workbench-warning">无感接管依赖电机模型和有效观测，低速或静止时可能无法锁定。300 rpm 低于设备接管门槛时，请先使用“只观察”。授权接管不等于已进入 CLOSED。未勾选的项沿用设备配置。</p><label><input type="checkbox" id="recipe-sl-speed"> 接管运行时启用速度闭环（需要有效反馈及合适的速度环参数）</label>';
  const slGroups=[
    ['model','电机模型','slmotor', [['电阻 Ω','sensorless_motor_r',1],['电感 mH','sensorless_motor_l',1000],['磁链 mWb','sensorless_flux_linkage',1000]]],
    ['threshold','接管 / 回退门槛','slrpm',[['接管 rpm','sensorless_transition_rpm',1],['回退 rpm','sensorless_fallback_rpm',1]]],
    ['observer','观测器与 PLL','slobs',[['观测器增益','sensorless_observer_gain',1],['PLL Kp','sensorless_pll_kp',1],['PLL Ki','sensorless_pll_ki',1]]],
    ['guard','接管护栏','slguard',[['开环上限 rpm','sensorless_openloop_max_rpm',1],['相位容差 rad','sensorless_phase_tolerance',1],['融合时间 s','sensorless_blend_time',1]]]
  ];
  for(const [key,label,cmd,fields]of slGroups){const group=el('div',{class:'recipe-option'}),toggle=el('label');toggle.append(el('input',{type:'checkbox','data-sl-use':key}),document.createTextNode(' '+label+' · '+cmd));group.append(toggle);for(const [title,field]of fields){const row=el('label',{},title+' ');row.append(el('input',{type:'number',step:'any',class:'studio-input','data-sl-field':field,'aria-label':title,placeholder:'回读或填写'}));group.append(row);}slPanel.append(group);}
  q('.recipe-start-label').before(slPanel);
  const definitions=[
    ['ramp','转速斜坡','rpm/s','10','ol_ramp_rpm_s','both','从当前速度向目标变化的快慢。'],
    ['max','最大转速','rpm','800','ol_max_rpm','both','限制目标范围，不替代失速保护。'],
    ['limit','电压上限','V','4','ol_vq_limit','both','限制输出电压幅值。'],
    ['iq','目标 Iq','A','0.3','iq_target','if','转矩电流目标，按电机与台架实际需要设置。'],
    ['id','目标 Id','A','0','id_target','if','d 轴电流目标。'],
    ['kp','电流环 Kp','','0.12','ol_current_kp','if','沿用设备参数时不要勾选。'],
    ['ki','电流环 Ki','','18','ol_current_ki','if','沿用设备参数时不要勾选。'],
    ['phase','无感相位偏置','rad','0','sensorless_phase_offset','sl','影响观测角度补偿，沿用设备值时不要勾选。'],
    ['spkp','速度环 Kp','A/rpm','0.003','speed_kp','sl','启用速度环时使用。'],
    ['spki','速度环 Ki','','0.003','speed_ki','sl','启用速度环时使用。'],
    ['spiq','速度环电流上限','A','0.5','speed_iq_limit','sl','限制速度环输出 Iq。'],
    ['boost','启动补偿','V','1','ol_v_boost','vf','低频时的补偿电压。'],
    ['kvf','V/F 斜率','V/Hz','0.057','ol_kv_f','vf','电频率增加时的电压增量。']
  ];
  for(const [key,label,unit,value,,mode,hint]of definitions){const item=el('label',{class:'recipe-option','data-recipe-mode':mode});item.innerHTML=`<span><input type="checkbox" data-recipe-use="${key}" ${['iq','id'].includes(key)?'checked':''}> ${label}</span><span><input type="number" class="studio-input" step="any" data-recipe-value="${key}" value="${value}" aria-label="组合${label}"> ${unit}</span><small>${hint}</small>`;box.querySelector('.recipe-options').append(item);}
  function plan(){const options={};for(const [key]of definitions)if(q(`[data-recipe-use="${key}"]`).checked)options[key]=q(`[data-recipe-value="${key}"]`).value;for(const [key,,,fields]of slGroups)if(q(`[data-sl-use="${key}"]`).checked)options[key]=fields.map(([,field])=>q(`[data-sl-field="${field}"]`).value);return buildOpenLoopRecipe({mode:q('#recipe-mode').value,rpm:q('#recipe-rpm').value,options,start:q('#recipe-start').checked,speedLoop:q('#recipe-sl-speed').checked,device:Object.fromEntries([...api.values].map(([key,entry])=>[key,entry.value]))});}
  function render(){const mode=q('#recipe-mode').value,sl=mode.startsWith('sl');slPanel.hidden=!sl;q('#recipe-sl-speed').disabled=mode!=='slrun'||running;for(const item of box.querySelectorAll('[data-recipe-mode]')){const type=item.dataset.recipeMode;item.hidden=!(type==='both'||type===mode||type==='if'&&sl||type==='sl'&&sl);}q('#recipe-mode-note').textContent={if:'I/F：关闭速度环及无感接管，使用开环角度；可选 Iq、Id 和电流环参数。',vf:'V/F：关闭速度环及无感接管，使用开环角度；可选启动补偿、V/F 斜率和电压上限。',slobserve:'无感只观察：I/F 开环起步，启用观测器但禁止接管，速度环关闭。用于比较观测与实际状态。',slrun:'无感接管运行：I/F 起步 → 启用观测器 → 选择无感反馈 → 授权接管；可选速度环。启动前须回读或填写模型和接管门槛，目标不得低于接管转速。'}[mode];
    const list=q('#recipe-plan');list.replaceChildren();try{const steps=plan();steps.forEach(step=>{const li=el('li');li.append(el('span',{},step.title),el('code',{},step.line));list.append(li);});q('#recipe-summary').textContent=steps.length+' 步 · '+(q('#recipe-start').checked?'包含电机启动':'仅配置，不启动');q('#recipe-send').disabled=running;q('#recipe-send').textContent=api.getState().transport==='demo'?'预览整套组合（不发送）':q('#recipe-start').checked?'一键配置并启动电机':'一键发送配置（不启动）';}catch(error){q('#recipe-summary').textContent=error.message;q('#recipe-send').disabled=true;}
  }
  box.addEventListener('input',render);box.addEventListener('change',render);api.onState(render);
  q('#recipe-default').onclick=()=>{q('#recipe-mode').value='if';q('#recipe-rpm').value='300';q('#recipe-start').checked=false;for(const [key,,,value]of definitions){q(`[data-recipe-use="${key}"]`).checked=['iq','id'].includes(key);q(`[data-recipe-value="${key}"]`).value=value;}render();};
  q('#recipe-readback').onclick=()=>{let count=0;for(const [key,,,,field]of definitions){const actual=api.values.get(field);if(actual&&Number.isFinite(actual.value)){q(`[data-recipe-value="${key}"]`).value=actual.value;count++;}}for(const [,,,fields]of slGroups)for(const [,field,scale]of fields){const actual=api.values.get(field);if(actual&&Number.isFinite(actual.value)){q(`[data-sl-field="${field}"]`).value=actual.value*scale;count++;}}api.notify(count?'已填入 '+count+' 项缓存回读值，请核对时间与适用模式；尚未发送':'尚无设备回读值');render();};
  function controls(disabled){for(const input of box.querySelectorAll('input,select,button'))input.disabled=disabled;q('#recipe-cancel').disabled=!disabled;}
  q('#recipe-cancel').onclick=()=>{cancelled=true;q('#recipe-progress').textContent='正在中止并请求停机…';void api.sendLine('olstop',{batch:true});};
  q('#recipe-send').onclick=async()=>{
    let steps;try{steps=plan();if(api.getState().transport==='demo'){for(const step of steps)api.appendLog('PLAN',step.line+' · '+step.title+'（未发送）');q('#recipe-progress').textContent='整套组合已记录为预览，未发送设备';return;}if(!api.getState().connected)throw Error('请先连接串口');if(ownsControl())throw Error('请先点击上方“停止并退出周期”，再发送组合');if(api.getState().sending)throw Error('串口正在发送，请稍后重试');
      running=true;cancelled=false;api.setBatch(true);controls(true);
      for(let i=0;i<steps.length;i++){if(cancelled)throw Error('用户中止');const step=steps[i];q('#recipe-progress').textContent=`${i+1}/${steps.length} · ${step.title}`;const li=q('#recipe-plan').children[i];li.classList.add('executing');await api.exchangeShell(step.line,step.expected);li.classList.remove('executing');li.classList.add('completed');}
      if(cancelled)throw Error('用户中止');q('#recipe-progress').textContent=q('#recipe-start').checked?'组合已执行，请观察设备状态和曲线':'配置组合已执行，电机保持停机';
    }catch(error){api.appendLog('PLAN-ERR',error.message);q('#recipe-progress').textContent='已中止：'+error.message;if(running){try{await api.exchangeShell('olstop',/Open-loop stopped/);q('#recipe-progress').textContent+=' · 设备已回复停机';}catch{q('#recipe-progress').textContent+=' · 停机未获确认，请检查设备';}}else api.notify(error.message);}
    finally{running=false;api.setBatch(false);controls(false);q('#recipe-cancel').disabled=true;render();}
  };
  render();
}

export function mountParallelScopes(api){
  const {q,el}=api,colors=['#5be2b3','#66a8ff','#f2b86b','#d294ff'];
  const definitions=[
    {key:'current',title:'三相电流',unit:'A',labels:['Ia','Ib','Ic'],values:t=>[t.ia,t.ib,t.ic]},
    {key:'speed',title:'转速',unit:'rpm',labels:['目标','反馈','斜坡'],values:t=>[t.targetRpm,t.speedValid?t.rpm:null,t.rampRpm]},
    {key:'dq',title:'d / q 电流',unit:'A',labels:['Id','Iq','Id 目标','Iq 目标'],values:t=>[t.id,t.iq,t.idTarget,t.iqTarget]},
    {key:'sine',title:'电角度正余弦',unit:'归一化',labels:['sin θ','cos θ'],values:t=>[Math.sin(t.theta),Math.cos(t.theta)]}
  ];
  let samples=[],frozen=null,lastDemo=0,lastPaint=0,source=api.getState().transport,windowMs=2000;
  const section=el('section',{class:'parallel-scopes','aria-label':'四组同步波形'});
  const toolbar=el('div',{class:'parallel-toolbar'}),status=el('span',{class:'studio-note'},'同一时间窗口 · 四图同步');
  const select=el('select',{'aria-label':'同步波形时间窗口',class:'studio-select'});
  for(const [value,label]of [[1000,'最近 1 秒'],[2000,'最近 2 秒'],[5000,'最近 5 秒'],[10000,'最近 10 秒']]){const option=el('option',{value:String(value)},label);option.selected=value===windowMs;select.append(option);}
  select.onchange=()=>windowMs=Number(select.value);
  const pause=el('button',{'aria-pressed':'false'},'暂停四图');pause.onclick=()=>{frozen=frozen?null:samples.slice();pause.textContent=frozen?'继续四图':'暂停四图';pause.setAttribute('aria-pressed',String(!!frozen));};
  const clear=el('button',{},'清空曲线');clear.onclick=()=>{samples=[];if(frozen)frozen=[];};
  const zeroLabel=el('label',{class:'scope-zero'}),zero=el('input',{type:'checkbox','aria-label':'速度纵轴包含零点'});zeroLabel.append(zero,document.createTextNode('速度含零点'));
  const restore=el('button',{},'恢复四图');restore.hidden=true;restore.onclick=()=>{grid.classList.remove('focused-scope');for(const card of grid.children)card.hidden=false;restore.hidden=true;};
  const exportButton=el('button',{},'导出四图 CSV');exportButton.onclick=()=>{const rows=frozen??samples;if(!rows.length)return api.notify('尚无波形数据');const names=['timeUs','targetRpm','rpm','rampRpm','speedValid','ia','ib','ic','id','iq','idTarget','iqTarget','theta'];api.download('foc-synchronized-scopes.csv','source,host_time_ms,'+names.join(',')+'\n'+rows.map(t=>[t.source,t.host,...names.map(name=>name==='rpm'&&!t.speedValid?'':t[name])].join(',')).join('\n'),'text/csv');};
  toolbar.append(status,select,zeroLabel,pause,clear,restore,exportButton);section.append(toolbar);
  const grid=el('div',{class:'scope-grid'});section.append(grid);
  for(const d of definitions){const card=el('section',{class:'scope-tile','data-scope':d.key}),heading=el('div',{class:'scope-tile-heading'});d.readout=el('span',{class:'scope-readout'},'等待数据');heading.append(el('h3',{},d.title),el('span',{class:'dim'},d.unit+(d.key==='speed'?' · 自动量程':'')));const legend=el('div',{class:'scope-tile-legend'});d.labels.forEach((label,i)=>{const item=el('span',{},label);item.style.color=colors[i];legend.append(item);});d.canvas=el('canvas',{'aria-label':d.title+'同步波形'});card.append(heading,legend,d.canvas,d.readout);grid.append(card);}
  for(const d of definitions){const card=grid.querySelector(`[data-scope="${d.key}"]`),expand=el('button',{class:'scope-expand','aria-label':'放大'+d.title},'放大');expand.onclick=()=>{grid.classList.add('focused-scope');for(const other of grid.children)other.hidden=other!==card;restore.hidden=false;};card.querySelector('.scope-tile-heading').append(expand);d.visible=d.labels.map(()=>true);[...card.querySelector('.scope-tile-legend').children].forEach((label,i)=>{const button=el('button',{'aria-label':'显示'+d.labels[i],'aria-pressed':'true'},label.textContent);button.style.color=colors[i];button.onclick=()=>{d.visible[i]=!d.visible[i];button.setAttribute('aria-pressed',String(d.visible[i]));};label.replaceWith(button);});d.includeZero=()=>zero.checked;}
  q('#live-monitor').append(section);q('#live-monitor').classList.add('parallel-monitor');
  function push(t){const host=t.host??Date.now();samples.push({...t,host,at:performance.now()+host-Date.now(),source:t.source??'device'});if(samples.length>1500)samples.shift();}
  api.onState(state=>{if(state.reset||state.transport!==source||!state.connected&&source==='serial'){samples=[];frozen=null;pause.textContent='暂停四图';pause.setAttribute('aria-pressed','false');}source=state.transport;});
  function paint(now){
    const demo=api.getState().transport==='demo';
    if(demo&&!api.remote&&now-lastDemo>=20){lastDemo=now;const phase=now/350;const sample={source:'demo',host:Date.now(),at:now,timeUs:Math.round(now*1000)>>>0,targetRpm:3000,rpm:3000+Math.sin(phase)*18,rampRpm:3000,speedValid:true,ia:Math.sin(phase)*1.3,ib:Math.sin(phase+2*Math.PI/3)*1.3,ic:Math.sin(phase+4*Math.PI/3)*1.3,id:.02+Math.sin(phase)*.01,iq:1.24+Math.sin(phase)*.03,idTarget:0,iqTarget:1.25,vbus:24.08,theta:phase};samples.push(sample);api.publishTelemetry?.(sample);if(samples.length>1500)samples.shift();}
    if(now-lastPaint>=50){lastPaint=now;const all=frozen??samples,end=all.at(-1)?.at??now,rows=all.filter(t=>t.at>=end-windowMs);status.textContent=(demo?'演示信号':samples.length?'设备遥测':'等待设备遥测')+' · '+(frozen?'显示已暂停':'四图同步')+' · 同一 '+windowMs/1000+' 秒窗口';for(const d of definitions)drawScope(d,rows,end,windowMs,colors);}
    requestAnimationFrame(paint);
  }
  requestAnimationFrame(paint);
  return {push,focus:key=>q(`[data-scope="${key}"]`).scrollIntoView({block:'nearest',behavior:'smooth'})};
}

function mountConvenience(api){
  const {q,el}=api,box=q('.recipe-builder'),key='foc-studio-recipes-v1';let saved={draft:null,templates:[]},timer;
  const fields=()=>[...box.querySelectorAll('input,select')].filter(n=>n.id!=='recipe-start'&&!n.closest('.recipe-library'));
  const fieldKey=n=>n.id||Object.entries(n.dataset).map(([k,v])=>k+':'+v).join('|');
  function capture(){return Object.fromEntries(fields().map(n=>[fieldKey(n),n.type==='checkbox'?n.checked:n.value]));}
  function persist(){try{localStorage.setItem(key,JSON.stringify(saved));return true;}catch{api.notify('浏览器无法保存本地草稿，请导出备份');return false;}}
  function apply(values){if(!values||typeof values!=='object')return;for(const n of fields()){const value=values[fieldKey(n)];if(n.type==='checkbox'){if(typeof value==='boolean')n.checked=value;}else if(typeof value==='string'&&value.length<100)n.value=value;}q('#recipe-start').checked=false;box.dispatchEvent(new Event('input',{bubbles:true}));}
  try{const raw=JSON.parse(localStorage.getItem(key)||'null');if(raw&&Array.isArray(raw.templates)){saved.draft=raw.draft;saved.templates=raw.templates.filter(t=>typeof t.name==='string'&&t.values&&typeof t.values==='object').slice(0,20);}}catch{}
  const library=el('div',{class:'recipe-library'}),name=el('input',{class:'studio-input',placeholder:'模板名称，例如：3508 无感观察',maxlength:'40','aria-label':'组合模板名称'}),select=el('select',{class:'studio-select','aria-label':'已保存组合模板'}),save=el('button',{},'保存模板'),load=el('button',{},'载入模板'),remove=el('button',{},'删除模板'),copy=el('button',{},'复制整套命令'),exportButton=el('button',{},'备份模板 JSON'),notice=el('span',{class:'studio-note'},'草稿自动保存在本机，载入不执行。');
  library.append(name,save,select,load,remove,copy,exportButton,notice);q('.recipe-fields').before(library);
  function options(){select.replaceChildren(el('option',{value:''},'选择已保存模板'));saved.templates.forEach((t,i)=>select.append(el('option',{value:String(i)},t.name)));load.disabled=remove.disabled=!saved.templates.length;}
  save.onclick=()=>{const title=name.value.trim();if(!title)return api.notify('请填写模板名称');const i=saved.templates.findIndex(t=>t.name===title),entry={name:title,at:Date.now(),values:capture()};if(i>=0)saved.templates[i]=entry;else if(saved.templates.length<20)saved.templates.push(entry);else return api.notify('最多保存 20 个模板，请先删除旧模板');if(persist()){options();select.value=String(saved.templates.findIndex(t=>t.name===title));notice.textContent='已保存“'+title+'”，未发送设备。';}};
  load.onclick=()=>{if(select.value==='')return api.notify('请先选择模板');const t=saved.templates[Number(select.value)];if(!t)return;apply(t.values);name.value=t.name;notice.textContent='已载入“'+t.name+'”，启动选项已清除，请核对后发送。';};
  remove.onclick=()=>{if(select.value==='')return;const i=Number(select.value);saved.templates.splice(i,1);persist();options();notice.textContent='已删除所选本地模板。';};
  copy.onclick=async()=>{const commands=[...q('#recipe-plan').querySelectorAll('code')].map(n=>n.textContent);if(!commands.length)return api.notify('请先修正参数，生成有效组合');try{await navigator.clipboard.writeText(commands.join('\n'));api.notify('已复制 '+commands.length+' 条命令，未发送');}catch{api.notify('复制失败，可在步骤列表选中复制');}};
  exportButton.onclick=()=>api.download('foc-recipes.json',JSON.stringify({version:1,...saved},null,2),'application/json');
  box.addEventListener('input',event=>{if(event.target.closest?.('.recipe-library'))return;clearTimeout(timer);timer=setTimeout(()=>{saved.draft=capture();persist();},400);});
  options();if(saved.draft){apply(saved.draft);notice.textContent='已恢复上次草稿，启动选项已清除；未发送设备。';}
  const guide=el('div',{class:'control-guide','aria-live':'polite'});q('.monitor-heading').after(guide);
  setInterval(()=>{const s=api.getState(),hint=q('#start-hint').textContent;guide.textContent=s.transport==='demo'?'演示预览：所有命令仅记录。实机操作请展开“串口与采集设置”。':!s.connected?'① 连接串口 → ② 回读并核对参数 → ③ 选择组合并发送 → ④ 明确点击启动':s.batch?'组合执行中 · 等待设备逐条确认，可点击“中止组合并停机”。':'已连接 · '+hint;},500);
  const jump=el('button',{class:'recipe-jump'},'组合操作 / 模板');jump.onclick=()=>{q('.studio-nav').click();box.scrollIntoView({block:'start',behavior:'smooth'});};q('.monitor-heading').append(jump);
  const query=el('button',{class:'recipe-jump'},'查询状态');query.onclick=()=>api.sendLine('olstatus');q('.monitor-heading').append(query);
}

function mountMonitorFeed(api){
  let queue=[],busy=false,lastSent=0;const publisher=crypto.randomUUID();
  const status=api.el('span',{id:'monitor-sync-state',class:'studio-note'},'独立波形页：等待同步');api.q('.monitor-heading').append(status);
  api.publishTelemetry=t=>{queue.push({...t,host:Date.now()});if(queue.length>100)queue.splice(0,queue.length-100);if(!busy&&Date.now()-lastSent>=40)void flush();};
  async function flush(){if(busy||!queue.length&&Date.now()-lastSent<800)return;busy=true;const samples=queue;queue=[];const state=api.getState();try{const response=await fetch('/monitor-feed',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({publisher,state:{transport:state.transport,connected:state.connected},samples}),signal:AbortSignal.timeout(1500)});if(!response.ok)throw Error(response.status===409?'另一控制页正在同步，真实串口优先':'波形同步失败 '+response.status);lastSent=Date.now();status.textContent=state.transport==='demo'?'独立波形页：演示信号':!state.connected?'独立波形页：串口未连接':samples.length?'独立波形页：真实数据已同步':'独立波形页：等待设备遥测';}catch(error){status.textContent=error.message;if(Date.now()-lastSent>3000){api.appendLog('MONITOR',error.message);lastSent=Date.now();}}finally{busy=false;}}
  setInterval(()=>void flush(),50);
  api.onState(()=>{queue=[];lastSent=0;void flush();});
}

function drawScope(d,rows,end,windowMs,colors){
  const canvas=d.canvas,w=canvas.clientWidth,h=canvas.clientHeight;if(w<1||h<1)return;
  const ratio=devicePixelRatio||1;if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)){canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);}
  const ctx=canvas.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);
  const left=48,right=w-10,top=10,bottom=h-23,values=rows.map(t=>d.values(t).map((v,i)=>d.visible?.[i]===false?null:v)),rangeRows=d.key==='speed'&&d.includeZero?.()?[...values,[0]]:values,{min,max}=scopeRange(rangeRows,d.key);
  ctx.font='9px Consolas';ctx.lineWidth=.6;
  for(let i=0;i<=4;i++){const y=top+(bottom-top)*i/4;ctx.strokeStyle='#293440';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();ctx.fillStyle='#8290a3';const v=max-(max-min)*i/4;ctx.fillText(Math.abs(v)>=100?Math.round(v).toString():v.toFixed(2),2,y+3);const x=left+(right-left)*i/4;ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,bottom);ctx.stroke();ctx.fillText(((i/4-1)*windowMs/1000).toFixed(1)+'s',x-12,h-6);}
  ctx.save();ctx.beginPath();ctx.rect(left,top,right-left,bottom-top);ctx.clip();
  d.labels.forEach((_,c)=>{if(d.visible?.[c]===false)return;let drawing=false,previous=null;ctx.strokeStyle=colors[c];ctx.lineWidth=1.4;ctx.beginPath();rows.forEach((t,i)=>{const value=values[i][c];if(!Number.isFinite(value)){drawing=false;return;}const x=left+(t.at-(end-windowMs))/windowMs*(right-left),y=top+(max-value)/(max-min)*(bottom-top);if(!drawing||previous!=null&&t.at-previous>150)ctx.moveTo(x,y);else ctx.lineTo(x,y);drawing=true;previous=t.at;});ctx.stroke();});ctx.restore();
  const latest=values.at(-1);d.readout.textContent=latest?d.labels.map((label,i)=>label+' '+(d.visible?.[i]===false?'已隐藏':Number.isFinite(latest[i])?latest[i].toFixed(d.key==='speed'?0:2):'无效')).join(' · '):'等待设备数据';
}
