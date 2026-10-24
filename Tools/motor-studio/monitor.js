import {mountParallelScopes} from './full-studio.js';
const q=selector=>document.querySelector(selector);
const el=(tag,attrs={},text='')=>{const node=document.createElement(tag);for(const [key,value]of Object.entries(attrs))node.setAttribute(key,value);node.textContent=text;return node;};
let state={transport:'serial',connected:false},receivedAt=0,lastSampleAt=0,link=false;
const listeners=new Set(),banner=q('#relay-status');
const api={q,el,remote:true,getState:()=>state,onState:listener=>listeners.add(listener),notify:text=>{banner.textContent=text;},download:(name,data,type)=>{const url=URL.createObjectURL(new Blob([data],{type})),a=el('a',{href:url,download:name});a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}};
const scopes=mountParallelScopes(api),events=new EventSource('/monitor-events');
events.onopen=()=>link=true;
events.onerror=()=>{link=false;banner.classList.add('stale');banner.textContent='本地同步连接中断，正在重连；当前曲线为最后缓存。';};
events.onmessage=event=>{try{const data=JSON.parse(event.data);if(!Array.isArray(data.samples)||!data.state)return;receivedAt=data.receivedAt;
  if(data.reset||state.transport!==data.state.transport||state.connected!==data.state.connected){state=data.state;for(const listener of listeners)listener({...state,reset:true});}
  for(const sample of data.samples){scopes.push(sample);lastSampleAt=sample.host;}
}catch{banner.textContent='同步数据无效，等待下一帧';}};
setInterval(()=>{if(!link)return;const age=Date.now()-receivedAt,sampleAge=Date.now()-lastSampleAt,stale=!receivedAt||age>2000||sampleAge>1500||state.transport==='serial'&&!state.connected;banner.classList.toggle('stale',stale);
  banner.textContent=!receivedAt?'等待控制台同步，请打开 4174 控制台。':age>2000?'控制台同步已过期 · 最后更新 '+Math.round(age/1000)+' 秒前 · 当前曲线为缓存':state.transport==='demo'?(sampleAge>1500?'演示数据已暂停，当前曲线为缓存':'演示信号 · 来自控制台，未连接真实设备'):!state.connected?'控制台串口未连接 · 当前曲线为缓存':sampleAge>1500?'串口已连接，但设备遥测已过期 · 当前曲线为缓存':'设备遥测 · 数据年龄 '+Math.max(0,sampleAge)+' ms · 只读监视';
},200);
