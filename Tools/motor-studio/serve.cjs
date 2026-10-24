const http=require('node:http');
const fs=require('node:fs');
const path=require('node:path');

// 两个端口共享内存中的只读遥测；串口仍由控制页独占。
function createStudioServers(){
  const clients=new Set();let snapshot={state:{transport:'serial',connected:false},samples:[],receivedAt:0},publisher='',publisherAt=0;
  const assets={'/workbench.css':['workbench.css','text/css'],'/workbench.js':['workbench.js','text/javascript'],'/catalog.js':['catalog.js','text/javascript'],'/protocol.mjs':['protocol.mjs','text/javascript'],'/command-help.mjs':['command-help.mjs','text/javascript'],'/studio-link.mjs':['studio-link.mjs','text/javascript'],'/full-studio.js':['full-studio.js','text/javascript'],'/monitor.js':['monitor.js','text/javascript']};
  function event(response,data){if(response.destroyed||response.writableLength>65536)return;response.write('data: '+JSON.stringify(data)+'\n\n');}
  function handler(monitor){return (request,response)=>{
    const pathname=new URL(request.url,'http://127.0.0.1').pathname;
    if(pathname==='/monitor-events'&&monitor&&request.method==='GET'){
      response.writeHead(200,{'Content-Type':'text/event-stream','Cache-Control':'no-store','Connection':'keep-alive'});response.write(': monitor connected\n\n');clients.add(response);event(response,{...snapshot,reset:true});request.on('close',()=>clients.delete(response));return;
    }
    if(pathname==='/monitor-feed'&&!monitor&&request.method==='POST'){
      const origin=request.headers.origin;if(origin&&!['http://127.0.0.1:4174','http://localhost:4174'].includes(origin)||!String(request.headers['content-type']).startsWith('application/json')){response.writeHead(403);response.end();return;}
      let body='';request.on('data',chunk=>{body+=chunk;if(body.length>65536){response.writeHead(413);response.end();request.destroy();}});
      request.on('end',()=>{if(response.writableEnded)return;try{
        const data=JSON.parse(body),now=Date.now();if(typeof data.publisher!=='string'||data.publisher.length>100||!['serial','demo'].includes(data.state?.transport)||typeof data.state?.connected!=='boolean'||!Array.isArray(data.samples)||data.samples.length>100)throw Error('bad payload');
        const priority=state=>state.transport==='serial'?(state.connected?3:0):1;
        if(publisher&&publisher!==data.publisher&&now-publisherAt<2500&&priority(data.state)<=priority(snapshot.state)){response.writeHead(409);response.end('另一控制页正在同步；已连接的真实串口优先');return;}
        const fields=['timeUs','targetRpm','rpm','rampRpm','ia','ib','ic','id','iq','vbus','theta','idTarget','iqTarget','host'];
        for(const t of data.samples)if(!fields.every(key=>Number.isFinite(t[key]))||typeof t.speedValid!=='boolean')throw Error('bad sample');
        const changed=publisher!==data.publisher||snapshot.state.transport!==data.state.transport||snapshot.state.connected!==data.state.connected;
        publisher=data.publisher;publisherAt=now;
        const samples=data.samples.map(t=>({...t,source:data.state.transport==='demo'?'demo':'device'}));
        snapshot={state:data.state,samples:[...(changed?[]:snapshot.samples),...samples].slice(-500),receivedAt:now};
        for(const client of clients)event(client,{state:data.state,samples,receivedAt:now,reset:changed});response.writeHead(204);response.end();
      }catch{response.writeHead(400);response.end('无效遥测');}});return;
    }
    if(request.method!=='GET'){response.writeHead(405);response.end();return;}
    const route=pathname==='/'||pathname==='/index.html'?[monitor?'monitor.html':'index.html','text/html']:assets[pathname];
    if(!route){response.writeHead(404);response.end('Not found');return;}
    try{const content=fs.readFileSync(path.join(__dirname,route[0]));response.writeHead(200,{'Content-Type':route[1]+'; charset=utf-8','Cache-Control':'no-store'});response.end(content);}catch{response.writeHead(404);response.end('Local asset missing');}
  };}
  const heartbeat=setInterval(()=>{for(const client of clients)if(!client.destroyed&&client.writableLength<65536)client.write(': heartbeat\n\n');},1000);heartbeat.unref();
  return {control:http.createServer(handler(false)),monitor:http.createServer(handler(true)),close(){clearInterval(heartbeat);for(const client of clients)client.end();}};
}
if(require.main===module){const servers=createStudioServers();for(const [key,port]of [['control',4174],['monitor',4175]]){servers[key].on('error',error=>{console.error(port+' '+error.message);process.exitCode=1;});servers[key].listen(port,'127.0.0.1',()=>console.log((key==='control'?'控制台 ':'波形监视 ')+'http://127.0.0.1:'+port+'/'));}}
module.exports={createStudioServers};
