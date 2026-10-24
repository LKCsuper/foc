const test=require('node:test'),assert=require('node:assert/strict');
const {createStudioServers}=require('./serve.cjs');
test('separate ports relay read-only telemetry, replay snapshot and reject monitor writes',async()=>{
  const servers=createStudioServers();
  await Promise.all([new Promise(r=>servers.control.listen(0,'127.0.0.1',r)),new Promise(r=>servers.monitor.listen(0,'127.0.0.1',r))]);
  const control='http://127.0.0.1:'+servers.control.address().port,monitor='http://127.0.0.1:'+servers.monitor.address().port;
  let reader;
  try{
    assert.match(await (await fetch(control)).text(),/电机调试台/);assert.match(await (await fetch(monitor)).text(),/独立波形监视/);
    const sample=Object.fromEntries(['timeUs','targetRpm','rpm','rampRpm','ia','ib','ic','id','iq','vbus','theta','idTarget','iqTarget','host'].map(k=>[k,1]));sample.speedValid=false;
    const payload={publisher:'test-control',state:{transport:'serial',connected:true},samples:[sample]};
    const send=(url,data=payload,origin)=>fetch(url+'/monitor-feed',{method:'POST',headers:{'Content-Type':'application/json',...(origin?{Origin:origin}:{})},body:JSON.stringify(data)});
    assert.equal((await send(control,{...payload,publisher:'earlier-demo',state:{transport:'demo',connected:false}})).status,204);
    // 真实串口立即接替先打开的演示页，无需等演示页关闭或超时。
    assert.equal((await send(control)).status,204);
    assert.equal((await send(control,{...payload,publisher:'earlier-demo',state:{transport:'demo',connected:false}})).status,409);
    const stream=await fetch(monitor+'/monitor-events');reader=stream.body.getReader();const text=new TextDecoder().decode((await reader.read()).value);assert.match(text,/"reset":true/);assert.match(text,/"speedValid":false/);assert.match(text,/"source":"device"/);
    const next=reader.read();payload.samples=[{...sample,timeUs:20000}];assert.equal((await send(control)).status,204);assert.match(new TextDecoder().decode((await next).value),/"timeUs":20000/);
    assert.equal((await send(monitor)).status,405);assert.equal((await send(control,payload,'https://untrusted.example')).status,403);
    assert.equal((await send(control,{...payload,publisher:'competing-control'})).status,409);
    assert.equal((await send(control,{...payload,samples:[{...sample,iq:null}]})).status,400);
    assert.equal((await fetch(control+'/monitor-events')).status,404);
  }finally{await reader?.cancel();servers.close();await Promise.all([new Promise(r=>servers.control.close(r)),new Promise(r=>servers.monitor.close(r))]);}
});
