import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {crc16,encodeTarget,encodePacket,StudioDecoder,decodeTelemetry,TargetPump,buildOpenLoopRecipe,startAvailability,scopeRange,buildModeRecipe,detectionCommand,parseDetectionResult,detectionCompletion} from './studio-link.mjs';
import catalog from './catalog.js';
test('inline calibration uses RAM variants and waits for the correct completion rather than an acceptance reply',()=>{
  assert.equal(detectionCommand('offset'),'detect offset ram');assert.equal(detectionCommand('encoder'),'encoder detect ram');assert.equal(detectionCommand('hall'),'detect hall');assert.equal(detectionCommand('phase'),'detect phase');
  assert.equal(detectionCompletion('encoder','Encoder detect: align theta=0'),null);assert.equal(detectionCompletion('encoder','Encoder detect: count=10 mech=0 elec0=0 zero_ok=1'),true);assert.equal(detectionCompletion('encoder','Encoder detect rejected: data invalid'),false);
  assert.equal(detectionCompletion('hall','Detect End: SUCCESS'),true);assert.equal(detectionCompletion('offset','Detect End: FAILED'),false);assert.equal(detectionCompletion('fluxscan','Detect End: SUCCESS'),null);assert.equal(detectionCompletion('fluxscan','Detect motor finished: SUCCESS'),true);
});
test('dedicated detection commands preserve firmware defaults and reject gaps or invalid test arguments',()=>{
  assert.equal(detectionCommand('rl',['']),'detect rl');assert.equal(detectionCommand('motor',['.5','1200','.3']),'detect motor 0.5 1200 0.3');assert.equal(detectionCommand('fluxscan',['.5','3000','.3']),'detect fluxscan 0.5 3000 0.3');
  for(const [kind,fields]of [['motor',['','1200']],['rl',['NaN']],['rl',['-1']],['fluxscan',['.5','800']],['invalid',[]]])assert.throws(()=>detectionCommand(kind,fields));
});
test('detection result parser respects VALID flags and units and takes the latest report',()=>{
  assert.equal(parseDetectionResult('waiting…'),null);
  const report='===== Motor Parameter Detect =====\n Status : OK\n Resistance : VALID 0.474 ohm\n Inductance : VALID 112.0 uH\n Flux : INVALID 0.000 mWb\n';
  const result=parseDetectionResult(report);assert.equal(result.resistance.value,.474);assert.equal(result.inductance.value,112);assert.equal(result.flux.valid,false);
  const next=parseDetectionResult(report+'===== Motor Parameter Detect =====\n Status : OK\n Resistance : VALID 0.5 ohm\n Inductance : VALID 120 uH\n Flux : VALID 2.39 mWb\n');assert.equal(next.resistance.value,.5);assert.equal(next.flux.valid,true);assert.equal(next.inductance.value/1000,.12);
});
test('unified mode plans configure sensors and loops before explicit start; position defaults to holding current position',()=>{
  const device={ol_max_rpm:6000,speed_iq_limit:2,encoder_calibrated:1,hall_table:[255,1,20,40,60,80,100,255],sensorless_transition_rpm:500,sensorless_fallback_rpm:300,sensorless_motor_r:.4,sensorless_motor_l:.0001,sensorless_flux_linkage:.002};
  for(const mode of ['open','current','sensored','sensorless','position']){const steps=buildModeRecipe({mode,target:mode==='current'?.3:800,device}),lines=steps.map(s=>s.line);assert.equal(lines[0],'olstop');assert.equal(lines.at(-2),'olstart');for(const s of steps)assert.ok(catalog.commands.some(c=>c.name===s.line.split(' ')[0]));if(mode==='current'){assert.ok(lines.includes('oliq 0.3'));assert.ok(!lines.includes('spenable 1'));}if(mode==='sensored')assert.ok(lines.includes('spenable 1'));if(mode==='position'){assert.ok(lines.includes('pos hold'));assert.ok(!lines.includes('pos zero'));}}
  assert.ok(buildModeRecipe({mode:'position',hold:false,target:30,device}).some(s=>s.line==='pos set 30'));
  assert.ok(buildModeRecipe({mode:'sensored',feedback:'hall',device}).some(s=>s.line==='olfb hall'));
  for(const args of [{mode:'sensored',device:{...device,encoder_calibrated:0}},{mode:'sensored',feedback:'hall',device:{...device,hall_table:[]}},{mode:'current',target:-1,device},{mode:'current',target:3,device},{mode:'open',target:7000,device},{mode:'sensorless',target:300,device},{mode:'position',feedback:'hall',device}])assert.throws(()=>buildModeRecipe(args));
});
test('speed axis follows the visible data including reverse and invalid feedback without forcing zero',()=>{
  assert.deepEqual(scopeRange([[300,301,300],[300,299,null]],'speed'),{min:289,max:311});
  assert.deepEqual(scopeRange([[-300,-302,-300]],'speed'),{min:-312,max:-290});
  assert.deepEqual(scopeRange([[null,NaN,Infinity]],'speed'),{min:-10,max:10});
  assert.deepEqual(scopeRange([[800,800,800]],'speed'),{min:790,max:810});
  const ramp=scopeRange([[0,300,200]],'speed');assert.ok(ramp.min<0&&ramp.max>300);
  assert.ok(scopeRange([[1,2]],'current').min<0);
});
test('ordinary start uses configured target without requiring a periodic session; guards remain enforced',()=>{
  const state={connected:true,ready:true,batch:false,pending:0,active:false,age:20,telemetry:{targetRpm:300,enabled:false,faultLatched:false,owned:false,appliedSequence:0}};
  assert.equal(startAvailability(state).enabled,true);assert.equal(startAvailability(state).line,'olstart');
  for(const patch of [{connected:false},{ready:false},{batch:true},{pending:1},{age:151},{telemetry:{...state.telemetry,targetRpm:0}},{telemetry:{...state.telemetry,faultLatched:true}},{telemetry:{...state.telemetry,enabled:true}},{telemetry:{...state.telemetry,owned:true}},{active:true}])assert.equal(startAvailability({...state,...patch}).enabled,false);
  const periodic=startAvailability({...state,active:true,telemetry:{...state.telemetry,owned:true,appliedSequence:3}});assert.equal(periodic.enabled,true);assert.equal(periodic.line,'studio start');
});

test('sensorless observation never arms takeover or speed loop; running enables observer before takeover',()=>{
  const observe=buildOpenLoopRecipe({mode:'slobserve',rpm:300,start:true,speedLoop:true}).map(s=>s.line);
  assert.ok(observe.includes('slenable 1'));assert.ok(!observe.includes('sltakeover 1'));assert.ok(!observe.includes('spenable 1'));assert.ok(!observe.includes('olfb sensorless'));
  const run=buildOpenLoopRecipe({mode:'slrun',rpm:800,start:true,speedLoop:true,options:{model:[.474,.112,2.39],threshold:[500,300],observer:[12000000,2000,10000],guard:[800,1.4,.2],phase:0,spiq:.5}});
  const lines=run.map(s=>s.line);assert.ok(lines.includes('slmotor 0.474 0.112 2.39'));assert.ok(lines.indexOf('slenable 1')<lines.indexOf('sltakeover 1'));assert.ok(lines.indexOf('spenable 1')<lines.indexOf('olstart'));assert.ok(lines.indexOf('slrpm 500 300')<lines.indexOf('sltakeover 1'));
  for(const step of run)assert.ok(catalog.commands.some(c=>c.name===step.line.split(' ')[0]));
});
test('sensorless start requires known model and thresholds and rejects incomplete groups or low speed',()=>{
  assert.throws(()=>buildOpenLoopRecipe({mode:'slrun',rpm:800,start:true}),/接管转速/);
  assert.throws(()=>buildOpenLoopRecipe({mode:'slrun',rpm:300,start:true,options:{threshold:[500,300],model:[.4,.1,2]}}),/低于接管/);
  assert.throws(()=>buildOpenLoopRecipe({mode:'slrun',options:{model:[.4,.1]}}),/完整/);
  assert.throws(()=>buildOpenLoopRecipe({mode:'slrun',options:{threshold:[300,500]}}),/回退/);
  const device={sensorless_transition_rpm:500,sensorless_fallback_rpm:300,sensorless_motor_r:.474,sensorless_motor_l:.000112,sensorless_flux_linkage:.00239};
  assert.ok(buildOpenLoopRecipe({mode:'slrun',rpm:800,start:true,device}).some(s=>s.line==='olstart'));
  assert.ok(!buildOpenLoopRecipe({mode:'slrun',rpm:300}).some(s=>s.line==='olstart'));
});

test('open-loop recipe orders dependencies and never starts unless explicitly selected',()=>{
  const steps=buildOpenLoopRecipe({rpm:300,options:{iq:.3,id:0,ramp:10}}),lines=steps.map(s=>s.line);
  assert.deepEqual(lines.slice(0,5),['olstop','spenable 0','sltakeover 0','olfb openloop','olmode 1']);
  assert.ok(lines.includes('olspeed 300'));assert.ok(lines.includes('oliq 0.3'));assert.ok(!lines.includes('olstart'));assert.equal(lines.at(-1),'olstatus');
  const starts=buildOpenLoopRecipe({rpm:-300,start:true}).map(s=>s.line);assert.deepEqual(starts.slice(-3),['olspeed -300','olstart','olstatus']);
  for(const step of steps)assert.ok(catalog.commands.some(c=>c.name===step.line.split(' ')[0]));
});
test('recipe validates every selected input and excludes parameters for the other mode',()=>{
  for(const args of [{rpm:''},{rpm:NaN},{mode:'unknown'},{rpm:0,start:true},{rpm:300,options:{max:200}},{options:{iq:Infinity}},{options:{ramp:-1}},{options:{ki:''}}])assert.throws(()=>buildOpenLoopRecipe(args));
  const vf=buildOpenLoopRecipe({mode:'vf',options:{iq:1,boost:1,kvf:.057}}).map(s=>s.line);assert.ok(vf.includes('olmode 0'));assert.ok(vf.includes('olboost 1'));assert.ok(!vf.some(s=>s.startsWith('oliq')));
  const current=buildOpenLoopRecipe({mode:'if',options:{boost:1,iq:.3}}).map(s=>s.line);assert.ok(!current.some(s=>s.startsWith('olboost')));assert.ok(!current.some(s=>/saveparam|olfault clear/.test(s)));
});

test('CRC matches CCITT-FALSE standard vector and control wire is bounded ASCII',()=>{
  assert.equal(crc16(new TextEncoder().encode('123456789')),0x29b1);
  const line=new TextDecoder().decode(encodeTarget(77,1,{rpm:-100,iq:.3,id:0}));
  assert.match(line,/^@C1,77,1,-100,0\.3,0\*[A-F0-9]{4}\r$/);assert.ok(line.length<160);
  assert.throws(()=>encodeTarget(1,1,{rpm:NaN,iq:0,id:0}));
  assert.throws(()=>encodeTarget(0,1,{rpm:1,iq:0,id:0}));
});
test('framed telemetry recovers from every USB split, CRC damage and mixed log output',()=>{
  const payload=new Uint8Array(72),view=new DataView(payload.buffer);view.setUint32(12,8|16|256,true);view.setFloat32(28,120,true);
  const frame=encodePacket(1,12,payload),prefix=new TextEncoder().encode('FOC ready\r\n');const all=new Uint8Array(prefix.length+frame.length);all.set(prefix);all.set(frame,prefix.length);
  for(let split=1;split<all.length;split++){let text='',frames=[];const decoder=new StudioDecoder(p=>frames.push(p),b=>text+=new TextDecoder().decode(b));decoder.push(all.slice(0,split));decoder.push(all.slice(split));assert.equal(text,'FOC ready\r\n');assert.equal(frames.length,1);const t=decodeTelemetry(frames[0].payload);assert.equal(t.rpm,120);assert.equal(t.speedValid,true);assert.equal(t.owned,true);}
  let count=0;const decoder=new StudioDecoder(()=>count++,()=>{}),bad=frame.slice();bad[20]^=1;decoder.push(bad);decoder.push(frame);assert.equal(count,1);assert.equal(decoder.badCrc,1);
  assert.throws(()=>decodeTelemetry(new Uint8Array(3)));
});
test('50 Hz pump sends latest values, never queues old targets, and rejects stale receive',async()=>{
  let now=0,writes=[],release,reason='';const pump=new TargetPump({now:()=>now,write:bytes=>{writes.push(new TextDecoder().decode(bytes));return new Promise(r=>release=r);},onStop:r=>reason=r});
  pump.start(1,{rpm:100,iq:.5,id:0});const pending=pump.tick();
  pump.setTarget({rpm:200,iq:.6,id:0});now=20;await pump.tick();assert.equal(writes.length,1);assert.equal(pump.skipped,1);
  release();await pending;now=40;const next=pump.tick();assert.match(writes[1],/,2,200,0.6,0\*/);release();await next;
  now=151;await pump.tick();assert.equal(pump.active,false);assert.match(reason,/150 ms/);assert.equal(writes.length,2);
});
test('late writer completion cannot mutate a stopped generation; invalid ACK stops control',async()=>{
  let resolve;const pump=new TargetPump({write:()=>new Promise(r=>resolve=r)});pump.start(10,{rpm:0,iq:0,id:0});const write=pump.tick();pump.stop();resolve();await write;assert.equal(pump.sequence,0);assert.equal(pump.sent,0);
  pump.start(11,{rpm:0,iq:0,id:0});pump.observe({token:10,owned:true});assert.equal(pump.active,false);
});
test('virtual foreground second produces 50 opportunities, not catch-up bursts',async()=>{
  let now=0,count=0;const pump=new TargetPump({now:()=>now,write:async()=>count++});pump.start(2,{rpm:10,iq:.3,id:0});
  for(let i=0;i<50;i++){now=i*20;pump.observe({token:2,owned:true,expired:false,result:0});await pump.tick();}
  assert.equal(count,50);assert.equal(pump.sequence,50);
});
test('firmware parameter IDs match the web catalog exactly; frequency stays readonly',()=>{
  const fields=readFileSync(new URL('../../App/cmd/studio_fields.inc',import.meta.url),'utf8');const entries=[...fields.matchAll(/FIELD\((\w+),\s*(\d),\s*(\d)\)/g)];
  assert.deepEqual(entries.map(x=>x[1]),catalog.config.map(x=>x.name));assert.equal(entries.length,43);assert.equal(entries.find(x=>x[1]==='ol_foc_freq')[3],'0');
  assert.equal([...fields.matchAll(/^CAL\(/gm)].length,20);
});
