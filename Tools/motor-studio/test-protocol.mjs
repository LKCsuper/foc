import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {SerialDecoder, validateCommand, numericDraft} from './protocol.mjs';
import {commandHelp, actionHelp, describeCommand} from './command-help.mjs';

function frame(values) {
  const bytes=new Uint8Array(values.length*4+4), view=new DataView(bytes.buffer);
  values.forEach((v,i)=>view.setFloat32(i*4,v,true));bytes.set([0,0,128,127],values.length*4);return bytes;
}
test('commands reject truncation, line injection, unknown names and empty numeric drafts',()=>{
  assert.equal(validateCommand(' slmotor 0.474 0.112 2.39 ',['slmotor']),'slmotor 0.474 0.112 2.39');
  for(const line of ['olstop\rolstart','olstop\nolstart','olstop\x1b','madeup'])assert.throws(()=>validateCommand(line,['olstop']));
  assert.throws(()=>validateCommand('sltest 1 2 3 4 5 6 7 8',['sltest']));
  assert.throws(()=>validateCommand('task '+ 'a'.repeat(155),['task']));
  assert.equal(validateCommand('task '+ 'a'.repeat(154),['task']).length,159);
  for(const v of [null,undefined,'', ' ', 'NaN','Infinity'])assert.throws(()=>numericDraft(v));
  assert.equal(numericDraft('-0.12'),'-0.12');
});
test('JustFloat survives every two-part USB split, including split tail and preceding reply',()=>{
  const prefix=new TextEncoder().encode('plot: started\r\n'), payload=frame([1,-2,3,24]);
  const all=new Uint8Array(prefix.length+payload.length);all.set(prefix);all.set(payload,prefix.length);
  for(let split=1;split<all.length;split++){
    let text='', frames=[];const decoder=new SerialDecoder(t=>text+=t,(mode,values)=>frames.push({mode,values}));
    decoder.setMode('current');decoder.push(all.slice(0,split));decoder.push(all.slice(split));
    assert.deepEqual(frames,[{mode:'current',values:[1,-2,3,24]}],`split ${split}`);
    assert.equal(text,'plot: started\r\n');
  }
});
test('binary payload that resembles ASCII text must not be consumed as a log line',()=>{
  const payload=new Uint8Array(20);payload.set(new TextEncoder().encode('abc\ndefghijklmno'));payload.set([0,0,128,127],16);
  const values=[];const decoder=new SerialDecoder(()=>{},(mode,v)=>values.push(v));decoder.setMode('current');
  for(const b of payload)decoder.push(new Uint8Array([b]));assert.equal(values.length,1);
});
test('multiple frames, five-channel modes, NaN rejection and stream recovery',()=>{
  const values=[],decoder=new SerialDecoder(()=>{},(mode,v)=>values.push(v));decoder.setMode('speed');
  const a=frame([100,99,98,1,2]),bad=frame([NaN,1,2,3,4]);decoder.push(a);decoder.push(bad);decoder.push(a);
  assert.equal(values.length,2);assert.deepEqual(values[1],[100,99,98,1,2]);
  decoder.setMode(null);let text='';const plain=new SerialDecoder(t=>text+=t,()=>{});
  const utf8=new TextEncoder().encode('中文回复\r\n');for(const byte of utf8)plain.push(new Uint8Array([byte]));assert.equal(text,'中文回复\r\n');
});
test('catalog covers every firmware registration, every configuration field and nested encoder controls',()=>{
  const catalog=JSON.parse(readFileSync(new URL('./catalog.js',import.meta.url),'utf8').split('export default ')[1].replace(/;\s*$/,''));
  const source=readFileSync(new URL('../../App/cmd/command.c',import.meta.url),'utf8');
  const registered=[...source.matchAll(/\{\s*"(\w+)"\s*,\s*\w+\s*,\s*"/g)].map(m=>m[1]);
  assert.deepEqual(catalog.commands.map(c=>c.name).sort(),[...registered,'help'].sort());
  const header=readFileSync(new URL('../../App/motor_param/inc/motor_settings.h',import.meta.url),'utf8').split('} motor_config_t;')[0];
  for(const m of header.matchAll(/^\s*(?:float|bool|uint16_t|int)\s+(\w+);/gm))assert.ok(catalog.config.some(f=>f.name===m[1]),m[1]);
  const encoder=catalog.commands.find(c=>c.name==='encoder');assert.ok(encoder.usage.some(s=>s.includes('cog')));
  assert.ok(catalog.commands.find(c=>c.name==='sltrace').subcommands.includes('tracking-arm'));
  assert.equal(catalog.baud,2000000);
});

test('every registered command and discovered subcommand has Chinese usage guidance',()=>{
  const catalog=JSON.parse(readFileSync(new URL('./catalog.js',import.meta.url),'utf8').split('export default ')[1].replace(/;\s*$/,''));
  for(const command of catalog.commands){
    assert.ok(commandHelp[command.name],command.name);
    for(const sub of command.subcommands)assert.ok(actionHelp[command.name+' '+sub],command.name+' '+sub);
    assert.ok(commandHelp[command.name].parameters);
    for(const field of ['title','purpose','conditions','effect'])assert.match(commandHelp[command.name][field],/[\u4e00-\u9fff]/,command.name+' '+field);
  }
  assert.match(describeCommand('encoder cog learn 1').effect,/Flash/);
  assert.match(describeCommand('encoder compare 1').effect,/保持不变/);
  assert.match(describeCommand('encoder detect ram').effect,/RAM/);
  assert.match(describeCommand('encoder reset').purpose,/不改写/);
  assert.match(describeCommand('plot stop').effect,/不停止电机/);
});
