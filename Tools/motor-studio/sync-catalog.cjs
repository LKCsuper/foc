// 只读取固件源码，生成页面目录；不修改任何固件文件。
const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '../..');
const read = p => fs.readFileSync(path.join(root, p), 'utf8');
const table = read('App/cmd/command.c');
const sources = fs.readdirSync(path.join(root, 'App/cmd')).filter(p => p.endsWith('.c'))
  .map(p => ({ file: 'App/cmd/' + p, text: read('App/cmd/' + p) }));
// 屏蔽字符串与注释，按真正的括号边界提取函数（不被 printf 中的括号干扰）。
const mask = s => s.replace(/\/\*[\s\S]*?\*\/|\/\/[^\n]*|"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'/g, x => x.replace(/[^\n]/g, ' '));
const functions = new Map();
for (const src of sources) {
  const clean = mask(src.text);
  for (const m of clean.matchAll(/\b([A-Za-z_]\w*)\s*\([^;{}]*\)\s*\{/g)) {
    if (['if', 'while', 'for', 'switch'].includes(m[1])) continue;
    const open = clean.indexOf('{', m.index); let depth = 1, end = open + 1;
    while (depth && end < clean.length) { if (clean[end] === '{') depth++; if (clean[end] === '}') depth--; end++; }
    functions.set(m[1], { body: src.text.slice(open, end), file: src.file, line: src.text.slice(0, m.index).split('\n').length });
  }
}
const categories = [
  ['基础控制', 'olspeed olstatus olrpm olboost olkvf olramp olvlimit olmax olmin olstart olstop olfault olmode olfb oliq olid fw olkp olki olphase olalign hfistart openloop duty'],
  ['速度与位置', 'speed spenable spkp spki spiq spbrake pos joint'],
  ['检测与校准', 'detect pitune encoder'], ['无感观测', 'slstats sltest slenable sltakeover slrpm slguard slmotor slobs sldt slphase'],
  ['BLDC 六步', 'bldc'], ['诊断与波形', 'sltrace slhandoff slcadence olsnap olcurdiag curtest plot'],
  ['系统与存储', 'saveparam show task reset help'],
];
const names = {studio:'网页与周期控制',
  olspeed:'目标转速',olstatus:'控制状态',olrpm:'转速与电流诊断',olboost:'V/F 启动补偿',olkvf:'V/F 电压斜率',olramp:'转速斜坡',olvlimit:'电压限幅',olmax:'最大转速',olmin:'默认起步转速',olstart:'启动电机',olstop:'停止电机',olfault:'故障查看与清除',olmode:'V/F 与 I/F 模式',olfb:'反馈源',oliq:'q 轴电流目标',olid:'d 轴电流目标',fw:'弱磁',olkp:'电流环 Kp',olki:'电流环 Ki',olphase:'旧启动相位偏置',olalign:'转子预对齐',hfistart:'HFI 启动扫描',openloop:'开环后台任务',duty:'占空比调试',speed:'速度状态与测试',spenable:'速度环使能',spkp:'速度环 Kp',spki:'速度环 Ki',spiq:'速度环电流限幅',spbrake:'速度环制动',pos:'位置控制',joint:'关节控制',detect:'参数检测与校准',pitune:'PI 自整定预览',encoder:'编码器与齿槽补偿',slstats:'无感稳态统计',sltest:'无感分档测试',slenable:'观测器使能',sltakeover:'观测器角度接管',slrpm:'接管与回退转速',slguard:'无感交接护栏',slmotor:'无感电机模型',slobs:'观测器与 PLL 增益',sldt:'观测电压死区补偿',slphase:'观测器相位偏置',bldc:'六步换相控制',sltrace:'运行与故障轨迹',slhandoff:'交接与 PWM 时序诊断',slcadence:'中断与任务节拍诊断',olsnap:'短窗口快照',olcurdiag:'分扇区电流诊断',curtest:'静态 d 轴电流测试',plot:'实时数据流',saveparam:'保存参数至 Flash',show:'霍尔电平',task:'任务、堆栈与 CPU',reset:'复位 MCU',help:'固件命令列表',
};
const collect = (name, seen = new Set()) => {
  if (seen.has(name) || !functions.has(name)) return '';
  seen.add(name); const fn = functions.get(name); let body = fn.body;
  for (const m of fn.body.matchAll(/\b([A-Za-z_]\w*)\s*\(/g)) if (functions.has(m[1])) body += '\n' + collect(m[1], seen);
  return body;
};
const commands = [...table.matchAll(/\{\s*"(\w+)"\s*,\s*(\w+)\s*,\s*"((?:\\.|[^"\\])*)"\s*\}/g)].map(m => {
  const fn = functions.get(m[2]); if (!fn) throw Error('Missing command implementation: ' + m[2]);
  const body = collect(m[2]); const strings = [...body.matchAll(/"((?:\\.|[^"\\])*)"/g)].map(x => {
    try { return JSON.parse('"'+x[1]+'"'); } catch { return x[1]; }
  });
  return { name:m[1], title:names[m[1]] || m[1], description:m[3], category:categories.find(x => x[1].split(' ').includes(m[1]))?.[0] || '系统与存储',
    file:fn.file, line:fn.line,
    subcommands:[...new Set([...body.matchAll(/strcmp\s*\(\s*argv\[1\]\s*,\s*"([^"\\]+)"/g)].map(x=>x[1]))],
    usage:[...new Set(strings.flatMap(x=>x.split(/\r?\n/)).filter(x=>x.includes(m[1]) && (/usage|Usage|<|\[|\|/.test(x) || x.trim().startsWith(m[1]+' '))))],
    gate: table.slice(0,m.index).split(/\r?\n/).filter(x=>/^#(if|endif)/.test(x)).slice(-1)[0] || '',
  };
});
commands.push({name:'help',title:names.help,category:'系统与存储',description:'固件命令列表',file:'App/cmd/command.c',line:447,subcommands:[],usage:['help']});
function fields(structName, file) {
  const text=read(file); const end=text.indexOf('} '+structName+';'); const begin=text.lastIndexOf('typedef struct',end);
  return [...text.slice(begin,end).matchAll(/^\s*(float|bool|u?int\d*_t|int)\s+(\w+)(\[[^\]]+\])?;[ \t]*(?:\/\*([^\n]*?)\*\/)?/gm)].map(m=>({name:m[2],type:m[1],array:m[3] || '',description:(m[4]||'').trim(),file,line:text.slice(0,begin+m.index).split('\n').length}));
}
const paramC=read('App/motor_param/motor_param.c');
const defaults=Object.fromEntries([...paramC.matchAll(/gpstMotorDefault->(\w+)\s*=\s*([^;]+);/g)].map(x=>[x[1],x[2].trim()]));
const config=fields('motor_config_t','App/motor_param/inc/motor_settings.h').map(x=>({...x,default:defaults[x.name] ?? null}));
const calibration=fields('motor_calibration_t','App/motor_param/inc/motor_settings.h');
const runtime=fields('motor_runtime_t','App/motor_param/inc/motor_param.h');
const profiles=['motor_3508.h','motor_2804_220kv.h','motor_c2208_100t.h','motor_atk_bl57h95d20e.h'].map(file=>{
  const text=read('App/motor_param/inc/'+file);
  const macros=[...text.matchAll(/^#define\s+(MOTOR_\w+)\s+([^\r\n]+)$/gm)].map(m=>{const pair=m[2].split(/\s*\/\*/);return {name:m[1],value:pair[0].trim(),description:(pair[1] || '').replace(/\*\/.*$/,'').trim()};});
  return {file:'App/motor_param/inc/'+file,name:macros.find(x=>/MODEL$/.test(x.name))?.value.replaceAll('"','') || file,macros,note:(text.match(/\/\*([\s\S]*?)\*\//)||[])[1]?.replace(/^\s*\* ?/gm,'').trim() || ''};
});
const baud=Number(read('Library/Bsp/inc/bsp_usart.h').match(/#define\s+DEBUG_USART_BAUD\s+(\d+)/)[1]);
const catalog={commands,config,calibration,runtime,profiles,defaults,baud,commandBytes:159,maxArgs:8};
fs.writeFileSync(path.join(__dirname,'catalog.js'), '/* 由 sync-catalog.cjs 从当前固件源码生成。 */\nexport default '+JSON.stringify(catalog,null,2)+';\n');
console.log(`${commands.length} commands, ${config.length} config, ${calibration.length} calibration, ${runtime.length} runtime fields; baud ${baud}`);
