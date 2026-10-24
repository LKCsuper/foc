export function validateCommand(input, knownNames) {
  const line = input.trim();
  if (!line || /[^\x20-\x7e\t]/.test(line)) throw Error('命令只能包含一行英文字符，不支持换行或控制字符');
  if (new TextEncoder().encode(line).length > 159) throw Error('命令超过固件的 159 字节限制');
  const args = line.split(/\s+/);
  if (args.length > 8) throw Error('固件最多接收 8 个参数（含命令名）');
  if (!knownNames.includes(args[0])) throw Error('当前源码未注册此命令');
  return line;
}

export const plotChannels = {
  current: ['ia', 'ib', 'ic', 'v_bus'],
  speed: ['target_rpm', 'measured_rpm', 'ramp_rpm', 'error_rpm', 'iq_ref'],
  pos: ['mech_rad', 'elec_rad', 'ol_theta', 'Iq', 'Id'],
  sine: ['theta', 'sin', 'cos', 'Id', 'Iq'],
  dq: ['Id', 'Iq', 'Id_target', 'Iq_target'],
};

// 接收可能被 USB 任意拆包的 JustFloat 帧；其前后可以混有固件的文本输出。
export class SerialDecoder {
  constructor(onText, onFrame) {
    this.onText = onText; this.onFrame = onFrame;
    this.decoder = new TextDecoder(); this.buffer = new Uint8Array(); this.mode = null;
  }
  setMode(mode) {
    if (this.buffer.length) this.onText(this.decoder.decode(this.buffer));
    this.buffer = new Uint8Array(); this.decoder = new TextDecoder();
    this.mode = mode === 'sin' ? 'sine' : mode;
  }
  push(bytes) {
    if (!plotChannels[this.mode]) { this.onText(this.decoder.decode(bytes, {stream:true})); return; }
    const merged = new Uint8Array(this.buffer.length + bytes.length);
    merged.set(this.buffer); merged.set(bytes, this.buffer.length); this.buffer = merged;
    const payload = plotChannels[this.mode].length * 4;
    for (;;) {
      let tail = -1;
      for (let i = 0; i <= this.buffer.length - 4; i++) {
        if (this.buffer[i] === 0 && this.buffer[i+1] === 0 && this.buffer[i+2] === 128 && this.buffer[i+3] === 127) { tail=i; break; }
      }
      if (tail < 0) break;
      const start = tail - payload;
      if (start >= 0) {
        const view = new DataView(this.buffer.buffer, this.buffer.byteOffset + start, payload);
        const values = Array.from({length:payload/4}, (_,i)=>view.getFloat32(i*4,true));
        if (values.every(Number.isFinite)) {
          if (start) this.onText(this.decoder.decode(this.buffer.slice(0,start)));
          this.onFrame(this.mode, values);
        } else this.onText('[无法解码的 JustFloat 帧]\n');
      } else this.onText('[不完整的 JustFloat 帧]\n');
      this.buffer=this.buffer.slice(tail+4);
    }
    // 未收到波形时也交付完整 ASCII 行，避免参数/报错文本一直积压。
    let newline=-1;
    for (let i=0;i<this.buffer.length;i++) {
      const byte=this.buffer[i];
      if (!(byte===9 || byte===10 || byte===13 || byte>=32 && byte<=126)) break;
      // 始终保留一个完整负载及尾标的空间，避免把拆包中的浮点字节误认成 ASCII 行。
      if (byte===10 && i<this.buffer.length-payload-4) newline=i;
    }
    if (newline>=0) { this.onText(this.decoder.decode(this.buffer.slice(0,newline+1))); this.buffer=this.buffer.slice(newline+1); }
    if (this.buffer.length > 65536) {
      this.onText('[接收流未匹配所选波形协议，已丢弃过长数据]\n'); this.buffer=this.buffer.slice(-64);
    }
  }
}

export function numericDraft(value) {
  if (value==null || String(value).trim()==='' || !Number.isFinite(Number(value))) throw Error('请输入有限数值，不能留空或使用 NaN/Inf');
  return String(Number(value));
}
