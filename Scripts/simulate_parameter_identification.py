"""参数辨识方法级仿真；不调用固件、不输出可直接刷入的真实电机参数。"""
import argparse, cmath, csv, json, math, random, statistics
from pathlib import Path

def line_fit(xs, ys):
    xm,ym=statistics.mean(xs),statistics.mean(ys)
    den=sum((x-xm)**2 for x in xs)
    if den<=1e-12: raise ValueError('Insufficient excitation')
    slope=sum((x-xm)*(y-ym) for x,y in zip(xs,ys))/den
    return slope,ym-slope*xm

def resistance(rng, gain=1.0, nonlinear=0.0):
    # 正负各三个电流平台；固定死区项和ADC偏置作为干扰。
    xs=[];ys=[]
    for sign in (-1,1):
        for i in (.5,.75,1.0):
            true_i=sign*i
            xs.append(gain*true_i+.04+rng.gauss(0,.003))
            ys.append(3.75*true_i+.3*sign+.08+nonlinear*true_i+rng.gauss(0,.008))
    slopes=[line_fit(xs[k:k+3],ys[k:k+3])[0] for k in (0,3)]
    # 不能把正负点直接放进只有截距的直线；死区项随符号变化。
    mixed=line_fit(xs,ys)[0]
    return sum(slopes)/2, mixed, statistics.mean(abs(y/x) for x,y in zip(xs,ys))

def inductance(rng, freq, actual_voltage=True, current_delay=0.0):
    # 已知连续RL模型的正弦稳态响应，按15kHz采样，再用同步DFT辨识。
    # 物理响应用实际端电压，指令电压有0.2V幅值误差。
    r,l,fs=3.75,.0013,15000
    physical,command=1.0,1.2
    z=complex(r,2*math.pi*freq*l)
    current=physical/z
    n=1500; v=0j;i=0j
    for k in range(n):
        t=k/fs; phase=2*math.pi*freq*t
        vm=(physical if actual_voltage else command)*math.cos(phase)+rng.gauss(0,.008)
        im=(current*cmath.exp(1j*2*math.pi*freq*(t-current_delay))).real+.04+rng.gauss(0,.003)
        basis=cmath.exp(-1j*phase)
        v+=vm*basis;i+=im*basis
    return (v/i).imag/(2*math.pi*freq)

def flux(rng, gain=1.0, commanded=False):
    actual=[500,800,1100,1400]
    xs=[];ys=[]
    for rpm in actual:
        omega=rpm*7*2*math.pi/60
        e=.0035*omega+.1+rng.gauss(0,.012)
        # 命令转速误当实际转速：模拟实际只达到命令的85%。
        xs.append(omega*(1/.85 if commanded else 1)*gain)
        ys.append(e)
    slope,_=line_fit(xs,ys)
    return slope,ys[0]/xs[0]

def summary(values,truth):
    errors=sorted(100*(v/truth-1) for v in values)
    return {'mean':statistics.mean(values),'bias_percent':statistics.mean(errors),
            'p05_percent':errors[int(.05*(len(errors)-1))],
            'p95_percent':errors[int(.95*(len(errors)-1))]}

def run(out, trials=200):
    rng=random.Random(2804); result={}
    for name,gain,nonlinear in [('R_constant_offsets',1,0),('R_current_gain_5pct',1.05,0),('R_extra_series_0p2ohm',1,.2)]:
        samples=[resistance(rng,gain,nonlinear) for _ in range(trials)]
        result[name]={method:summary([v[k] for v in samples],3.75) for k,method in enumerate(['separate_polarity_fit','mixed_polarity_fit','direct_V_over_I'])}
    for name,actual,delay in [('L_actual_voltage',True,0),('L_command_voltage',False,0),('L_current_delay_20us',True,.00002)]:
        result[name]={str(f):summary([inductance(rng,f,actual,delay) for _ in range(30)],.0013) for f in (500,1000,1500)}
    for name,gain,command in [('flux_actual_speed',1,False),('flux_command_speed',1,True),('flux_speed_gain_5pct',1.05,False)]:
        samples=[flux(rng,gain,command) for _ in range(trials)]
        result[name]={method:summary([v[k] for v in samples],.0035) for k,method in enumerate(['multi_speed_with_intercept','single_low_speed'])}
    out.mkdir(parents=True,exist_ok=True)
    (out/'parameter_identification_sim.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    with (out/'parameter_identification_sim.csv').open('w',newline='',encoding='utf8') as f:
        w=csv.writer(f);w.writerow(['scenario','method_or_frequency','mean','bias_percent','p05_percent','p95_percent'])
        for scenario,methods in result.items():
            for method,data in methods.items():w.writerow([scenario,method,*data.values()])
    print(json.dumps(result,indent=2))
    return result
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--out',default='Doc/verify/parameter_identification_sim_2026-10-11')
    run(Path(ap.parse_args().out))
