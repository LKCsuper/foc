"""Physical/control-principle illustrations; all numerical curves are educational."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Arc
from PIL import Image, ImageDraw

OUT = Path(__file__).resolve().parent
IMAGES = OUT / 'principles'
IMAGES.mkdir(exist_ok=True)
plt.rcParams.update({'font.sans-serif': ['Microsoft YaHei', 'SimHei', 'DejaVu Sans'],
                     'axes.unicode_minus': False, 'svg.fonttype': 'none'})
B, R, G = '#245b8a', '#c05060', '#308363'
GALLERY = []

def save(fig, name, title, explanation, sources):
    fig.suptitle(title, fontsize=20, color=B, weight='bold')
    fig.text(.025, .012, '原理示意：数值为教学参数；符号/分支对应当前代码，曲线不是硬件实测。', fontsize=10, color='#666')
    fig.tight_layout(rect=(0, .045, 1, .94))
    for ext in ['png', 'svg']:
        fig.savefig(IMAGES / (name+'.'+ext), dpi=170, facecolor='white')
    plt.close(fig)
    GALLERY.append((name, title, explanation, sources))

def vector(ax, p, q, label='', color=B):
    ax.annotate('', xy=q, xytext=p, arrowprops=dict(arrowstyle='-|>', lw=2, color=color))
    if label:
        ax.text(q[0]+.025, q[1]+.025, label, fontsize=12, color=color)

def axes_xy(ax, limit=1.3):
    ax.set_aspect('equal'); ax.set_xlim(-limit,limit); ax.set_ylim(-limit,limit)
    ax.axhline(0, color='#ddd'); ax.axvline(0,color='#ddd'); ax.grid(alpha=.15)

def block(ax, x,y,w,h,label):
    ax.add_patch(Rectangle((x,y),w,h,fc='#e4eff7',ec=B,lw=1.4))
    ax.text(x+w/2,y+h/2,label,ha='center',va='center',fontsize=12)

def sum_node(ax,x,y):
    ax.add_patch(Circle((x,y),.028,fc='white',ec=B,lw=1.4))
    ax.text(x,y,'Σ',ha='center',va='center',fontsize=14)

def arrow(ax,p,q): vector(ax,p,q)

# 1. Physical field, torque, and rotating coordinates.
fig,aa=plt.subplots(1,3,figsize=(17,6))
theta=.6; d=np.array([np.cos(theta),np.sin(theta)]); q=np.array([-np.sin(theta),np.cos(theta)])
for ax in aa: axes_xy(ax)
for ang,label in [(0,'A'),(2*np.pi/3,'B'),(4*np.pi/3,'C')]:
    tip=np.array([np.cos(ang),np.sin(ang)])
    vector(aa[0],(0,0),tip,label,color='#8ba7bd')
    aa[0].add_patch(Circle(tip*.85,.11,fill=False,ec=B,lw=2))
field=.85*(.35*d+.9*q);vector(aa[0],(0,0),field,'定子合成磁场',R)
aa[0].set_title('三相绕组轴相差 120°\n电流叠加生成一个空间矢量')
for tip,label,color in [(d,'转子磁链 / d',G),(q,'q',B),(field,'电流矢量',R)]:vector(aa[1],(0,0),tip,label,color)
aa[1].add_patch(Arc((0,0),.9,.9,theta1=np.degrees(theta),theta2=np.degrees(np.arctan2(field[1],field[0])),color=R,lw=2))
aa[1].set_title('Iq 是垂直于转子磁链的分量\n表贴 PMSM：T≈(3/2)p ψf Iq')
vector(aa[2],(0,0),d,'d',G);vector(aa[2],(0,0),q,'q',B)
vector(aa[2],(0,0),field,'I',R)
pd=np.dot(field,d)*d; pq=np.dot(field,q)*q
aa[2].plot([pd[0],field[0]],[pd[1],field[1]],'--',color=G)
aa[2].plot([pq[0],field[0]],[pq[1],field[1]],'--',color=B)
aa[2].set_title('Park 是向旋转轴投影\nId=Iαcosθ+Iβsinθ\nIq=−Iαsinθ+Iβcosθ')
save(fig,'P01_field_torque','三相磁场、转矩与 dq：FOC 为什么要变换坐标',
     'd 轴沿转子磁链，q 轴与其垂直。转矩公式是表贴 PMSM 的物理近似，不是代码中已有转矩估算函数。',
     ['App/motor/motor_foc.c','App/motor/motor_current.c'])

# 2. A real negative-feedback loop and implementation-order saturation.
fig,aa=plt.subplots(2,1,figsize=(14,10));ax=aa[0];ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off')
ax.text(.01,.65,'I*',fontsize=15);sum_node(ax,.13,.65);ax.text(.095,.70,'+',fontsize=13);ax.text(.105,.58,'−',fontsize=13)
arrow(ax,(.035,.65),(.10,.65));block(ax,.22,.56,.19,.18,'PI\nKp + Ki/s');arrow(ax,(.16,.65),(.22,.65))
block(ax,.49,.56,.19,.18,'电压限幅\n±Vlim / dq 圆');arrow(ax,(.41,.65),(.49,.65))
block(ax,.76,.56,.21,.18,'绕组电压→电流\n1/(Ls+R)');arrow(ax,(.68,.65),(.76,.65))
ax.plot([.90,.90,.13,.13],[.56,.28,.28,.62],color=B,lw=1.6)
ax.text(.46,.29,'反馈：实测电流 I',fontsize=12)
ax.plot([.58,.58,.32,.32],[.56,.43,.43,.56],color=R,lw=1.6)
ax.text(.35,.44,'饱和时积分回写 z=Vlimited−Kp·e',fontsize=12,color=R)
ax.set_title('d、q 轴各用独立 PI；误差通过电压改变电流，实测电流返回负端')
dt=.0005;time=np.arange(0,.25,dt);meas=0.;z=0.;ys=[];zs=[];vs=[]
for tt in time:
    ref=4. if tt<.10 else 0.;e=ref-meas;z+=20*e*dt;v=.7*e+z
    limited=np.clip(v,-2,2)
    if limited!=v:z=limited-.7*e
    meas+=(limited-.8*meas)/.02*dt
    ys.append(meas);zs.append(z);vs.append(limited)
aa[1].plot(time,np.where(time<.10,4,0),'--',label='I* (A)');aa[1].plot(time,ys,label='I (A)')
aa[1].plot(time,vs,label='V limited (V)');aa[1].plot(time,zs,label='积分 z (V)')
aa[1].axvspan(0,.10,alpha=.08,color=R);aa[1].set(xlabel='时间 (s)',ylabel='A / V')
aa[1].legend(ncol=4);aa[1].grid(alpha=.2);aa[1].set_title('教学 RL 模型：目标超出电压能力时不持续积累积分，目标下降后能够退出饱和')
save(fig,'P02_current_feedback','电流 PI 的负反馈和抗饱和原理',
     '波形使用自选 RL 教学模型演示代码的单轴 clamping。dq 合成圆限幅仍须在单轴限幅后回写两轴积分。',
     ['App/motor/motor_pid.c','App/motor/motor_current.c'])

# 3. PWM voltage synthesis with switches and time allocation.
fig,aa=plt.subplots(1,3,figsize=(17,6));ax=aa[0];ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off')
ax.plot([.10,.90],[.90,.90],color=R,lw=2);ax.plot([.10,.90],[.10,.10],color=B,lw=2)
ax.text(.03,.90,'+Vbus');ax.text(.03,.08,'0 V')
for x,label in [(.25,'A'),(.50,'B'),(.75,'C')]:
    ax.plot([x,x],[.90,.73],color=B);ax.plot([x,x],[.61,.39],color=B);ax.plot([x,x],[.27,.10],color=B)
    ax.plot([x-.035,x+.035],[.64,.73],color=R,lw=2);ax.plot([x-.035,x+.035],[.30,.39],color=R,lw=2)
    ax.plot([x,x+.10],[.50,.50],color=B);ax.text(x+.105,.50,label,va='center')
ax.text(.13,.01,'桥臂互补开关需死区；这里画理想开关');ax.set_title('直流母线经三相桥臂\n得到离散开关电压')
ax=aa[1];axes_xy(ax,.9);a=np.arange(7)*np.pi/3;r=2/3
ax.plot(r*np.cos(a),r*np.sin(a),'-o',color=B)
for j in range(6):vector(ax,(0,0),(r*np.cos(a[j]),r*np.sin(a[j])),f'V{j+1}')
u=np.array([.38,.22]);vector(ax,(0,0),u,'Vref',R)
ax.set_title('每个 PWM 周期内平均\nVref=T1/T·V1+T2/T·V2')
ax=aa[2];edges=np.array([0,.15,.35,.55,.70,.90,1]);levels=np.array([0,1,2,0,2,1,0])
ax.step(edges,levels,where='post',lw=2,color=B)
ax.set(xlim=(0,1),ylim=(-.35,2.8),xlabel='一个 PWM 周期 / T',yticks=[0,1,2],yticklabels=['零矢量','V1','V2'])
ax.grid(alpha=.2);ax.set_title('零矢量 + 相邻有效矢量\nT0+T1+T2=T（时序示意）')
ax.text(.02,2.5,'改变时间比例 → 改变平均电压',fontsize=11)
save(fig,'P03_bridge_svpwm','逆变桥与 SVPWM：用开关时间合成连续电压矢量',
     '开关序列用于解释平均原理，并非本板所有计数侧的实际门极时序。代码归一化为 1.5×Vαβ/Vbus，板级 A-C-B 和极性按原实现保留。',
     ['App/motor/motor_foc.c','App/motor/motor_output.c','Library/Hal/port/stm32f4/hal_motor_stm32f4.c'])

# 4. V/F versus I/F fields and electrical/mechanical angle.
fig,aa=plt.subplots(1,3,figsize=(17,6))
tt=np.linspace(0,2,500);rpm=np.minimum(300*tt,300);omega=rpm*2*np.pi/60*7;theta=np.cumsum(omega)*(tt[1]-tt[0])
aa[0].plot(tt,rpm,label='斜坡场 RPM');aa[0].set(xlabel='时间 (s)',ylabel='RPM');aa[0].grid(alpha=.2);aa[0].legend()
aa[0].set_title('I/F：先规定旋转磁场速度\nθol=∫(RPM·2π/60·p)dt')
axes_xy(aa[1]);vector(aa[1],(0,0),(.8,0),'定子场角',B);vector(aa[1],(0,0),(.6,-.4),'实际转子磁链',G)
aa[1].add_patch(Arc((0,0),.7,.7,theta1=-34,theta2=0,color=R,lw=2))
aa[1].text(-1.1,-1.1,'负载角变化；开环场角≠真实转子角',fontsize=11)
aa[1].set_title('转子受转矩跟随定子场\n负载过大时可能失步')
aa[2].plot(tt,np.full_like(tt,1.5),label='快速 V/F：Vq=boost (V)')
aa[2].plot(tt,np.minimum(tt/.25,1)*.6,label='I/F 拖转：Iq 示意 (A)')
aa[2].set(xlabel='时间 (s)',ylabel='V / A');aa[2].grid(alpha=.2);aa[2].legend();aa[2].set_title('快速 V/F 直接规定电压\nI/F 用电流 PI 维持指定电流')
save(fig,'P04_openloop_field','开环为什么能拖转：旋转磁场与转子跟随',
     '本仓库快速 V/F 的 Vq 为限幅 boost，不是经典 V∝f。I/F 时 dq 相对于强制场角，不能假定已对准转子磁链；速度和电流曲线仅示意。',
     ['App/motor/diagnostic/motor_fast_vf.c','App/motor/motor_control.c','App/motor/start/motor_if_start.c'])

# 5. Sensor observability and resolution.
fig,aa=plt.subplots(1,3,figsize=(17,6));ang=np.linspace(0,2*np.pi,700)
aa[0].plot(ang,ang,label='真实连续角');aa[0].step(ang,np.floor(ang/(np.pi/3))*np.pi/3,where='post',label='霍尔扇区量化')
aa[0].set(xlabel='电角度 (rad)',ylabel='估计角 (rad)');aa[0].legend();aa[0].grid(alpha=.2);aa[0].set_title('霍尔每电周期只有 6 个扇区\n边沿之间角度依靠预测/插值')
ax=aa[1];ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off');sum_node(ax,.15,.60)
ax.text(.01,.66,'编码器角');arrow(ax,(.01,.60),(.12,.60));block(ax,.27,.50,.22,.20,'PLL 校正\nKp / Ki');arrow(ax,(.18,.60),(.27,.60))
block(ax,.61,.50,.30,.20,'速度积分成角度\n预测 + 测量校正');arrow(ax,(.49,.60),(.61,.60))
ax.plot([.80,.80,.15,.15],[.50,.30,.30,.57],color=B);ax.text(.30,.31,'预测角度回到负端');ax.set_title('PLL 兼顾平滑速度与连续角\n无新帧按速度预测，新帧纠偏')
i=np.linspace(0,.5,400);aa[2].plot(i,i,label='理想');aa[2].step(i,np.round(i/.0806)*.0806,label='ADC 量化')
aa[2].axvspan(.10,.20,color=R,alpha=.12);aa[2].set(xlabel='真实电流 (A)',ylabel='可读电流 (A)')
aa[2].legend();aa[2].grid(alpha=.2);aa[2].set_title('BM75≈0.0806 A/count\n低电流只有少量 ADC 计数')
save(fig,'P05_sensor_resolution','传感器原理：离散测量如何得到连续反馈',
     '霍尔插值、编码器 PLL 和 ADC 量化有不同的信息限制。PLL 不能凭空恢复过期或错误原始帧；反馈有效性须同时检查。',
     ['App/motor_sensor/motor_encoder.c','App/motor_sensor/motor_hall_angle.c','App/motor/motor_sense.c'])

# 6. Outer-loop mechanics and impedance.
fig,aa=plt.subplots(2,1,figsize=(15,10));ax=aa[0];ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off')
ax.text(.02,.67,'ω*');sum_node(ax,.14,.65);arrow(ax,(.04,.65),(.11,.65));block(ax,.24,.55,.19,.20,'速度 PI\n输出 Iq*');arrow(ax,(.17,.65),(.24,.65))
block(ax,.51,.55,.19,.20,'电流控制\nIq → 转矩 T');arrow(ax,(.43,.65),(.51,.65));block(ax,.77,.55,.20,.20,'机械系统\nJ·ω点=T−TL−Bω');arrow(ax,(.70,.65),(.77,.65))
ax.plot([.88,.88,.14,.14],[.55,.27,.27,.62],color=B);ax.text(.45,.30,'机械速度反馈');ax.text(.10,.59,'−');ax.set_title('速度环：转矩改变加速度，速度反馈修正转矩需求')
ax=aa[1];ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off')
ax.add_patch(Rectangle((.05,.30),.08,.30,fc='#aaa'))
x=np.linspace(.13,.65,15);y=.45+.055*np.array([0]+[(-1)**j for j in range(13)]+[0]);ax.plot(x,y,color=G,lw=2)
ax.add_patch(Rectangle((.65,.33),.16,.24,fc='#e4eff7',ec=B));ax.text(.73,.45,'关节',ha='center')
vector(ax,(.83,.45),(.97,.45),'u',R);ax.text(.18,.70,'虚拟弹簧：Kp(p*−p) + 虚拟阻尼：Kd(v*−v)',fontsize=14)
ax.text(.18,.12,'u=Kp·位置误差+Kd·速度误差+前馈+条件积分\n应用层把 u 映射成 Vq 或 Iq，滞回选择低电流后端。',fontsize=13)
ax.set_title('关节阻抗：把软件增益解释成弹性与阻尼')
save(fig,'P06_outerloop_impedance','速度、位置与关节外环的机械原理',
     '位置 P 产生速度参考；当前近目标电压保持有专用速度 PI，不能把全部位置行为画成通用三环。关节纯算法的输出单位由后端配置决定。',
     ['App/motor/motor_speed.c','App/motor/motor_position_control.c','App/motor/joint/motor_joint_control.c'])

# 7. Braking distance and hysteresis boundaries.
fig,aa=plt.subplots(1,3,figsize=(17,6));distance=np.linspace(0,1,500);a=2.;v=np.minimum(1,np.sqrt(2*a*distance))
aa[0].plot(distance,v,lw=2);aa[0].fill_between(distance,0,v,alpha=.12)
aa[0].set(xlabel='剩余距离 |Δx|',ylabel='允许速度');aa[0].set_title('制动距离 d=v²/(2a)\nv允许=min(vmax,√(2a|Δx|))');aa[0].grid(alpha=.2)
axes_xy(aa[1]);p=np.linspace(-1,1,300);aa[1].plot(p,np.sqrt(np.maximum(0,1-p*p)),color=B);aa[1].plot(p,-np.sqrt(np.maximum(0,1-p*p)),color=B)
vector(aa[1],(0,0),(-.45,.75),'负 Id + 转矩 Iq',R)
aa[1].set(xlabel='Id / Ibudget',ylabel='Iq / Ibudget');aa[1].set_title('弱磁负 Id 抵消部分永磁磁链\n|Id|≤√(Ibudget²−Iq²)')
aa[2].plot([0,.25,.25,.4],[0,0,1,1],lw=2,label='需求上升')
aa[2].plot([.4,.20,.20,0],[1,1,0,0],lw=2,label='需求下降')
aa[2].set(xlabel='|Iq需求| (A)',ylabel='后端',yticks=[0,1],yticklabels=['Vq','dq PI']);aa[2].legend();aa[2].grid(alpha=.2)
aa[2].set_title('0.25 A 进入、0.20 A 退出\n滞回避免阈值附近反复切换')
save(fig,'P07_limits_hysteresis','轨迹制动、弱磁电流预算与后端滞回原理',
     '制动速度上限由运动学推导；弱磁消耗电流圆的 d 轴余量；关节后端阈值依据需求而非单拍实测电流。',
     ['App/motor/motor_trajectory.c','App/motor/motor_fw.c','App/motor/joint/motor_joint.c'])

# 8. Observer physical equation and innovation.
fig,aa=plt.subplots(1,3,figsize=(17,6));ax=aa[0];axes_xy(ax,1.3)
psi=np.array([.75,.30]);li=np.array([-.1,.55]);vector(ax,(0,0),psi,'ψrotor',G);vector(ax,psi,psi+li,'L·i',B);vector(ax,(0,0),psi+li,'x=ψstator',R)
ax.set_title('电压方程积分：x≈∫(v−Ri)dt\n转子磁链 ψr=x−Li\n角度=atan2(ψβ,ψα)')
ax=aa[1];ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off');block(ax,.05,.60,.35,.20,'输入 v 和模型 R/L\n预测电流 î')
block(ax,.60,.60,.35,.20,'传感器电流 i');sum_node(ax,.50,.42)
arrow(ax,(.225,.60),(.475,.42));arrow(ax,(.775,.60),(.525,.42));block(ax,.28,.08,.44,.20,'残差 i−î\n修正反电势 / 状态估计');arrow(ax,(.50,.39),(.50,.28))
ax.set_title('观测器：用预测与测量的差\n修正不可直接测量的量')
ax=aa[2];axes_xy(ax);ph=np.linspace(0,2*np.pi,300);ax.plot(np.cos(ph),np.sin(ph),color='#bbb')
ax.scatter([1.12,.92],[.3,.12],c=[R,B]);vector(ax,(0,0),(1.12,.3),'估计 ψr',R);vector(ax,(1.12,.3),(.96,.26),'过大时拉回',G)
ax.set_title('Ortega 用磁链模长约束抑制漂移\n主 sensorless：ε=min(ψf²−|ψr|²,0)')
save(fig,'P08_observer_physics','无感观测的物理来源：电压、电流与磁链',
     '主 sensorless 用单边 Ortega 修正和 PLL；独立 flux 为双向修正。Luenberger 连续残差校正、SMO 平滑非线性残差注入、EKF 按协方差权衡创新，均不能凭模块存在认定默认生效。',
     ['App/motor/observer/motor_sensorless.c','App/motor/observer/motor_flux.c','App/motor/observer/motor_luenberger.c','App/motor/observer/motor_smo.c','App/motor/observer/motor_ekf.c'])

# 9. HFI inductance-dependent response.
fig,aa=plt.subplots(1,2,figsize=(14,6));ang=np.linspace(0,np.pi,500);L=.0004+.00012*np.cos(2*(ang-.65));response=1/L
aa[0].plot(np.degrees(ang),response/response.max(),lw=2);aa[0].set(xlabel='试探电角轴 (deg)',ylabel='归一化 Δi 响应');aa[0].grid(alpha=.2)
aa[0].set_title('相同电压脉冲：Δi≈V·Δt/L(θ)\n不同试探轴电感不同，响应也不同')
t=np.arange(20);inj=(-1.)**t;iq_delta=.08*inj
aa[1].step(t,inj,where='mid',label='交替 ±Vd（归一化）');aa[1].plot(t,iq_delta,'o-',label='电流响应示意')
aa[1].set(xlabel='控制拍',ylabel='相对幅值');aa[1].legend();aa[1].grid(alpha=.2);aa[1].set_title('利用注入极性同步解调\n移除符号交替，得到角度纠偏信号')
save(fig,'P09_hfi_saliency','HFI 为什么能在低速估角：凸极电感与脉冲响应',
     '电感曲线用自选凸极模型说明原理。代码扫描最大响应轴，再由正负极性差补 π；不保证任意电机都有足够可辨凸极信号。',
     ['App/motor/start/motor_hfi_start.c','App/motor/observer/motor_hfi.c'])

# 10. BLDC current path and integral event.
fig,aa=plt.subplots(1,2,figsize=(15,6));ax=aa[0];ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off')
ax.plot([.10,.90],[.85,.85],color=R,lw=2);ax.text(.02,.88,'+Vbus')
ax.plot([.10,.90],[.15,.15],color=B,lw=2);ax.text(.02,.08,'0 V')
for x,label in [(.25,'A'),(.50,'B'),(.75,'C')]:
    ax.plot([x,x],[.85,.61],color='#aaa');ax.plot([x,x],[.39,.15],color='#aaa');ax.text(x,.53,label,ha='center')
ax.plot([.25,.25,.50,.50],[.85,.50,.50,.15],color=R,lw=4,alpha=.6)
vector(ax,(.28,.50),(.46,.50),'电流',R);ax.text(.60,.60,'C 悬空\n测 Vfloat',fontsize=13,color=G)
ax.text(.14,.02,'step 0：A=PWM，B=LOW，C=FLOAT',fontsize=12);ax.set_title('两相形成驱动电流回路\n第三相不驱动，观察反电势')
t=np.linspace(0,1,400);e=2*(t-.35);dt=t[1]-t[0];area=np.cumsum(np.maximum(e-.10,0))*dt
aa[1].plot(t,e,label='按步极性修正 BEMF');aa[1].fill_between(t,.10,e,where=e>.10,color=G,alpha=.2,label='扣噪声后正面积')
aa[1].plot(t,area,label='累计面积');aa[1].axhline(.22,color=R,ls='--',label='面积阈值（示意）')
aa[1].set(xlabel='归一化扇区时间',ylabel='示意幅值');aa[1].legend();aa[1].grid(alpha=.2);aa[1].set_title('Vn=(Va+Vb+Vc)/3\ne=Vfloat−Vn，面积达阈值换相')
save(fig,'P10_bldc_circuit','BLDC 原理：两相通电、悬空相观察与积分换相',
     '高亮回路为 step 0 的等效导通路径，未画 MOS 死区/续流细节。实际算法还包含消隐、有效采样窗、最小周期和失锁限制。',
     ['App/motor/bldc/motor_bldc_six_step.c','App/motor/bldc/motor_bldc_sensorless.c'])

# 11. Identification from characteristic slopes.
fig,aa=plt.subplots(1,3,figsize=(17,6));i=np.linspace(.2,1,8);voltage=.7*i+.12
aa[0].scatter(i,voltage,c=R);aa[0].plot(i,voltage,c=B);aa[0].set(xlabel='I (A)',ylabel='V (V)');aa[0].grid(alpha=.2)
aa[0].set_title('R：稳态 di/dt≈0\n拟合 V=R·I+offset，斜率是 R')
t=np.linspace(0,.002,300);current=(2/.7)*(1-np.exp(-.7*t/.001));aa[1].plot(t*1000,current,lw=2)
aa[1].plot(t[:60]*1000,2000*t[:60],'--',label='初始斜率 V/L');aa[1].set(xlabel='脉冲时间 (ms)',ylabel='I (A)');aa[1].legend();aa[1].grid(alpha=.2)
aa[1].set_title('L：瞬态 V=Ri+L·di/dt\nL=(V有效−R·I中值)/(di/dt)')
w=np.linspace(0,250,300);aa[2].plot(w,.008*w,lw=2);aa[2].set(xlabel='电角速度 |ωe| (rad/s)',ylabel='反电势幅值 (V)');aa[2].grid(alpha=.2)
aa[2].set_title('磁链：关桥滑行时 e≈ωe·ψf\nψf≈|e|/|ωe|')
save(fig,'P11_identification','R、L、磁链为什么可以由电压和电流检测',
     '拟合/脉冲/滑行是不同物理工况。曲线为理想模型，实际代码有比例校正、窗口和样本有效性检查，检测结果仅 RAM。',
     ['App/motor_param/motor_detect.c'])

# 12. Debounce and atomic commit principle rather than software call sequence.
fig,aa=plt.subplots(1,2,figsize=(15,6));t=np.linspace(0,1,500);condition=((t>.1)&(t<.18))|((t>.4)&(t<.8));timer=[];z=0;latched=False;fault=[]
for c in condition:
    z=z+t[1]-t[0] if c else 0.;timer.append(z)
    latched=latched or z>=.15;fault.append(int(latched))
aa[0].plot(t,condition.astype(float)*.08,label='异常条件×0.08');aa[0].plot(t,timer,label='连续异常计时');aa[0].plot(t,np.array(fault)*.25,label='故障锁存×0.25')
aa[0].axhline(.15,color=R,ls='--',label='持续时间门槛');aa[0].grid(alpha=.2);aa[0].legend();aa[0].set(xlabel='时间 (s)',ylabel='示意量')
aa[0].set_title('短脉冲解除后计时清零\n持续异常触发；条件解除不自动清锁存')
ax=aa[1];ax.set(xlim=(0,1),ylim=(0,1));ax.axis('off')
for y,label,col in [(.70,'槽 A：旧数据 + 已提交 + CRC 有效',G),(.40,'槽 B：新主体 + CRC → 最后 committed',B)]:
    ax.add_patch(Rectangle((.05,y),.90,.16,fc='#e4eff7',ec=col,lw=2));ax.text(.50,y+.08,label,ha='center',fontsize=12)
ax.text(.10,.20,'提交前掉电：新槽无效 → 仍能读取旧槽\n提交并校验成功：按序号选择新槽',fontsize=13)
ax.set_title('双槽保留上一份有效数据\n提交标记表达“完整写入”的边界')
save(fig,'P12_debounce_commit','时间去抖与双槽提交：保护和存储的可靠性原理',
     '软件保护示意门槛不是当前配置值；硬件紧急关断不由软件去抖覆盖。双槽图说明最后写提交标记和回读 CRC 的目的。',
     ['App/motor/motor_protect.c','App/motor_param/flash_storage.c'])

lines=['# 先看原理：磁场、电路、反馈、波形与数学关系','',
       '本组专门解释机制。旧目录 images/ 为代码结构与流程辅助图；本目录 principles/ 为物理和控制原理图。',
       '', '重新生成：`python draw_principles.py`（Python、numpy、matplotlib、Pillow）。', '',
       '具体示意模型的参数写在脚本中；不将教学曲线当成上板记录。板级极性和算法特殊分支说明以实际代码为准。']
for name,title,explanation,sources in GALLERY:
    lines+=['','## '+title,'','!['+title+'](principles/'+name+'.png)','',explanation,'',
            '对应源码：'+'、'.join('['+s+'](../../'+s+')' for s in sources)]
(OUT/'原理图阅读.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
tiles=[]
for name,*_ in GALLERY:
    image=Image.open(IMAGES/(name+'.png')).convert('RGB');image.thumbnail((600,380))
    tile=Image.new('RGB',(620,410),'white');tile.paste(image,((620-image.width)//2,0))
    ImageDraw.Draw(tile).text((10,385),name,fill='black');tiles.append(tile)
sheet=Image.new('RGB',(1240,410*((len(tiles)+1)//2)),'#dde3e9')
for n,tile in enumerate(tiles):sheet.paste(tile,((n%2)*620,(n//2)*410))
sheet.save(OUT/'principle_contact_sheet.jpg')
print(f'Generated {len(GALLERY)} principle plates, PNG + SVG.')
