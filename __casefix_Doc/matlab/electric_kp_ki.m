L = 0.00046; %电机相电感
R = 0.4; %电机相电阻
f = 400;  %需要设计的电流环带宽
Td = 100e-6;%电流环的延迟时间
Kp = L*f*2*pi;%理想情况下没有延迟时间整定的Kp
Ki = R*f*2*pi;%理想情况下没有延迟时间整定的Ki
s=tf('s');
G1 = Kp + Ki/s; %PI控制器传函
G2 = 1/(L*s + R);%电机传函
Gc = feedback(G1*G2,1);%电流闭环系统
bode(Gc) %按理想不带延迟时间整定的PI参数，在理想不带延迟时间电流环中的伯德图
hold on
Gc2 = feedback(G1*G2*exp(-Td*s),1);
bode(Gc2) %按理想不带延迟时间整定的PI参数，在带延迟时间电流环中的伯德图
hold on
b = f*2*pi*Td; 
a = b*(sqrt(sin(b)*sin(b)+1)-sin(b));
Kp = L*a/Td;
Ki = R*a/Td;
G1 = Kp + Ki/s;
G2 = 1/(L*s + R);
Gc3 = feedback(G1*G2*exp(-Td*s),1);%按带延迟时间整定的PI参数，在带延迟时间电流环中的伯德图
bode(Gc3)
