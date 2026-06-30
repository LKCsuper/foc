% csdn 电流环整定
% https://blog.csdn.net/wanrenqi/article/details/131523107#:~:text=L%20%3D%200.00046%3B%20%25%E7%94%B5%E6%9C%BA%E7%9B%B8%E7%94%B5%E6%84%9F%20R%20%3D%200.62%3B%20%25%E7%94%B5%E6%9C%BA%E7%9B%B8%E7%94%B5%E9%98%BB,%3D%201%2F%28L%2As%20%2B%20R%29%3B%20Gc3%20%3D%20feedback%28G1%2AG2%2Aexp%28-Td%2As%29%2C1%29%3B%25%E6%8C%89%E5%B8%A6%E5%BB%B6%E8%BF%9F%E6%97%B6%E9%97%B4%E6%95%B4%E5%AE%9A%E7%9A%84PI%E5%8F%82%E6%95%B0%EF%BC%8C%E5%9C%A8%E5%B8%A6%E5%BB%B6%E8%BF%9F%E6%97%B6%E9%97%B4%E7%94%B5%E6%B5%81%E7%8E%AF%E4%B8%AD%E7%9A%84%E4%BC%AF%E5%BE%B7%E5%9B%BE%20bode%28Gc3%29

L = 0.00046; %电机相电感
R = 0.62; %电机相电阻
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
