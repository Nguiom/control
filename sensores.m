close all
clear
clc

G=(0.14163.*0.005+0.013.*0.0445).*9.7774;
I=0.1505e-3;
Beta=0.112708.*0.9e-3;
Gamma=0.375;
M=0.003025734446711853;
t=0:0.0025:6;

A=[0,1;(-G)./I,-Beta./I];
B=[0;(M.*0.14163)./I];
C=[1,0];
K=[5.7636,5.3824];
L=[79.326;2362.946];
A1=[A-B*K,B*K;zeros(size(A)),A-L*C];
B1=[zeros(size(B));zeros(size(B))];
C1=[C,zeros(size(C))];
x0=[-(pi().*20)./180,0];
x1=[-(pi().*20)./180,0,0,0];

simulado=ss(A,B,C);
simulado1=ss(A1,B1,C1);
[y,tSim,x]=lsim(simulado,zeros(1,length(t)),t,x0);
[y1,tSim1,x1]=lsim(simulado1,zeros(1,length(t)),t,x1);

figure(1)
hold on
plot(tSim1,y1.*(180./pi()),";calidad;")

hold on
plot(tSim,y.*(180./pi()),";normal;")

hold off


figure(2)
hold on
plot(tSim1,x1(:,2).*(180./pi()),";calida velocidad;")

hold on
plot(tSim,x(:,2).*(180./pi()),";normal;")

hold off


