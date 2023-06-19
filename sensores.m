close all
clear
clc

G=(0.14163.*0.005+0.013.*0.0445).*9.7774;
I=0.1505e-3;
Beta=0.112708.*0.9e-3;
Gamma=0.375;
M=0.003025734446711853;
t=0:0.1:120;

A=[0,1;(-G)./I,-Beta./I];
B=[0;(M.*0.14163)./I];
C=[1,0];

simulado=ss(A,B,C);
simulado=c2d(simulado,0.1,'zoh');



x0=[-(pi().*20)./180,0];
x1=[-(pi().*20)./180,0,0,0];

po=obsv(simulado);
pc=ctrb(simulado);

wnc=4./(0.8.*120);
wno=4./(0.8.*24);

dc=simulado.A^2+(0.8.*wnc.*2).*simulado.A+(wnc.^2).*eye(size(simulado.A));
dob=simulado.A^2+(0.8.*wno.*2).*simulado.A+(wno.^2).*eye(size(simulado.A));

k=[0,1]*inv(pc)*dc;
l=dob*inv(po)*[0;1];

k(1)=k(1).*1.5;
k(2)=k(2);
k
l

A1=[simulado.A-simulado.B*k,simulado.B*k;zeros(size(simulado.A)),simulado.A-l*simulado.C];
B1=[zeros(size(simulado.B));zeros(size(simulado.B))];
C1=[simulado.C,zeros(size(simulado.C))];

controlador=ss(A1,B1,C1);

[y,tSim,x]=lsim(simulado,zeros(1,length(t)),t,x0);
[yc,tSimc,xc]=lsim(controlador,zeros(1,length(t)),t,x1);

figure(1)
hold on
plot(tSimc,yc.*(180./pi()),';control;')
hold off

figure(2)
hold on
plot(tSimc,x(:,2).*(180./pi()),';control;')
hold off


