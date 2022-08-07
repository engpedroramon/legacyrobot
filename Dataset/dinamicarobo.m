function dxdt =dinamicarobo(t,x,flength,Mil,Mir)

m=1; %Massa
r=0.2; %Raio da roda
d=0.1; %Distancia para o centro de massa
bm=2; %Atrito do eixo do motor
Ia=1; %Inercia do robô em relação ao centro de massa
Io=0.1; %Momento de inercia do rotor e roda

Mil=interp1(flength,Mil,t);
Mir=interp1(flength,Mir,t);

Ml=Mil;
Mr=Mir;

K=0.5; %Constante de atrito viscoso com o solo

A=m*r^2/4+(Ia+m*d^2)*r^2/bm^2+Io;
B=m*r^2/4+(Ia+m*d^2)*r^2/bm^2;
C=(B-A^2/B);

dx1=x(2);
dx3=x(4);

dx2=(-(A/B)*Mr+Ml+K*((A/B)*dx1-dx3))/C;
dx4=(-(A/B)*Ml+Mr+K*((A/B)*dx3-dx1))/C;


dxdt=[dx1,dx2,dx3,dx4]';

%Entradas -> Torque na roda esquerda (Mil) e direita (Mir)
%Saídas -> velocidades e posição angular nas rodas direita e esquerda

%Modelo baseado no artigo:
%MODELLING OF MOBILE ROBOT DYNAMICS
%Edouard Ivanjko1, Toni Petrini, Ivan Petrovi