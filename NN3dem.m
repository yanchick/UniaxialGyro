function [alp, nnet, time,tr]=NN3dem(k,n1,f0,f1,func,w,gl)
%=======================================================================
%Объект третьего порядка с нейронным регулятором
%Моделирование, две сети
%======================================================================
k1=k+1;%Количеств� � слоев задержки по состоянию
k2_1=0;%Количество слоев задержки по упраавлению
k3=3;%Порядок системы

[net3,time0,tr0]=NNcalc(k,n1,f0,f1,func,w,gl);
dt=0.01;
Tend=50;
x0=zeros(k3,1);
x=x0;
xz=x;
sqerr=[];err2=[];

ksi=0.7;
w0=1*2*pi;
H=0.12;
H0=H;
A=0.016;
B=0.0005;
h=0.05;
F=[h/A  H/A 0;
   -H/B   0 0;
   0     1 0;];
G=[1; 0 ;0];
C=[0 0 1];  
D=zeros(3,1);
w0=f0;w1=f1;ksi=0.85;
goal=sort([desir(w0,ksi) -1/w1]);
P=place(F,G,goal);


a=-1;
b=1;
InNet1=zeros(k1,1);
alpha=[0];
alphapr=0;
t1=0:dt:Tend;
u0=0;
xest=[0;0;0];xk=x0;
xwcap=[0;0;0];
Sqerr=[0;0;0];

for t=0:dt:Tend;
    w=1;
    tin1=InNet1;
    u=-P*xest;
    dx=F*x+G*u+G*w;   
    x=x+dx*dt;
    xz=[xz x];
    y=x(3)+0.0*deg2rad(1/60)*randn(1);
    
    tin1=[y; tin1];
    
    InNet1=tin1(1:k1);
    
    tin1=[];
    xest=sim(net3,[InNet1]);
    Sqerr=[Sqerr sqrt((xest-x).^2)];

    sqerr=[sqerr xest];   
    alphapr=alphapr+x(1)*dt;
    alpha=[alpha; rad2deg(alphapr)*60];
end
x0=1;
alp=alpha;
nnet=net3;
time=time0;
tr=tr0;
end

function [net, time,tr]=NNcalc(k1,n1,f0,f1,func,w0,gl)
%=======================================================================
%Объект третьего порядка с нейронным регулятором
%Расчет, одна сеть
%======================================================================
%clear
t=0:0.01:20*pi;
%w=0.1*randn(length(t),1)';
w=sin(2*pi*w0*t);
ksi=0.7;
w0=1*2*pi;
H=0.12;
A=7*0.016;
B=0.0005;
h=0.05;
%
F=[h/A  -H/A 0;
   H/B   0 0;
   0     1 0;];
G=[1; 0 ;0];
C=eye(3);  
D=zeros(3,1);
w0=f0;w1=f1;ksi=0.85;
goal=sort([desir(w0,ksi) -1/w1]);
P=place(F,G,goal);
Mysys=ss((F-G*P),G,C,D);
xf=lsim(Mysys,w,t);
x=[];
Q=[];

%=============================
%Первая подсистема - гироблок
%=============================
y_target=xf;    
y=xf(:,3)'+0.00*max(xf(:,3)')*rand(1,length(xf(:,3)'));

for j=0:k1
n=length(y);
z=[zeros(1,j) y(1:n-j)];
x=[x; z];
Q=[Q ;min(y) max(y)];
end
t=cputime;
fu={func 'purelin'};
net3=newff(Q,[n1 3],fu,'trainlm');
net3.trainParam.show = 2;
net3.trainParam.epochs = 25000;
net3.trainParam.goal = gl;
net3.performFcn = 'mse'
net3.trainParam.min_grad=1e-20;
[net3 tr]=train(net3,x,y_target');
time=cputime-t;
net=net3
end
