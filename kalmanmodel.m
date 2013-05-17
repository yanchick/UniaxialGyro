%=======================================================================
%Объект третьего порядка с нейронным регулятором
%Моделирование, две сети
%======================================================================
t=load('rd.mat');
global A8 C8out B8 Q W G1 P;
A = [              0                       0                     0                   0                         1                            0          -1                          0;  
                    0                       0                     0                   0                         0                            1           0                         -1;                 
                    1                       0                     0                   0                         0                            0           0                          0;  
                    0                       1                     0                   0                         0                            0           0                          0;                 
                    0                   t.kd*t.w0                 0                   0                    -t.kd/(t.B_+t.b)  (t.A_+2*t.b)*t.w0/(t.B_+t.b)                   0   t.w0*(t.A_+t.a+t.b-t.c)/(t.B_+t.b);
              t.kd*t.w0                     0                     0                   0       -(t.A_+2*t.b)*t.w0/(t.B_+t.b)             -t.kd/(t.B_+t.b)     -t.w0*(t.A_+t.a+t.b-t.c)/(t.B_+t.b)     0;      
                    0                       0                     0                   0                        0                             0     -t.h0/t.A0                        0; 
                    0                       0                     0                   0                        0                             0          0                       -t.h0/t.B0  ];
A(:,3:4)=[];
A(3:4,:)=[];
niz=2;
B=[zeros(4,2);eye(2)];
C=[eye(2) zeros(2,4)] ;
D=[zeros(2,2) eye(2)];
Do=zeros(2,6);
Gs=ss(A,B,C,0,'outputnames',{'Alpha','Beta'},'inputnames',{'Mom1','Mom2'});% Simulink

w0=55;w2=40;w3=45;ksi=0.9;
goal1=sort([desir(w0,ksi) desir(w2,ksi) desir(w3,ksi)]);
P=place(A,B,goal1);

Ma=0.4;
Umax=0.001*pi/(180*60);
Q=[Umax 0;0 Umax];
W=[Ma 0;0 Ma];

Sig=zeros(6);

dt0=1e-5;
Sigp=[];
for t=0:dt0:0.25
   dSig=A*Sig+Sig*transpose(A)-Sig*transpose(C)*inv(Q)*C*Sig+B*W*transpose(B);  
   Sig=Sig+dSig*dt0;
   Sigp=[Sigp Sig(1,1)];
end
K=Sig*transpose(C)*inv(Q);


x=zeros(6,1);ex=zeros(6,1);
i=1;
OutX=[];OutEx=[];q=[];West=[];Sig_est=Sig;Wvar=W;
Jmp=5;xlast=0;Sr=[];alx=0;Shum=[];Wful=[];F1=[];F2=[];
u1=[];
als=[];

InputY1=zeros(1,k1_1);
InputY2=zeros(1,k1_1);
xestv=[];Outx=[];
u=zeros(2,1);
 Pe=[P zeros(2,6)];
dt=1e-5; Tend=0.5;

for t=0:dt:Tend
 u=-P*ex;%Управление
 u1=[u1 u];
  y=C*x;%Наблюдение  
  Delt_x=y-C*ex;%Разность между выходом фильтра и измерен
  w=[rand(1); rand(1)];
  dx=A*x+B*(u+w);%Основная модель 
  dex=A*ex+B*u+K*Delt_x;%Фильтр
  x=x+dx*dt;%Перменная состояния объекта
  exlast=ex;
  ex=ex+dex*dt;%Переменная состояния фильтра
 
  %Определение праметров наблюдения
OutX=[OutX x];
i=i+1;
end
Out=OutX;