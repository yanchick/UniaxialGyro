clear all;
dt=10e-4;%Шаг интегрирования
T0=10;%Время интегрирования
ksi=0.7;
w0=1*2*pi;
H=0.12;
A=0.016;
B=0.0005;
F=[0  -H/B 0;     %альфа'
   H/A   0 0;     %бета'
   0     1 0;];   %бета
G=[1; 0 ;0];
C=[0 0 1];  
D=zeros(3,1);
w0=5;w1=10;ksi=0.9;
goal=sort([desir(w0,ksi) -1/w1]);
P=place(F,G,goal);
w0=30;w1=40;ksi=0.65;
goal=sort([desir(w0,ksi) -1/w1]);
L=place(F',C',goal)';
Sk=ss(F,G,C,zeros(1,1));
%Формирования случайных величин
Ma=1;%СКО момента
Umax=0.01;%СКО шума
%Моделирование Системы

x=zeros(3,1);ex=zeros(3,1);
i=1;
OutX=[];OutEx=[];q=[];West=[];
Jmp=5;xlast=0;Sr=[];alx=0;Shum=[];Wful=[];F1=[];F2=[];
u1=[];
als=[];
u=-P*x;
DeelX=[];

for t=0:dt:T0
  dx=F*x+G*u+G*rand(1);%Основная модель 
  x=x+dx*dt;%Перменная состояния объекта
  y=C*x+0.1*rand(1);%Наблюдение  
  dex=(F-L*C)*ex+L*y+G*u;%Фильтр
   u=-P*ex;%Управление
%  exlast=ex;
  ex=ex+dex*dt;%Переменная состояния фильтра
  Delt_x=y-C*ex;%Разность между выходом фильтра и измерен
  DeelX=[DeelX  sqrt((ex-x).^2)];
  %Эталонная модель
  %Определение праметров наблюдения
OutX=[OutX x];
i=i+1;
end

alpha0=0;alpha=[];
for i=1:length(OutX)
alpha0=alpha0+OutX(1,i)*dt;
alpha=[alpha rad2deg(alpha0)*60];
end
max(alpha)
sum(DeelX')./length(DeelX);