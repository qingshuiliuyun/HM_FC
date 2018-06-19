%参考文献
%Decoupling joint probabilistic data association algorithm for multiple target tracking 
%杂波环境下多传感器的数据融合
%三维常速CV模型

clear all;
clc;

T=1;                                                     % 采样周期
hits=2000;                                               % 采样点数
MCNum=10;                                                % Monte Carlo仿真次数
Qn=50;                                                   % 观测误差标准差 
R_Q=50;                                                  % R方向观测误差标准差
THETA_Q=0.1;                                               % THETA方向观测误差标准差
PHI_Q=0.1;                                                 % PHI方向观测误差标准差
v_x=300; v_y=200; v_z=100;                               % X、Y和Z方向的速度
x0=1000; y0=5000;  z0=10000;                           % 初始位置

DX_Average=zeros(1,hits);                                % Monte Carlo仿真的需要
DY_Average=zeros(1,hits);
DZ_Average=zeros(1,hits);

O2=[0 0; 0 0];                                           % 2*2阶的零矩阵，为了以后赋值的方便
O3=[0,0,0]'; 

f=[1 T;0 1];
F=[f O2 O2;O2 f O2;O2 O2 f];                             % 状态转移矩阵

q=[T^4/4 T^3/2;T^3/2 T^2]; 
Q=[q O2 O2;O2 q O2;O2 O2 q];                             % 模型噪声协方差阵

Qq=0.001;                                                % 模拟加速度的过程噪声方差
Q=Q*Qq;

% H=[1 0 0 0 0 0 ;0 0 1 0 0 0;0 0 0 0 1 0 ];             % 观测矩阵

R=[R_Q^2 0 0;0 THETA_Q^2 0;0 0 PHI_Q^2];                         % 观测噪声协方差阵

p1=[R_Q^2 0;0 1];
p2=[THETA_Q^2 0;0 1];
p3=[PHI_Q^2 0;0 1];

P0=[p1 O2 O2;O2 p1 O2;O2 O2 p1];                           % 估计误差协方差阵初值

X0=[x0,v_x,y0,v_y,z0,v_z]';                    % 状态向量初值

%模拟轨迹
for t=1:T:hits
    if(t==1)
        X(t)=x0+v_x;
        Y(t)=y0+v_y;
        Z(t)=z0+v_z;
    else
        X(t)=X(t-1)+v_x;
        Y(t)=Y(t-1)+v_y;
        Z(t)=Z(t-1)+v_z;
    end
end

[THETA,PHI,Ro] = cart2sph(X,Y,Z);

% Monte Carlo 仿真的开始
for i=1:1:MCNum

noise_x=Qn*randn(1,hits);
noise_y=Qn*randn(1,hits);
noise_z=Qn*randn(1,hits);


Z_X=X+noise_x;                                          %X方向加噪声
Z_Y=Y+noise_y;                                          %Y方向加噪声
Z_Z=Z+noise_z;                                          %Z方向加噪声

noise_r=R_Q*randn(1,hits);
noise_theta=THETA_Q*randn(1,hits);
noise_phi=PHI_Q*randn(1,hits);


Z_R=Ro+noise_r;                                                     %X方向加噪声
Z_THETA=THETA+noise_theta;                                          %Y方向加噪声
Z_PHI=PHI+noise_phi;                                                %Z方向加噪声

Z_T=[Z_R;Z_THETA;Z_PHI];

%注意每次循环都要给X0和P0赋初值
X0=[x0,v_x,y0,v_y,z0,v_z]';                                % 状态向量初值
P0=[p1 O2 O2;O2 p1 O2;O2 O2 p1];                           %估计误差协方差阵初值

%EKF滤波方程
for t=1:T:hits
    Xk_predict=F*X0;

    xm=Xk_predict(1);
    ym=Xk_predict(3);
    zm=Xk_predict(5);

    r1=sqrt(xm^2+ym^2+zm^2);
    r2=sqrt(xm^2+ym^2);
    h1=[xm/r1, -ym/r2^2, -xm*zm/(r1^2*r2)]';
    h2=[ym/r1,  xm/r2^2, -ym*zm/(r1^2*r2)]';
    h3=[zm/r1,  0,        r2/r1^2        ]';
    H=[h1,O3,h2,O3,h3,O3];
    
    Pk_predict=F*P0*F'+Q;                                      %预测误差协方差阵
    S=H*Pk_predict*H'+R;                                       %信息协方差阵
    K=Pk_predict*H'*inv(S);                                    %增益矩阵

    Z_predict=[sqrt(xm^2+ym^2+zm^2),atan(ym/xm),atan(zm/sqrt(xm^2+ym^2))]';

    Xk(:,t)=Xk_predict+K*(Z_T(:,t)-Z_predict);                 %估计矩阵(最后的输出值)
    Pk=(eye(6)-K*H)*Pk_predict;                                %估计误差协方差阵
    P0=Pk;                                                     %估计误差协方差阵
    X0=Xk(:,t);                                                %估计矩阵更新
end

% [x,y,z] = sph2cart(THETA,PHI,R)

DX=abs(X-Xk(1,:));                                             %X方向的滤波误差
DY=abs(Y-Xk(3,:));                                             %Y方向的滤波误差
DZ=abs(Z-Xk(5,:));   

DX_Average=DX_Average+DX;
DY_Average=DY_Average+DY;
DZ_Average=DZ_Average+DZ;
end

DX_Average=DX_Average/MCNum;
DY_Average=DY_Average/MCNum;
DZ_Average=DZ_Average/MCNum;


t=1:hits;
subplot(2,1,1);
plot3(Xk(1,t),Xk(3,t),Xk(5,t), 'r',Z_X(t),Z_Y(t),Z_Z(t), 'g');
%legend('X','Y','Z');
grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('原始轨迹及其跟踪')

subplot(2,1,2);
plot(t,DX_Average(t),'r',t,DY_Average(t),'g',t,DZ_Average(t),'b');
legend('X','Y','Z');
title('X、Y和Z方向的滤波误差')