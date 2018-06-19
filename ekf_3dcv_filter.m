%�ο�����
%Decoupling joint probabilistic data association algorithm for multiple target tracking 
%�Ӳ������¶ഫ�����������ں�
%��ά����CVģ��

clear all;
clc;

T=1;                                                     % ��������
hits=2000;                                               % ��������
MCNum=10;                                                % Monte Carlo�������
Qn=50;                                                   % �۲�����׼�� 
R_Q=50;                                                  % R����۲�����׼��
THETA_Q=0.1;                                               % THETA����۲�����׼��
PHI_Q=0.1;                                                 % PHI����۲�����׼��
v_x=300; v_y=200; v_z=100;                               % X��Y��Z������ٶ�
x0=1000; y0=5000;  z0=10000;                           % ��ʼλ��

DX_Average=zeros(1,hits);                                % Monte Carlo�������Ҫ
DY_Average=zeros(1,hits);
DZ_Average=zeros(1,hits);

O2=[0 0; 0 0];                                           % 2*2�׵������Ϊ���Ժ�ֵ�ķ���
O3=[0,0,0]'; 

f=[1 T;0 1];
F=[f O2 O2;O2 f O2;O2 O2 f];                             % ״̬ת�ƾ���

q=[T^4/4 T^3/2;T^3/2 T^2]; 
Q=[q O2 O2;O2 q O2;O2 O2 q];                             % ģ������Э������

Qq=0.001;                                                % ģ����ٶȵĹ�����������
Q=Q*Qq;

% H=[1 0 0 0 0 0 ;0 0 1 0 0 0;0 0 0 0 1 0 ];             % �۲����

R=[R_Q^2 0 0;0 THETA_Q^2 0;0 0 PHI_Q^2];                         % �۲�����Э������

p1=[R_Q^2 0;0 1];
p2=[THETA_Q^2 0;0 1];
p3=[PHI_Q^2 0;0 1];

P0=[p1 O2 O2;O2 p1 O2;O2 O2 p1];                           % �������Э�������ֵ

X0=[x0,v_x,y0,v_y,z0,v_z]';                    % ״̬������ֵ

%ģ��켣
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

% Monte Carlo ����Ŀ�ʼ
for i=1:1:MCNum

noise_x=Qn*randn(1,hits);
noise_y=Qn*randn(1,hits);
noise_z=Qn*randn(1,hits);


Z_X=X+noise_x;                                          %X���������
Z_Y=Y+noise_y;                                          %Y���������
Z_Z=Z+noise_z;                                          %Z���������

noise_r=R_Q*randn(1,hits);
noise_theta=THETA_Q*randn(1,hits);
noise_phi=PHI_Q*randn(1,hits);


Z_R=Ro+noise_r;                                                     %X���������
Z_THETA=THETA+noise_theta;                                          %Y���������
Z_PHI=PHI+noise_phi;                                                %Z���������

Z_T=[Z_R;Z_THETA;Z_PHI];

%ע��ÿ��ѭ����Ҫ��X0��P0����ֵ
X0=[x0,v_x,y0,v_y,z0,v_z]';                                % ״̬������ֵ
P0=[p1 O2 O2;O2 p1 O2;O2 O2 p1];                           %�������Э�������ֵ

%EKF�˲�����
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
    
    Pk_predict=F*P0*F'+Q;                                      %Ԥ�����Э������
    S=H*Pk_predict*H'+R;                                       %��ϢЭ������
    K=Pk_predict*H'*inv(S);                                    %�������

    Z_predict=[sqrt(xm^2+ym^2+zm^2),atan(ym/xm),atan(zm/sqrt(xm^2+ym^2))]';

    Xk(:,t)=Xk_predict+K*(Z_T(:,t)-Z_predict);                 %���ƾ���(�������ֵ)
    Pk=(eye(6)-K*H)*Pk_predict;                                %�������Э������
    P0=Pk;                                                     %�������Э������
    X0=Xk(:,t);                                                %���ƾ������
end

% [x,y,z] = sph2cart(THETA,PHI,R)

DX=abs(X-Xk(1,:));                                             %X������˲����
DY=abs(Y-Xk(3,:));                                             %Y������˲����
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
title('ԭʼ�켣�������')

subplot(2,1,2);
plot(t,DX_Average(t),'r',t,DY_Average(t),'g',t,DZ_Average(t),'b');
legend('X','Y','Z');
title('X��Y��Z������˲����')