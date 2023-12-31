clc;
clear;
%%
%物理建模和状态空间矩阵
% 车体质量、惯量、质心高度
% mb = 14.48527;    %车体质量(整车减去轮子重量)%加入滑块后整车质量+1kg(16kg)
% Jb = 1.05512;   %车体对y轴惯量(yaw)
% L = 0.126807;     %质心到轮轴距离
% Jbz = 0.484517;   %车体对z轴惯量(pitch)
mb = 10.463524;    %车体质量(整车减去轮子重量)%加入滑块后整车质量+1kg(16kg)
Jb = 1.05512;   %车体对y轴惯量(yaw)
L = 0.107101;     %质心到轮轴距离
Jbz = mb*L*L;   %车体对z轴惯量(pitch)

% 车轮质量、惯量、轮半径、轮距
mw = 0.5 + 0.8;
Rw = 0.110;
Jw = 0.5 * mw * Rw * Rw + (4656 * 10^-7);%加上了9025转子的惯量
d = 0.21;

%滑块参数（一块）
ms = 0.735;
rs = 0.05;%滑块半径
Ls = 0.06 + rs;%滑块到轮轴距离

%质心耦合参量
k = (mb*L+2*ms*Ls);

% 重力加速度
g0 = 9.8;

% 电机最大扭矩
Tmax = 5.2;

%加入上坡坡度
p1=0.0001/180*pi;

syms M m Ja R h J Jo D g p
E=[R*(M+2*m+2*J/(R^2)),M*R*h          ,0                       ,0;...
    -1*M*h            ,-1*(M*(h^2)+Ja),0                       ,0;...
    0                 ,0              ,R*(m*D+J*D/(R^2)+2*Jo/D),0;...
    0                 ,0              ,0                       ,0];
F=[0                ,1,1 ,-M*R*g*sin(p)/p;...
    -1*M*g*h*cos(p),1,1 ,0;...
    0               ,1,-1,0;...
    0               ,0,0 ,0];
G=E\F;
S=simplify(G);  %求解传递函数
% disp(S);
S1=subs(S,{M m Ja R h J Jo D g p},{mb mw Jbz Rw L Jw Jb d g0 p1});    %将未知数替换为实参
S2=vpa(S1);     %设置数据精度，此处为默认精度
% disp(S2);

a23=double(S2(1,1));
a43=double(S2(2,1));
a52=double(S2(1,4));
a54=double(S2(2,4));
b21=double(S2(1,2));
b22=b21;
b41=double(S2(2,2));
b42=b41;
b61=double(S2(3,2));
b62=-b61;


%%
%观察状态空间矩阵的特征值
A1 = [0,1,0,0;...
    0,0,a23,0;...
    0,0,0,1;...
    0,0,a43,0];
e=eig(A1);      %求该矩阵的特征根|入I-A|=0
disp(e);
% M = [B A*B A^2*B ... A^n*B]	是满秩的，就说明系统是可控的
B1 = [0 0;...
    b21 b22;...
    0 0;...
    b41 b42];
A2 = [0 1;0 0];
B2 = [0 0;b61 b62];
M1=rank(ctrb(A1,B1));   %取可控性矩阵的秩
M2=rank(ctrb(A2,B2));
% disp(M1);
% disp(M2);

%%
%不考虑滑块系统
%直立环
%设计lqr控制器(二维输入)
% 原系统状态空间方程
A = [0,1,0,0;...
    0,0,a23,0;...
    0,0,0,1;...
    0,0,a43,0];
B = [0 0;...
    b21 b22;...
    0 0;...
    b41 b42];
C = eye(4);     %单位矩阵
D = zeros(4,2); %零矩阵

% LQR方法
    %参数1
    Q = eye(4);     %构建Q矩阵
    R = 1;          %构建R矩阵
%     Q(1,1) = 10;   %设置权重，每个值对应状态空间矩阵当中的物理量
%     Q(2,2) = 100;
%     Q(3,3) = 40;
%     Q(4,4) = 0.01;
    Q(1,1) = 100;   %设置权重，每个值对应状态空间矩阵当中的物理量
    Q(2,2) = 20;
    Q(3,3) = 48;
    Q(4,4) = 12;

    K = lqr(A, B, Q, R); %lqr计算,A是状态矩阵，B是输入矩阵，对应整个状态空间矩阵

    %参数2
    Q2 = eye(4);     %构建Q矩阵
    R2 = 1;          %构建R矩阵
    
    Q2(1,1) = 0.000000001;   %设置权重，每个值对应状态空间矩阵当中的物理量
    Q2(2,2) = 10;
    Q2(3,3) = 30;
    Q2(4,4) = 3;

    K2 = lqr(A, B, Q2, R2); %lqr计算,A是状态矩阵，B是输入矩阵，对应整个状态空间矩阵
    disp(K);
    disp(K2);
%%
%不考虑滑块边界的系统（LQR2）
a25 = 2*ms*a23/k;
a45 = 2*ms*a43/k;
a63 = g0;
b43 = (Ls-rs)/rs * b41;
b44 = b43;
b63 = 1/rs/ms;
b64 = b63;

% 原系统状态空间方程
A = [0,1,0,0,0,0;...
    0,0,a23,0,a25,0;...
    0,0,0,1,0,0;...
    0,0,a43,0,a45,0;...
    0,0,0,0,0,1;...
    0,0,a63,0,0,0];
B = [0,0,0,0;...
    b21,b22,0,0;...
    0,0,0,0;...
    b41,b42,b43,b44;...
    0,0,0,0;...
    0,0,b63,b64];
C = eye(6);     %单位矩阵
D = zeros(6,4); %零矩阵

% LQR方法
    %参数1
    Q = eye(6);     %构建Q矩阵
    R = eye(4);          %构建R矩阵

    Q(1,1) = 0.000000001;   %设置权重，每个值对应状态空间矩阵当中的物理量
    Q(2,2) = 20;
    Q(3,3) = 40;
    Q(4,4) = 4;
    Q(5,5) = 100;
    Q(6,6) = 5;
    
    R(1,1) = 1;
    R(2,2) = 1;
    R(3,3) = 1;
    R(4,4) = 1;
    

    K = lqr(A, B, Q, R); %lqr计算,A是状态矩阵，B是输入矩阵，对应整个状态空间矩阵

    %参数2
    Q2 = eye(6);     %构建Q矩阵
    R2 = eye(4);          %构建R矩阵
    
    Q2(1,1) = 0.000000001;   %设置权重，每个值对应状态空间矩阵当中的物理量
    Q2(2,2) = 15;
    Q2(3,3) = 60;
    Q2(4,4) = 5;
    Q2(5,5) = 400;
    Q2(6,6) = 4.2;
    
    R2(1,1) = 1;
    R2(2,2) = 1;
    R2(3,3) = 1;
    R2(4,4) = 1;

    K2 = lqr(A, B, Q2, R2); %lqr计算,A是状态矩阵，B是输入矩阵，对应整个状态空间矩阵
%     disp(K);
%     disp(K2);
    fprintf(sprintf("float body_speed_kp = %d;\n",K(1,2)));
    fprintf(sprintf("float body_pitch_kp = %d;\n",K(1,3)));
    fprintf(sprintf("float body_pitchSpeed_kp = %d;\n",K(1,4)));
    fprintf(sprintf("float body_sposition_kp = %d;\n",K(1,5)));
    fprintf(sprintf("float body_sspeed_kp = %d;\n\n",K(1,6)));
    
    fprintf(sprintf("float slider_speed_kp = %d;\n",K(3,2)));
    fprintf(sprintf("float slider_pitch_kp = %d;\n",K(3,3)));
    fprintf(sprintf("float slider_pitchSpeed_kp = %d;\n",K(3,4)));
    fprintf(sprintf("float slider_sposition_kp = %d;\n",K(3,5)));
    fprintf(sprintf("float slider_sspeed_kp = %d;\n\n",K(3,6)));

    fprintf(sprintf("float body_speed_kp = %d;\n",K2(1,2)));
    fprintf(sprintf("float body_pitch_kp = %d;\n",K2(1,3)));
    fprintf(sprintf("float body_pitchSpeed_kp = %d;\n",K2(1,4)));
    fprintf(sprintf("float body_sposition_kp = %d;\n",K2(1,5)));
    fprintf(sprintf("float body_sspeed_kp = %d;\n\n",K2(1,6)));
    
    fprintf(sprintf("float slider_speed_kp = %d;\n",K2(3,2)));
    fprintf(sprintf("float slider_pitch_kp = %d;\n",K2(3,3)));
    fprintf(sprintf("float slider_pitchSpeed_kp = %d;\n",K2(3,4)));
    fprintf(sprintf("float slider_sposition_kp = %d;\n",K2(3,5)));
    fprintf(sprintf("float slider_sspeed_kp = %d;\n\n",K2(3,6)));
%%
% %在状态空间矩阵里面加入上坡的模型
%     A(2,5)=a52;
%     A(4,5)=a54;
%     A(5,1)=0;
%     B(5,1)=0;
%     C(5,5)=1;
%     D(5,2)=0;
%     K(1,5)=0;
%     K2(1,5)=0;
%     disp(K);
%     disp(K2);
%     Turn_Init = [0;0];
%     Stand_Init = [0;0;35/180*pi;0;p1];
%%
%   %转向环
%     L = [0,1;0,0];
%     M = [0 0;b61 b62];
%     N = eye(2);
%     O = zeros(2,2);
%   % LQR方法
%     %参数1
%     Q1 = eye(2);     %构建Q矩阵
%     R1 = 1;          %构建R矩阵
%     Q1(1,1) = 0.010;   %设置权重，每个值对应状态空间矩阵当中的物理量
%     Q1(2,2) = 0.1;
%     K1 = lqr(L, M, Q1, R1); %lqr计算,A是状态矩阵，B是输入矩阵，对应整个状态空间矩阵
%     disp(K1);
%     %参数2
%     Q3 = eye(2);     %构建Q矩阵
%     R3 = 1;          %构建R矩阵
%     Q3(1,1) = 0.010;   %设置权重，每个值对应状态空间矩阵当中的物理量
%     Q3(2,2) = 1;
%     K3 = lqr(L, M, Q3, R3); %lqr计算,A是状态矩阵，B是输入矩阵，对应整个状态空间矩阵
%     disp(K3);
%     
%     
% % %%
% % %查看系统响应
% % sys_c = ss(A-B*K,B,C,D);      %将线性时不变系统转换为状态空间
% % t=0:0.005:4;                  %离散化状态步长
% % x=initial(sys_c,[0;0;1;0],t); %状态空间模型的初始条件响应对应（输出、速度、角度、角速度）
% % x1=[1 0 0 0]*x';              %取出每个状态,单引号为转置矩阵
% % x2=[0 1 0 0]*x'; 
% % x3=[0 0 1 0]*x'; 
% % x4=[0 0 0 1]*x'; 
% % subplot(2,2,1);               %画图
% % plot(t,x1),grid on; 
% % title(' displacement '); 
% % xlabel('t'),ylabel('x');
% % subplot(2,2,2);
% % plot(t,x2),grid on;
% % title('speed ');
% % xlabel('t'),ylabel('x');
% % subplot(2,2,3);
% % plot(t,x3),grid on;
% % title('angle ');
% % xlabel('t'),ylabel('x');
% % subplot(2,2,4);
% % plot(t,x4),grid on;
% % title('angle velocity');
% % xlabel('t'),ylabel('x');
% 
% % %%
% % %直立环
% % %设计lqr控制器（一维输入）
% % % 原系统状态空间方程
% % A = [0,1,0,0;...
% %     0,0,a23,0;...
% %     0,0,0,1;...
% %     0,0,a43,0];
% % B = [0;...
% %     b21;...
% %     0;...
% %     b41];
% % C = eye(4);     %单位矩阵
% % D = zeros(4,1); %零矩阵
% % 
% % % LQR方法
% %     Q = eye(4);     %构建Q矩阵
% %     R = 2;          %构建R矩阵
% % %     Q(1,1) = 0.0001;   %设置权重，每个值对应状态空间矩阵当中的物理量
% % %     Q(2,2) = 100;
% % %     Q(3,3) = 40;
% % %     Q(4,4) = 0.01;
% %      Q(1,1) = 1;   %设置权重，每个值对应状态空间矩阵当中的物理量
% %      Q(2,2) = 500;
% %      Q(3,3) = 1000;
% %      Q(4,4) = 1;
% % 
% %     K = lqr(A, B, Q, R); %lqr计算,A是状态矩阵，B是输入矩阵，对应整个状态空间矩阵
% %     disp(K);
