%% |*实现末端椭圆轨迹运动*|

%% step1.建立7自由度冗余机械臂模型
%
close all;clear;clc;

% D-H参数
Dbs=0.36;    %肩宽（脖子到肩关节距离）
Dse=0.42;   %上臂长（肩关节到肘关节距离）
Dew=0.40;   %下臂长（肘关节到腕关节距离）
Dwt=0.32;  %腕关节到末端距离

L1 = Link('d', Dbs,   'a', 0,    'alpha',  -pi/2);   %Link 类函数
L2 = Link('d', 0,     'a', 0,    'alpha',   pi/2);
L3 = Link('d', Dse,   'a', 0,    'alpha',  -pi/2);
L4 = Link('d', 0,     'a', 0,    'alpha',   pi/2);
L5 = Link('d', Dew,   'a', 0,    'alpha',  -pi/2);
L6 = Link('d', 0,     'a', 0,    'alpha',   pi/2);
L7 = Link('d', Dwt,   'a', 0,    'alpha',   0);
robot=SerialLink([L1,L2,L3,L4,L5,L6,L7]);   %SerialLink 类函数
robot.name='7DOF Manipulator';     %SerialLink 属性值
robot.display();    %显示参数表

init_ang0=[0 90 0 0 0 0 0]/180*pi;
q=init_ang0;
%% step2.圆形轨迹
%
delete IXT.txt IYT.txt
MouseDraw
pause; close all
A=importdata('IXT.txt')-100;A=A/100;
B=importdata('IYT.txt')-100;B=B/100;

N0=60;
dN=round(size(A,1)/N0);

vec=[0; 1; 0]; % 法向量

xt=0.5*A(2:dN:end); N=size(xt,1);
yt=0.8*ones(N,1);
zt=0.5*B(2:dN:end)+0.5;
% plot3(xt,yt,zt,'k.:'); hold on

% 轨迹点固连坐标系
T=zeros(4,4,N);
for ii=1:N
    T(:,:,ii)=[[1;0;0] [0;0;1] [0;1;0] [xt(ii);yt(ii);zt(ii)];[0 0 0 1]];
end
%% step.3各个轨迹点反解关节角
%
M=10; 
q0=zeros(N,7);
for ii=1:N
    TH=invkine2(T(:,:,ii),M, Dbs, Dse, Dew, Dwt);
    if ii==8
        Phi=5;
        q0(ii,:)=TH(Phi,:);
    else
        Phi=5;
        q0(ii,:)=TH(Phi,:);
    end    
end
%% step.4关节角空间规划
%
figure(2);set(gcf,'outerposition',get(0,'screensize'));
robot.plot(q); 

time=1;dt=0.1;
t=dt:dt:time;
step=length(t);

for iii=1:N
    init_ang=init_ang0;
    targ_ang=q0(iii,:);
    [q,qd,qdd] = jtraj(init_ang, targ_ang, step);
    init_ang0=targ_ang;
    
%     pause(0.1);
    robot.plot(q);  hold on
    plot3(xt(iii),yt(iii),zt(iii),'k.');
end
plot3(xt,yt,zt,'k.:');
