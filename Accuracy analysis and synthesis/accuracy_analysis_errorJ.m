addpath("C:\Users\lenovo\Documents\GitHub\POE\POE without Redundant_3PRRU\Ad.m");
addpath("C:\Users\lenovo\Documents\GitHub\ModernRobotics\packages\MATLAB\mr\ad.m")
% 精度预估
% 蒙特卡洛预估
% 选取yz工作空间内一个平面切片为预估范围
% 离散化选取N个点，并计算对应的位姿
% 将所有运动学参数代入误差，通过正解计算末端位姿误差，进行M次模拟后计算该点处的体积误差均值
clear;clc
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 参数初始化
initial_parameters_initp;
n = 30;
x = linspace(0,0,n)/1000;
y = linspace(-50,50,n)/1000;
z = linspace(146,146+50,n)/1000;
p = [x;y;z];
[y_new,z_new] = meshgrid(y,z);
y_new1 = reshape(y_new,[1,n^2]);
z_new1 = reshape(z_new,[1,n^2]);
p_new1 = [y_new1*0;y_new1;z_new1];
T_ideal_yz = zeros(4*n^2,4);
T_ideal_ang = zeros(4*n^2,4);
P = zeros(3,n^2);



for i = 1:n^2
%     angle = zeros(3,1);
%     [p_new1(:,i),angle] = parasitic_motion(p_new1(:,i),angle);
    T_ideal_yz(4*i-3:4*i,:) = posture_matrix(p_new1(:,i),zeros(3,1));
    [h1,h2] = parasitic_motion(p_new1(:,i),0.35*rands(3,1));
    T_ideal_ang(4*i-3:4*i,:) = posture_matrix(h1,h2);

    P(:,i) = T_ideal_yz(4*i-3:4*i-1,4);
end
% T_ideal = gpuArray(T_ideal);
% 生成运动学参数随机误差
tic
d_par = 0.1e-3;
delta_P = zeros(2,n^2);
delta_VA = zeros(2,n^2);

for i = 1:n^2
    disp(i)
    temp1 = 0;
    temp2 = 0;
    JJ1 = JacobianPara(T_ideal_yz(4*i-3:4*i,1:4),screw_ideal,screw_local_T,screw_local,T_end_ideal,theta);
    JJ2 = JacobianPara(T_ideal_ang(4*i-3:4*i,1:4),screw_ideal,screw_local_T,screw_local,T_end_ideal,theta);
%     clc
    parfor k = 1:100000
        tt1 = JJ1*d_par*rands(112,1);
        tt2 = JJ2*d_par*rands(112,1);
        temp1 = temp1 + norm(tt1);
        temp2 = temp2 + norm(tt2);
    end
    delta_VA(1,i) = temp1/100000;
    delta_VA(2,i) = temp2/100000;
    clc
end
% 绘图c
figure(1)
xlim([-0.05, 0.05]);
ylim([0.146, 0.196]);
zlim([0, 1e-2]);
surf(reshape(P(2,:),n,n),reshape(P(3,:),n,n),reshape(delta_VA(1,:),n,n))
hold on
surf(reshape(P(2,:),n,n),reshape(P(3,:),n,n),reshape(delta_VA(2,:),n,n))
colorbar
figure(2)
surf(reshape(P(2,:),n,n),reshape(P(3,:),n,n),reshape(delta_VA(2,:),n,n))
colorbar
toc
% figure(3)
% surf(reshape(P(2,:),n,n),reshape(P(3,:),n,n),reshape(delta_VA(2,:),n,n),abs(reshape(P(2,:),n,n)).*abs(reshape(P(3,:),n,n)))
figure(3)
mesh(reshape(P(2,:),n,n),reshape(P(3,:),n,n),reshape(delta_VA(1,:),n,n))
colorbar
figure(4)
mesh(reshape(P(2,:),n,n),reshape(P(3,:),n,n),reshape(delta_VA(2,:),n,n))
colorbar
kk = JJ1'*JJ1;
for hf = 1:112
    mu(hf,1) = norm(kk(:,hf));
end



