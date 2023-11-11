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
% screw_local 关节旋量在局部坐标系下的表征（单位旋量）
% screw_local_T 相邻关节之间的转换矩阵（旋量）
% screw_local_T_real 带误差的相邻关节之间的转换矩阵（旋量）
% screw_ideal 全局坐标系下的关节旋量（旋量）
% screw_real 全局坐标系下带误差的关节旋量（旋量）
% T_end_ideal 初始位姿下的支链末端位姿
% T_end_real 初始位姿下的带误差的支链末端位姿
% theta 初始位姿下的关节转角
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 选取工作空间
% 确定点数为n
n = 10;
x = linspace(0,0,n)/1000;
y = linspace(-50,50,n)/1000;
z = linspace(146,146+100,n)/1000;
p = [x;y;z];
[y_new,z_new] = meshgrid(y,z);
y_new1 = reshape(y_new,[1,n^2]);
z_new1 = reshape(z_new,[1,n^2]);
p_new1 = [y_new1*0;y_new1;z_new1];
T_ideal = zeros(4*n^2,4);
P = zeros(3,n^2);
for i = 1:n^2
    angle = zeros(3,1);
%     [p_new1(:,i),angle] = parasitic_motion(p_new1(:,i),angle);
    T_ideal(4*i-3:4*i,:) = posture_matrix(p_new1(:,i),angle);
    P(:,i) = T_ideal(4*i-3:4*i-1,4);
end
% 生成运动学参数随机误差
tic
d_par = 1e-4*0;c
for i = 1:n^2
    disp(i)
    for j = 1:1000
        theta_ideal = zeros(29,1);
        screw_local_T_monto = {screw_local_T{1} + (rand(6,6)-rand(6,6))*d_par;
                          screw_local_T{2} + (rand(6,7)-rand(6,7))*d_par;
                          screw_local_T{3} + (rand(6,7)-rand(6,7))*d_par;
                          screw_local_T{4} + (rand(6,7)-rand(6,7))*d_par;
                          screw_local_T{5} + (rand(6,7)-rand(6,7))*d_par;};
%         screw_local_T_monto{1}(6,1) = 0.001;

        [theta_ideal(1:5,1),~] = IKinSpace(screw_ideal{1},T_end_ideal(1:4,:),T_ideal(4*i-3:4*i,:),theta{1},1e-12,1e-12);
        [theta_ideal(6:11,1),~] = IKinSpace(screw_ideal{2},T_end_ideal(5:8,:),T_ideal(4*i-3:4*i,:),theta{2},1e-12,1e-12);
        [theta_ideal(12:17,1),~] = IKinSpace(screw_ideal{3},T_end_ideal(9:12,:),T_ideal(4*i-3:4*i,:),theta{3},1e-12,1e-12);
        [theta_ideal(18:23,1),~] = IKinSpace(screw_ideal{4},T_end_ideal(13:16,:),T_ideal(4*i-3:4*i,:),theta{4},1e-12,1e-12);
        [theta_ideal(24:29,1),~] = IKinSpace(screw_ideal{5},T_end_ideal(17:20,:),T_ideal(4*i-3:4*i,:),theta{5},1e-12,1e-12);
        % 计算实际末端位姿
        [T_real_j,~] = realPosture1(screw_local,theta_ideal(:,1),screw_local_T_monto);
        tt = [T_real_j(1:3,4);PoseTrans(T_real_j(1:3,1:3),'OTC')] - [T_ideal(4*i-3:4*i-1,4);PoseTrans(T_ideal(4*i-3:4*i-1,1:3),'OTC')];
        delta_P(:,j) = [norm(tt(2:3));norm(tt(4:6))];
        aha = cal_realPose(screw_local_T_monto,T_ideal(4*i-3:4*i,:))
        disp("fuck!")
    end
    delta_VA(:,i) = mean(delta_P')';
end
% 绘图c
figure(1)
surf(reshape(P(2,:),n,n),reshape(P(3,:),n,n),reshape(delta_VA(1,:),n,n))
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

