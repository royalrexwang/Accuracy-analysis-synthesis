
clear;clc
close all
% 生成并联机构雅克比矩阵，通过雅克比矩阵推导末端位置误差
addpath("C:\Users\lenovo\Documents\GitHub\POE\POE without Redundant_3PRRU\Ad.m");
addpath("C:\Users\lenovo\Documents\GitHub\ModernRobotics\packages\MATLAB\mr\ad.m")
 
load('C:\Users\royal\Desktop\Sigma_opt.mat')
% 精度预估
% 蒙特卡洛预估
% 选取yz工作空间内一个平面切片为预估范围
% 离散化选取N个点，并计算对应的位姿
% 将所有运动学参数代入误差，通过正解计算末端位姿误差，进行M次模拟后计算该点处的体积误差均值

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 参数初始化
initial_parameters_initp;
num = 100000;
d_par = 0.1e-4;
% d_par = 1e-4*rand(204,1);
[h1,h2] = parasitic_motion([0 0 0.14],[0.1509 0.335 0.1464]);
T_ideal = posture_matrix(h1,h2);
TTT = T_ideal;
mu_TTT = Vee(logm(T_ideal));
tt1 = zeros(3,num);
ang1 = zeros(3,num);
[JJ1,JJ2] = JacobianPara(T_ideal,screw_ideal,screw_local_T,screw_local,T_end_ideal,theta);
% tt2 = JJ1*d_par*rands(204,num);
tt2 = JJ1*diag(d_par)*rands(204,num);
parfor k = 1:num
        ht = expm(Wedge(tt2(:,k)))*TTT;
        tt1(:,k) = ht(1:3,4);
        ang1(:,k) = PoseTrans(ht(1:3,1:3),'OTC');
end
scatter3(tt1(1,:),tt1(2,:),tt1(3,:),'o','r','filled')
axis equal
hold on

% 根据协方差计算
Sigma = diag(d_par*d_par*ones(204,1)/3); % 这儿为什么除3协方差就一样了
% Sigma = diag(d_par.*d_par)/3;
Sigma_new = JJ1*Sigma*JJ1';
position_TTT = zeros(3,num);
ang_TTT = zeros(3,num);
randomSamples2 = mvnrnd(zeros(6,1), Sigma_new, num);
randomSamples2 = randomSamples2';
for i = 1:num
    TTT_b = expm(Wedge(randomSamples2(:,i)))*TTT;
    position_TTT(:,i) = TTT_b(1:3,4);
    ang_TTT(:,i) = PoseTrans(TTT_b(1:3,1:3),'OTC');
end
% figure
scatter3(position_TTT(1,:),position_TTT(2,:),position_TTT(3,:),'*','b', 'LineWidth',1)
% 示例数据
xlabel('x')
ylabel('y')
zlabel('z')
title('空间位置分布对比');
legend('误差传播理论','蒙特卡洛模拟')
set(gca,'Fontname','Monospaced');

figure
scatter3(ang1(1,:),ang1(2,:),ang1(3,:),'o','r','filled')
hold on
scatter3(ang_TTT(1,:),ang_TTT(2,:),ang_TTT(3,:),'*','b', 'LineWidth',1)

xlabel('\alpha')
ylabel('\beta')
zlabel('\gamma')
title('空间姿态分布对比');
legend('误差传播理论','蒙特卡洛模拟')
set(gca,'Fontname','Monospaced');

% 比较协方差，这两种方式得到的结果本身是等价的，于是说可以用第二种误差传递的方式用来表示蒙特卡洛模拟的结果
Sigma_mon = cov(tt2');
Sigma_new;
w1 = eig(Sigma_new(1:3,1:3))
l1_w = sqrt(w1)*3
p1 = eig(Sigma_new(4:6,4:6));
l1_p =sqrt(p1)*3*1000
disp("位置误差范围")
[sqrt(diag(Sigma_new(4:6,4:6)))*6 [max(position_TTT(1,:))-min(position_TTT(1,:)) max(position_TTT(2,:))-min(position_TTT(2,:)) max(position_TTT(3,:))-min(position_TTT(3,:))]']
disp("姿态误差范围")
[sqrt(diag(Sigma_new(1:3,1:3)))*6 [max(ang_TTT(1,:))-min(ang_TTT(1,:)) max(ang_TTT(2,:))-min(ang_TTT(2,:)) max(ang_TTT(3,:))-min(ang_TTT(3,:))]']


% 梳理协方差，椭圆轴，取值范围三者之间的关系




[A_tt2,B_tt2] = eig(Sigma_mon);
[A_new,B_new] = eig(Sigma_new);
d_tt2 = diag(B_tt2);
d_new = diag(B_new);
[d_tt2,ind_tt2] = sort(d_tt2,'descend');
[d_new,ind_new] = sort(d_new,'descend');
B_tt2 = B_tt2(ind_tt2,ind_tt2);
B_new = B_new(ind_new,ind_new);
A_tt2 = A_tt2(:,ind_tt2);
A_new = A_new(:,ind_new);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 精度综合
% pinv(JJ1)*Sigma_new*pinv(JJ1)'
% JJ1不满秩，当JJ1满秩时可以处理


% 根据协方差计算
hx = Sigma_opt/1000;
hx = hx.^2;
Sigma_new = JJ2*diag(hx/3)*JJ2';
position_TTTT = zeros(3,num);
ang_TTTT = zeros(3,num);
randomSamples3 = mvnrnd(zeros(6,1), Sigma_new, num);
randomSamples3 = randomSamples3';
for i = 1:num
    TTTT_b = expm(Wedge(randomSamples3(:,i)))*TTT;
    position_TTTT(:,i) = TTTT_b(1:3,4);
    ang_TTTT(:,i) = PoseTrans(TTTT_b(1:3,1:3),'OTC');
end
figure
scatter3(position_TTTT(1,:),position_TTTT(2,:),position_TTTT(3,:),'*','b', 'LineWidth',1)
% 示例数据
xlabel('x')
ylabel('y')
zlabel('z')
title('空间位置分布对比');
legend('误差传播理论','蒙特卡洛模拟')
set(gca,'Fontname','Monospaced');

figure
scatter3(ang_TTTT(1,:),ang_TTTT(2,:),ang_TTTT(3,:),'*','b', 'LineWidth',1)








