% 生成并联机构雅克比矩阵，通过雅克比矩阵推导末端位置误差
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
num = 200; % 位姿个数
d_par = 0.1e-3;
T_ideal = zeros(4*num,4);
JJ1 = zeros(6*num,204);
JJ2 = zeros(6*num,112);
Sigma = diag(d_par*d_par*ones(204,1)/3);
Sigma_para = zeros(6*num,6*num);
for i = 1:num
    % 生成随机位姿
    [h1,h2] = parasitic_motion([0 0 0.14 + 0.01*rands(1)],0.35*rands(3,1));
    T_ideal(4*i-3:4*i,:) = posture_matrix(h1,h2);
    % 计算雅可比矩阵
    % [JJ1(6*i-5:6*i,:),JJ2(6*i-5:6*i,:)] = JacobianPara(T_ideal(4*i-3:4*i,:),screw_ideal,screw_local_T,screw_local,T_end_ideal,theta);
    [JJ1(6*i-5:6*i,:),JJ2(6*i-5:6*i,:)] = JacobianPara(T_ideal(4*i-3:4*i,:),screw_real,screw_local_T_real,screw_local,T_end_real,theta);
    
    Sigma_para(6*i-5:6*i,6*i-5:6*i) = JJ1(6*i-5:6*i,:)*Sigma*JJ1(6*i-5:6*i,:)';
end

% 问题1：如何使矩阵满秩？
% 在理论位姿下，初始的时候存在一些假设使矩阵不满秩，如果矩阵满秩，我就能进行反向运算，
% 问题2：如何在满秩的情况下进行求解？
% 问题3：矩阵的特征值和特征向量对应，但是如何确定哪个特征值对应的哪个误差？
% 问题4：正向运算的时候特征值是给定的，反向运算时如果涉及到计算特征值和特征向量就不对应了
% 问题5：如果先反向计算椭圆轴长，得到满足要求的轴长后再通过轴长还原输入协方差矩阵
% 问题6：给定椭球轴长，以轴长为特征值求逆得到协方差矩阵
% 问题7：

TTT = T_ideal;
mu_TTT = Vee(logm(T_ideal));
tt1 = zeros(3,num);
ang1 = zeros(3,num);
[JJ1,JJ2] = JacobianPara(T_ideal,screw_ideal,screw_local_T,screw_local,T_end_ideal,theta);
tt2 = JJ1*d_par*rands(204,num);
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

figure


% 比较协方差，这两种方式得到的结果本身是等价的，于是说可以用第二种误差传递的方式用来表示蒙特卡洛模拟的结果





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 精度综合
% pinv(JJ1)*Sigma_new*pinv(JJ1)'
% JJ1不满秩，当JJ1满秩时可以处理

% 给定矩阵 J 和常数 sigma
J = JJ2;% 你的 J 矩阵
sigma = 1e-4;% 你的 sigma 常数

% 定义问题的优化函数
% objective = @(Sigma) max(abs(eig(J * diag(Sigma) * J'))); % 目标函数求最大的谱半径
objective = @(Sigma) max(norm(Sigma)); % 目标函数：使Sigma模长最大（Sigma模长最大物理意义是使整体的公差范围最大）

% 设置优化选项
options = optimset('Display', 'iter', 'MaxFunEvals', 10000, 'TolFun', 1e-6);

% 使用 fmincon 进行优化
% Sigma0 = eye(size(J)); % 初始矩阵可以是单位矩阵
Sigma0 = 1e-4*ones(112,1)/2;

% 约束函数constraint，谱半径小于给定sigma
Sigma_optimal = fmincon(objective, Sigma0, [], [], [], [], zeros(112,1), 1e-4*ones(112,1), @(Sigma) constraint(Sigma, sigma, J), options);

% 显示最优解
disp('Optimal Sigma:');
disp(Sigma_optimal);




