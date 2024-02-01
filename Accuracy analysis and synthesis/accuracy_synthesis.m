% clc
% 给定矩阵 J 和常数 sigma
% J = JJ2(1:18,:);% 你的 J 矩阵
load("/MATLAB Drive/新建文件夹/matlab.mat")
J = JJ2;
sigma = [0.01 ;1e-3];% 你的 sigma 常数

% 定义问题的优化函数
% objective = @(Sigma) max(abs(eig(J * diag(Sigma) * J'))); % 目标函数求最大的谱半径
objective = @(Sigma) -Sigma'*Sigma; % 目标函数：使Sigma模长最大（Sigma模长最大物理意义是使整体的公差范围最大）

% 设置优化选项
options = optimset('Display', 'iter');

% 使用 fmincon 进行优化
% Sigma0 = eye(size(J)); % 初始矩阵可以是单位矩阵
Sigma0 = 5e-5*ones(112,1);

% 约束函数constraint，谱半径小于给定sigma
Sigma_optimal = fmincon(objective, Sigma0, [], [], [], [], 1e-5*ones(112,1), 1e-4*ones(112,1), @(Sigma) constraint(Sigma, sigma, J), options);

% 显示最优解
disp('Optimal Sigma:');
disp(Sigma_optimal);
anss = sqrt(Sigma_optimal)
 % 3*sqrt(max(eig(J*diag(1e-8*ones(112,1))*J')))
for i = 1:1:200
    tk = J(6*i-5:6*i,:) * diag(Sigma_optimal.*Sigma_optimal/3) * J(6*i-5:6*i,:)';
    [6 * sqrt(max(diag(tk(1:3,1:3)))) - sigma(1), 6 * sqrt(max(diag(tk(4:6,4:6)))) - sigma(2)]
end