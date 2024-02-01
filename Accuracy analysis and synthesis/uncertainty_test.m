clear;close all;clc
% 齐次变换矩阵
T_0 = eye(4);
T = eye(4);
r = 100;
T(1,4) = 1;
TT = eye(4);
TT(1,4) = 100*1;

% xi 服从正态分布 xi~N(0,Sigma)
% 蒙特卡洛模拟
mu = zeros(6,1);
Sigma = diag([0 0 0.03^2 0 0 0]);
num = 100;
N = 10000; % 样本数
xi = zeros(6,N);
position = zeros(3,N);
for k = 1:N
    T_b = eye(4);
    randomSamples = mvnrnd(mu, Sigma, num);
    randomSamples = randomSamples';
    for i = 1:num
	    T_b = T_b*expm(Wedge(randomSamples(:,i)))*T;
    end
    xi(:,k) = Vee(logm(T_b*pinv(TT)));
    position(:,k) = T_b(1:3,4);
end
scatter(position(1,1:100),position(2,1:100),'*','r')
hold on
axis equal


% 利用二阶结论
mu_TT = zeros(1,6);

Sigma_TT = zeros(6,6);
Sigma_TT(5,5) = 100*(100 - 1)*(2*100 - 1)*1^2*0.03^2/6;
Sigma_TT(3,5) = -100*(100 - 1)*1*0.03^2/2;
Sigma_TT(5,3) = Sigma_TT(3,5);
Sigma_TT(3,3) = 100*0.03*0.03;
position_TT = zeros(3,num);
randomSamples1 = mvnrnd(mu_TT, Sigma_TT, num);
randomSamples1 = randomSamples1';
% randomSamples1 = [randomSamples1(4:6,:);randomSamples1(1:3,:)];
for i = 1:num
    TT_b = expm(Wedge(randomSamples1(:,i)))*TT;
    position_TT(:,i) = TT_b(1:3,4);
end
% figure
scatter(position_TT(1,:),position_TT(2,:),'*','g')



Sigma_1 = diag([0 0 0 0 0 0]);

% Sigma_2 = diag([0 0 0 0 0 0.03*0.03]);
Sigma_2 = diag([0 0 0.03*0.03 0 0 0]);
T_1 = T;
Sigma_new = Sigma_1;
for i = 1:100
    Sigma_2 = Ad(T_1)*Sigma_2*Ad(T_1)';
    Sigma_new = Sigma_new + Sigma_2;
end

mu_TTT = zeros(1,6);
TTT = eye(4);
TTT(1,4) = 100*1;

position_TTT = zeros(3,num);
randomSamples2 = mvnrnd(mu_TTT, Sigma_new, num);
randomSamples2 = randomSamples2';
% randomSamples2 = [randomSamples2(4:6,:);randomSamples2(1:3,:)];
for i = 1:num
    TTT_b = expm(Wedge(randomSamples2(:,i)))*TTT;
    position_TTT(:,i) = TTT_b(1:3,4);
end
% figure
scatter(position_TTT(1,:),position_TTT(2,:),'*','b', 'LineWidth',1)

cov(xi')
Sigma_new
Sigma_TT




