close all;clear;clc
% 定义优化变量的数量
nVars = 204; 

options = optimoptions('gamultiobj', ...
    'PopulationSize', 200, ...       % 较大的种群大小可以提高搜索的广度
    'MaxGenerations', 3, ...       % 增加代数可以提高解的精度
    'FunctionTolerance', 1e-2, ...   % 函数容差设置得更小可以提高解的精度
    'ParetoFraction', 0.7, ...       % Pareto前沿的比例
    'PlotFcn', {@gaplotpareto,@gaplotscorediversity}, ... % 可视化函数
    'Display', 'iter');              % 显示每一代的信息
% 定义变量的上下界
lb = -0.001*ones(1,204); % 变量的下界
ub = 0.001*ones(1,204); % 变量的上界
% 选项设置
% options = optimoptions('gamultiobj', 'PlotFcn', @gaplotpareto,'FunctionTolerance',1e-2);
T = posture_matrix([0,0,0.148],[0,0,0]);
range = [0.001;0.01];
% 调用优化函数
for i = 1:100
    range = rands(1:2);
    [x, fval] = gamultiobj(@(p) myObjective(p,T,range), 204, [], [], [], [], lb, ub, options);
end

