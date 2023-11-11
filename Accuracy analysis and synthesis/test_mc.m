% 定义蒙特卡洛采样数量
num_samples = 10000;

% 生成随机样本在一定范围内
x_samples = rand(num_samples, 1) * 10; % 假设x在[0, 10]范围内
y_samples = rand(num_samples, 1) * 10; % 假设y在[0, 10]范围内

% 代入方程组
equation_results = 2 * x_samples + 3 * y_samples;

% 找到满足条件的样本
valid_samples = [x_samples(equation_results < 10), y_samples(equation_results < 10)];
valid_samples1 = [x_samples(equation_results < 5), y_samples(equation_results < 5)];
valid_samples = [valid_samples;valid_samples1];
% 绘制散点图显示采样结果
scatter(valid_samples(:, 1), valid_samples(:, 2), 'b.');
xlabel('x');
ylabel('y');
title('Monte Carlo Sampling for Underdetermined System');

% 显示方程
hold on;
x_range = 0:0.1:10;
y_range = (6 - 2 * x_range) / 3;
plot(x_range, y_range, 'r-', 'LineWidth', 2);
legend('满足条件的样本', '方程 2x + 3y = 6');
hold off;
