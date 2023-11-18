% % 定义矢量集合 A 和 B
% A = [1, 2; 3, 4; 5, 6];  % 三个二维矢量
% B = [1, 1; 4, 3; 6, 5];
% 
% % 使用循环实现二元矢量集合加法
% C_loop = zeros(size(A));
% for i = 1:size(A, 1)
%     C_loop(i, :) = A(i, :) + B(i, :);
% end
% 
% disp('通过循环实现的二元矢量集合加法结果：');
% disp(C_loop);
% 
% % 使用矩阵运算实现二元矢量集合加法
% C_matrix = A + B;
% 
% disp('通过矩阵运算实现的二元矢量集合加法结果：');
% disp(C_matrix);
% 不给点z方向角度
% y方向变化不会影响x方向的移动
clear;clc
x = [0,1,0.149];
angle = [0.5,0.1,0];
[x,angle] = parasitic_motion(x,angle)

x = [0,0.8,0.149];
angle = [0.5,0.1,0];
[x,angle] = parasitic_motion(x,angle)