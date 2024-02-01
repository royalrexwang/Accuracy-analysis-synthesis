% 设定均值和协方差矩阵
mu = [0, 0, 0];  % 均值
sigma = 1e-6*[0.1372, -0.0155, 0.0284; 
              -0.0155, 0.0028, -0.0036; 
              0.0284, -0.0036, 0.0071];

% 求解协方差矩阵的特征值和特征向量
[eigenvectors, eigenvalues] = eig(sigma);

% 计算椭球的半轴长度
radii = sqrt(diag(eigenvalues));

% 生成三维高斯分布椭球的数据
[x, y, z] = ellipsoid(0, 0, 0, sqrt(radii(1)), sqrt(radii(2)), sqrt(radii(3)));

% 将椭球数据进行线性变换，以适应实际协方差矩阵
transformed_ellipsoid = [x(:), y(:), z(:)] * chol(sigma);

% 提取变换后的坐标
x_transformed = reshape(transformed_ellipsoid(:, 1), size(x));
y_transformed = reshape(transformed_ellipsoid(:, 2), size(y));
z_transformed = reshape(transformed_ellipsoid(:, 3), size(z));

% 绘制三维高斯分布椭球
figure;
surf(x_transformed + mu(1), y_transformed + mu(2), z_transformed + mu(3), 'FaceAlpha', 0.5);
xlabel('X轴');
ylabel('Y轴');
zlabel('Z轴');
title('三维高斯分布椭球');

% 设置坐标轴比例相等
% axis equal;
% grid on;
