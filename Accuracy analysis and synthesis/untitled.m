% 设置矩阵的大小
rows = 6;
cols = 20;

% 生成一个随机矩阵
A = rand(rows, cols);

% 检查矩阵的秩
while rank(A) < min(rows, cols)
    % 如果矩阵不是满秩的，则重新生成
    A = rand(rows, cols);
end

% 显示生成的满秩矩阵
disp('满秩矩阵 A:');
disp(A);
p = rands(20,1)
e = A*p
[pinv(A)*e p]

[e A*pinv(A)*e]

% 误差传播协方差矩阵
% 给定末端位姿误差范围，如何求解满足条件的协方差矩阵
% 空间中的概率密度函数
% 要求空间中的误差概率密度函数最小

% 之前是没办法量化随机误差偏离均值的程度，现在可以通过协方差定义误差空间的大小
% 如果在给定误差空间大小的情况下，如何反向求解输入误差源的误差空间大小
% 工作空间中输出误差空间整体最大
% 工作空间中输出误差空间整体最小
% 工作空间中输出误差空间均衡

% 工作空间概率密度函数的物理意义？
% 工作空间概率密度函数如何使用？
% 如何定义一个类似的概率密度函数评价误差空间？
% 并联存在各个不同的支链，对于多根支链，如何推导出共同的协方差矩阵
% 表示误差范围d的参数如何应用的？
% 由于结构参数误差，并联机构存在协调运动，如何将协调运动加入建模中
% 对于固定的运动学参数，不考虑关节处的间隙，那么零部件结构参数确定后得到的机构末端位姿也是唯一确定的
% 以上内容与运动学正解相关联
% 如何考虑误差传播与约束关系
% 构成的闭合回环与误差传播有什么关联
% 在标定方面要求从支链出发，支链构成的末端位姿满足相等，从不同的支链出发得到的末端位姿协方差矩阵应当是一样的，但保证一样的原因是因为约束条件的存在
% 推导出当前位型下支链误差参数的影响对末端位姿误差输出的影响，是整体误差源对末端位姿误差的影响
% 因此，推导出的雅克比矩阵可作为传递关系

 
% 各个误差参数是否相互独立的？如何说明是相互独立的



% 现在已知末端位置误差正态分布函数的均值和协方差，如何反向求解112个独立误差源的均值和协方差？
% 给定多个均值跟协方差输入，会得到当前位型下的均值和协方差输出，根据这个输入和输出可以做回归吗？
% 在标定问题中我们不知道实际的输入参数，但是这个输入参数是唯一的固定的，所以通过测量不同位姿下的输出情况可以反应输入的特征
% 在位型确定下，给定不同的输入会得到对应的输出
% 现在我想在给定输出的情况下反向求解输入，反向求解时存在冗余，需要不断处理冗余信息
 

% 其中误差满足高维椭球面约束
% 多元正态分布中存在椭球
% 多元正态分布与椭球之间的关系



% 得到末端均值和协方差后就能得到末端概率密度分布函数，概率密度函数描述离某一点处的概率，概率密度函数的和表示离该点概率的大小，越大表示离该点越集中，在3sigma范围内表示接受所有的值
% 根据概率密度函数能够得到样本的分布情况，样本离中心点的远近，样本在中心点处的集中程度，通过集中程度能反向看轴上哪段范围比较集中
% 一维概率密度函数是一维输入一维输出，二维概率密度函数使二维输入一维输出，三维概率密度函数使三维输入一维输出
% 一维分布在一条线上离点的附件分布，二维分布离一个点的平面分布，三维分布离一个点的空间分布，分布情况通过概率密度函数反应

% 给定概率密度函数（均值和协方差），如何绘制椭球？
% 经过雅克比矩阵映射后得到末端误差变量，输入204个变量，这204个变量是相互独立的，存在非零向量通过映射后为零，投影到了零空间（被矩阵映射到零向量的输入向量的集合）
% 非零向量映射到零空间后
% 零空间与矩阵的位型有关
% 已知矩阵的零空间和行空间
% 给定多个误差矩阵，这样多个位型下可以构成矩阵的零空间和行空间，这样的矩阵不影响误差的正向传播，但是对于矩阵误差的逆向求解是否有帮助？
% 假设：我再给定多个位姿下获得了多个雅克比矩阵，如果是在测量观察下是可以逆向求解出输入误差的，但是现在我不知道输入情况，得到了多个末端位姿误差的限定范围，现在能反向求解输入误差范围吗？
% 问题1：给定误差雅克比矩阵，给定自变量正态分布概率密度函数，能够得到各个位型下的末端概率密度函数（√），反过来，如果我知道雅克比矩阵和末端正态分布概率密度函数，能反向得到自变量正态分布概率密度函数吗（？）（我知道，由于误差雅可比矩阵不满秩，所以自变量中存在冗余变量，只能求解消除冗余后的独立变量，如果上述问题能解决，那个这个问题是可以接近标定问题的求解）
% 问题2：在我给定末端概率密度函数时，还能等价为回归问题吗？能用最小二乘法解决吗？
% 问题3：寻找一组参数，使得所有的概率密度函数的最大协方差小于给定范围，二次规划能解决这个问题吗？
% 账户数空比0.61 持仓量1.02 多空人数比0.53




