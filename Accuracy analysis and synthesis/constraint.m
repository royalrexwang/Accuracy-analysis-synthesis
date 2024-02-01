function [c, ceq] = constraint(Sigma, sigma, J)
% 辅助函数定义约束条件
    % c = [3*sqrt(max(eig(J(1:6,:) * diag(Sigma) * J(1:6,:)'))) - sigma;...
    %     3*sqrt(max(eig(J(7:12,:) * diag(Sigma) * J(7:12,:)'))) - sigma;...
    %     3*sqrt(max(eig(J(13:18,:) * diag(Sigma) * J(13:18,:)'))) - sigma;];
    % 初始化不等式约束向量

    c = zeros(200*2, 1);

    % 逐个添加约束行
    for i = 1:1:200
        J_pos = J(6*i-5:6*i,:) * diag(Sigma.*Sigma/3) * J(6*i-5:6*i,:)';
        J_p = J_pos(1:3,1:3);
        J_ang = J_pos(4:6,4:6);
        c(2*i - 1) = 3 * sqrt(max(diag(J_p))) - sigma(1); % 姿态
        c(2*i) = 3 * sqrt(max(diag(J_ang))) - sigma(2); % 位置
    end

    % 等式约束为空
    ceq = [];
end