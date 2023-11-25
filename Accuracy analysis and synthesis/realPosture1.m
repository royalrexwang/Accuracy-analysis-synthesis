function [f,theta_new]= realPosture1(screw,theta,screw_T)
% 由理论末端位姿推导实际末端位姿
% 输入：旋量轴（5根支链）-元胞，关节角（五根支链所以关节角）-元胞
% 输出：实际末端位姿
% theta
% 末端旋量作差初始化
e_ij = ones(24,1);
% 迭代
ii = 0;
screw_global = screw;
for i = 1:5
    screw_global{i} = local2globalPOE(screw{i},screw_T{i});
end
while norm(e_ij) > 10e-8
    ii = ii + 1;
    link1_real = Fserial_local(screw{1},theta(1:5),screw_T{1});
    link2_real = Fserial_local(screw{2},theta(6:11),screw_T{2});
    link3_real = Fserial_local(screw{3},theta(12:17),screw_T{3});
    link4_real = Fserial_local(screw{4},theta(18:23),screw_T{4});
    link5_real = Fserial_local(screw{5},theta(24:29),screw_T{5});

    e_ij = [se3ToVec(MatrixLog6(link2_real/link1_real));...
            se3ToVec(MatrixLog6(link3_real/link2_real));...
            se3ToVec(MatrixLog6(link4_real/link3_real));...
            se3ToVec(MatrixLog6(link5_real/link4_real))];
    % 雅克比矩阵
    J1 = JacobianSpace(screw_global{1},theta(1:5));
    J2 = JacobianSpace(screw_global{2},theta(6:11));
    J3 = JacobianSpace(screw_global{3},theta(12:17));
    J4 = JacobianSpace(screw_global{4},theta(18:23));
    J5 = JacobianSpace(screw_global{5},theta(24:29));
    J1(:,4) = [];
    J2(:,3) = [];
    J3(:,3) = [];
    J4(:,3) = [];
    J5(:,3) = [];

    J = zeros(24,24);
    J(1:6,1:4) = J1;
    J(1:6,5:9) = -J2;
    J(7:12,5:9) = J2;
    J(7:12,10:14) = -J3;
    J(13:18,10:14) = J3;
    J(13:18,15:19) = -J4;
    J(19:24,15:19) = J4;
    J(19:24,20:24) = -J5; 
    
    % 更新角度
    theta_k = theta([1 2 3 5 6 7 9 10 11 12 13 15 16 17 18 19 21 22 23 24 25 27 28 29]);
    theta_k = theta_k + pinv(J)*e_ij;
%     theta_k = theta_k + pinv(J'*J + 0.1*eye(24))*J'*e_ij;
%     theta_k = theta_k + tls(J,e_ij,0.01)
    theta([1 2 3 5 6 7 9 10 11 12 13 15 16 17 18 19 21 22 23 24 25 27 28 29]) = theta_k;
    if ii > 2000
        disp("Break!%%%%%%%%%%%%%%%%%%%%%%%")
        e_ij
        break
    end
end
% 输出f = position + posture
link1_real = Fserial_local(screw{1},theta(1:5),screw_T{1});
% f = se3ToVec(MatrixLog6(link1_real));
f = link1_real;
% f = se3ToVec(MatrixLog6(f));
theta_new = theta;
if norm(e_ij) < 10e-8
%     disp("finish!")
end
end