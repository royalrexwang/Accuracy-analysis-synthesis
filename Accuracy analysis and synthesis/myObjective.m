function [ff] = myObjective(ps,TT,range)
    % 输入：给定名义末端位姿，给定末端位姿变化范围
    % 输出：末端位姿偏差最小
    initial_parameters_initp;
% screw_local 关节旋量在局部坐标系下的表征（单位旋量）
% screw_local_T 相邻关节之间的转换矩阵（旋量）
% screw_local_T_real 带误差的相邻关节之间的转换矩阵（旋量）
% screw_ideal 全局坐标系下的关节旋量（旋量）
% screw_real 全局坐标系下带误差的关节旋量（旋量）
% T_end_ideal 初始位姿下的支链末端位姿
% T_end_real 初始位姿下的带误差的支链末端位姿
% theta 初始位姿下的关节转角
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 末端位姿作差
% 根据理论平台末端位姿求解关节角，同时求解实际末端位姿

pp = {screw_local_T{1}+reshape(ps(1:36),[6,6]);
      screw_local_T{2}+reshape(ps(37:78),[6,7]);
      screw_local_T{3}+reshape(ps(79:120),[6,7]);
      screw_local_T{4}+reshape(ps(121:162),[6,7]);
      screw_local_T{5}+reshape(ps(163:204),[6,7]);};

theta_ideal_constp = zeros(29,1);
[theta_ideal_constp(1:5,1),~] = IKinSpace(screw_ideal{1},T_end_ideal(1:4,:),TT,theta{1},1e-12,1e-12);
[theta_ideal_constp(6:11,1),~] = IKinSpace(screw_ideal{2},T_end_ideal(5:8,:),TT,theta{2},1e-12,1e-12);
[theta_ideal_constp(12:17,1),~] = IKinSpace(screw_ideal{3},T_end_ideal(9:12,:),TT,theta{3},1e-12,1e-12);
[theta_ideal_constp(18:23,1),~] = IKinSpace(screw_ideal{4},T_end_ideal(13:16,:),TT,theta{4},1e-12,1e-12);
[theta_ideal_constp(24:29,1),~] = IKinSpace(screw_ideal{5},T_end_ideal(17:20,:),TT,theta{5},1e-12,1e-12);
% 计算实际末端位姿
[T_const_real, ~] = realPosture1(screw_local,theta_ideal_constp(:,1),pp);
data_plot(:,1,1) = [T_const_real(1:3,4);PoseTrans(T_const_real(1:3,1:3),'R2C')] - [TT(1:3,4);PoseTrans(TT(1:3,1:3),'R2C')];
error_pos = norm(data_plot(1:3,1,1));
error_ang = norm(data_plot(4:6,1,1));
% 不停变化边界，求解满足边界的运动学参数
ff(1) = norm(error_pos - range(1));
ff(2) = norm(error_ang - range(2));

end
