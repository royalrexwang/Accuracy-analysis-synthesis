function T_real_j = cal_realPose(screw_local_T_real,T_ideal,screw_ideal,screw_local,T_end_ideal,theta)
% 通过给定运动学参数计算当前位姿下的理论位姿
%     initial_parameters_initp;

    theta_ideal0 = zeros(29,1);
    [theta_ideal0(1:5,1),aa] = IKinSpace(screw_ideal{1},T_end_ideal(1:4,:),T_ideal,theta{1},1e-12,1e-12);
    [theta_ideal0(6:11,1),bb] = IKinSpace(screw_ideal{2},T_end_ideal(5:8,:),T_ideal,theta{2},1e-12,1e-12);
    [theta_ideal0(12:17,1),cc] = IKinSpace(screw_ideal{3},T_end_ideal(9:12,:),T_ideal,theta{3},1e-12,1e-12);
    [theta_ideal0(18:23,1),dd] = IKinSpace(screw_ideal{4},T_end_ideal(13:16,:),T_ideal,theta{4},1e-12,1e-12);
    [theta_ideal0(24:29,1),ee] = IKinSpace(screw_ideal{5},T_end_ideal(17:20,:),T_ideal,theta{5},1e-12,1e-12);
    % 计算实际末端位姿
%     [aa bb cc dd ee]
    [T_real_j,~] = realPosture1(screw_local,theta_ideal0(:,1),screw_local_T_real);
%     tt = [T_real_j(1:3,4);PoseTrans(T_real_j(1:3,1:3),'OTC')] - [T_ideal(4*i-3:4*i-1,4);PoseTrans(T_ideal(4*i-3:4*i-1,1:3),'OTC')];
end