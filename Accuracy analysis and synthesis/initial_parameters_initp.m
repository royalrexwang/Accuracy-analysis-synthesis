%% 储存5根支链的关节变化量
theta0 = {zeros(5,1);zeros(6,1);zeros(6,1);zeros(6,1);zeros(6,1);};
theta = {zeros(5,1);zeros(6,1);zeros(6,1);zeros(6,1);zeros(6,1);};
theta_init = theta;
%% 运动学参数初始化
l_initial = 130.98/1000;
l_0 = zeros(5,1);% 初始杆长，1x1
% A 为定平台
A = [0, 40+50*sind(53), 90, -90, -40-50*sind(53);
        122-0, 72+50*cosd(53)-0, 0-0, 0-0, 72+50*cosd(53)-0;
        0, 0, 0, 0, 0;]/1000;
% B 为动平台

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p = [0; 52; 130.98]/1000;
% p = [0;0;130.98]/1000;
x = [0/3; 0/6; 0/6];
[p,x] = parasitic_motion(p,x);
T = posture_matrix(p,x); % p是末端位置，x是末端姿态
% B_temp = T*[B;1 1 1 1 1];
% B0 = B_temp(1:3,:);
% for i = 1:5
%     l_0(i) = norm(B0(1:3,i) - A(1:3,i));
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 局部旋量配置
% 支链1 SPR
theta_real1 = zeros(6,5);
T01 = [0 0 1 A(1,1);
       0 1 0 A(2,1);
       -1 0 0 A(3,1);
       0 0 0 1];
sp11 = se3ToVec(logm(T01));
theta_real1(1:3,1) = [0;0;1]; theta_real1(4:6,1) = -cross(theta_real1(1:3,1),[0;0;0]);
T12 = [1 0 0 0;0 0 1 0;0 -1 0 0;0 0 0 1];
sp12 = se3ToVec(logm(T12));
theta_real1(1:3,2) = [0;0;1]; theta_real1(4:6,2) = -cross(theta_real1(1:3,2),[0;0;0]);
T23 = [0 0 -1 0;0 1 0 0;1 0 0 0;0 0 0 1];
sp13 = se3ToVec(logm(T23));
theta_real1(1:3,3) = [0;0;1]; theta_real1(4:6,3) = -cross(theta_real1(1:3,3),[0;0;0]);
T34 = eye(4);
sp14 = se3ToVec(logm(T34));
theta_real1(1:3,4) = [0;0;0]; theta_real1(4:6,4) = [0;0;1];
T45 = [1 0 0 0;
       0 0 -1 0;
       0 1 0 130.98/1000;
       0 0 0 1];
sp15 = se3ToVec(logm(T45));
theta_real1(1:3,5) = [0;0;1]; theta_real1(4:6,5) = -cross(theta_real1(1:3,5),[0;0;0]);
T56 = [0 1 0 -0.07; 0 0 1 0;1 0 0 0;0 0 0 1];
sp16 = se3ToVec(MatrixLog6(T56));
% UPS
% 设计一个针对UPS支链的计算函数，输入为6列旋量，size为6x6，输出为末端位姿
% 支链 2
theta_real2 = zeros(6,6);
T01 = [0 0 1 A(1,2);
       0 1 0 A(2,2);
      -1 0 0 A(3,2);
       0 0 0 1];
sp21 = se3ToVec(logm(T01));
theta_real2(1:3,1) = [0;0;1]; theta_real2(4:6,1) = -cross(theta_real2(1:3,1),[0;0;0]);
T12 = [1 0 0 0;0 0 1 0;0 -1 0 0;0 0 0 1];
sp22 = se3ToVec(logm(T12));
theta_real2(1:3,2) = [0;0;1]; theta_real2(4:6,2) = -cross(theta_real2(1:3,2),[0;0;0]);
T23 = [0 0 -1 -130.98/1000;0 1 0 0;1 0 0 0;0 0 0 1];
sp23 = se3ToVec(logm(T23));
theta_real2(1:3,3) = [0;0;0]; theta_real2(4:6,3) = [0;0;1];
T34 = eye(4);
sp24 = se3ToVec(logm(T34));
theta_real2(1:3,4) = [0;0;1]; theta_real2(4:6,4) = -cross(theta_real2(1:3,4),[0;0;0]);
T45 = [1 0 0 0;0 0 -1 0;0 1 0 0;0 0 0 1];
sp25 = se3ToVec(logm(T45));
theta_real2(1:3,5) = [0;0;1]; theta_real2(4:6,5) = -cross(theta_real2(1:3,5),[0;0;0]);
T56 = [0 0 1 0;0 1 0 0;-1 0 0 0;0 0 0 1];
sp26 = se3ToVec(logm(T56));
theta_real2(1:3,6) = [0;0;1]; theta_real2(4:6,6) = -cross(theta_real2(1:3,6),[0;0;0]);
T67 =[-1 0 0 0.0681;0 0 1 0;0 1 0 -0.022;0 0 0 1]; % -30 -70 15
sp27 = se3ToVec(MatrixLog6(T67));
% 支链 3
theta_real3 = zeros(6,6);
T01 = [0 0 1 A(1,3);
       0 1 0 A(2,3);
       -1 0 0 A(3,3);
       0 0 0 1];
sp31 = se3ToVec(logm(T01));
theta_real3(1:3,1) = [0;0;1]; theta_real3(4:6,1) = -cross(theta_real3(1:3,1),[0;0;0]);
T12 = [1 0 0 0;0 0 1 0;0 -1 0 0;0 0 0 1];
sp32 = se3ToVec(logm(T12));
theta_real3(1:3,2) = [0;0;1]; theta_real3(4:6,2) = -cross(theta_real3(1:3,2),[0;0;0]);
T23 = [0 0 -1 -130.98/1000;0 1 0 0;1 0 0 0;0 0 0 1];
sp33 = se3ToVec(logm(T23));
theta_real3(1:3,3) = [0;0;0]; theta_real3(4:6,3) = [0;0;1];
T34 = eye(4);
sp34 = se3ToVec(logm(T34));
theta_real3(1:3,4) = [0;0;1]; theta_real3(4:6,4) = -cross(theta_real3(1:3,4),[0;0;0]);
T45 = [1 0 0 0;0 0 -1 0;0 1 0 0;0 0 0 1];
sp35 = se3ToVec(logm(T45));
theta_real3(1:3,5) = [0;0;1]; theta_real3(4:6,5) = -cross(theta_real3(1:3,5),[0;0;0]);
T56 = [0 0 1 0;0 1 0 0;-1 0 0 0;0 0 0 1];
sp36 = se3ToVec(logm(T56));
theta_real3(1:3,6) = [0;0;1]; theta_real3(4:6,6) = -cross(theta_real3(1:3,6),[0;0;0]);
T67 =[-1 0 0 0.0681;0 0 1 0;0 1 0 0.022;0 0 0 1];
sp37 = se3ToVec(MatrixLog6(T67));

% 支链 4
theta_real4 = zeros(6,6);
T01 = [0 0 1 A(1,4);
       0 1 0 A(2,4);
       -1 0 0 A(3,4);
       0 0 0 1];
sp41 = se3ToVec(logm(T01));
theta_real4(1:3,1) = [0;0;1]; theta_real4(4:6,1) = -cross(theta_real4(1:3,1),[0;0;0]);
T12 = [1 0 0 0;0 0 1 0;0 -1 0 0;0 0 0 1];
sp42 = se3ToVec(logm(T12));
theta_real4(1:3,2) = [0;0;1]; theta_real4(4:6,2) = -cross(theta_real4(1:3,2),[0;0;0]);
T23 = [0 0 -1 -130.98/1000;0 1 0 0;1 0 0 0;0 0 0 1];
sp43 = se3ToVec(logm(T23));
theta_real4(1:3,3) = [0;0;0]; theta_real4(4:6,3) = [0;0;1];
T34 = eye(4);
sp44 = se3ToVec(logm(T34));
theta_real4(1:3,4) = [0;0;1]; theta_real4(4:6,4) = -cross(theta_real4(1:3,4),[0;0;0]);
T45 = [1 0 0 0;0 0 -1 0;0 1 0 0;0 0 0 1];
sp45 = se3ToVec(logm(T45));
theta_real4(1:3,5) = [0;0;1]; theta_real4(4:6,5) = -cross(theta_real4(1:3,5),[0;0;0]);
T56 = [0 0 1 0;0 1 0 0;-1 0 0 0;0 0 0 1];
sp46 = se3ToVec(logm(T56));
theta_real4(1:3,6) = [0;0;1]; theta_real4(4:6,6) = -cross(theta_real4(1:3,6),[0;0;0]);
T67 =[-1 0 0 -0.0681;0 0 1 0;0 1 0 0.022;0 0 0 1];
sp47 = se3ToVec(MatrixLog6(T67));
% 支链 5
theta_real5 = zeros(6,6);
T01 = [0 0 1 A(1,5);
       0 1 0 A(2,5);
       -1 0 0 A(3,5);
       0 0 0 1];
sp51 = se3ToVec(logm(T01));
theta_real5(1:3,1) = [0;0;1]; theta_real5(4:6,1) = -cross(theta_real5(1:3,1),[0;0;0]);
T12 = [1 0 0 0;0 0 1 0;0 -1 0 0;0 0 0 1];
sp52 = se3ToVec(logm(T12));
theta_real5(1:3,2) = [0;0;1]; theta_real5(4:6,2) = -cross(theta_real5(1:3,2),[0;0;0]);
T23 = [0 0 -1 -130.98/1000;0 1 0 0;1 0 0 0;0 0 0 1];
sp53 = se3ToVec(logm(T23));
theta_real5(1:3,3) = [0;0;0]; theta_real5(4:6,3) = [0;0;1];
T34 = eye(4);
sp54 = se3ToVec(logm(T34));
theta_real5(1:3,4) = [0;0;1]; theta_real5(4:6,4) = -cross(theta_real5(1:3,4),[0;0;0]);
T45 = [1 0 0 0;0 0 -1 0;0 1 0 0;0 0 0 1];
sp55 = se3ToVec(logm(T45));
theta_real5(1:3,5) = [0;0;1]; theta_real5(4:6,5) = -cross(theta_real5(1:3,5),[0;0;0]);
T56 = [0 0 1 0;0 1 0 0;-1 0 0 0;0 0 0 1];
sp56 = se3ToVec(logm(T56));
theta_real5(1:3,6) = [0;0;1]; theta_real5(4:6,6) = -cross(theta_real5(1:3,6),[0;0;0]);
T67 =[-1 0 0 -0.0681;0 0 1 0;0 1 0 -0.022;0 0 0 1];
sp57 = se3ToVec(MatrixLog6(T67));

screw_local_T = {[sp11,sp12,sp13,sp14,sp15,sp16];
                  [sp21,sp22,sp23,sp24,sp25,sp26,sp27];
                  [sp31,sp32,sp33,sp34,sp35,sp36,sp37];
                  [sp41,sp42,sp43,sp44,sp45,sp46,sp47];
                  [sp51,sp52,sp53,sp54,sp55,sp56,sp57];};
screw_local = {theta_real1;theta_real2;theta_real3;theta_real4;theta_real5};
% 带误差的参数
d_par = 1e-3;
% d_par = 0;
screw_local_T_real = {[sp11,sp12,sp13,sp14,sp15,sp16]+se3ToVec(MatrixLog6(posture_matrix(d_par*rands(3,1),d_par*rands(3,1))));
                      [sp21,sp22,sp23,sp24,sp25,sp26,sp27]+se3ToVec(MatrixLog6(posture_matrix(d_par*rands(3,1),d_par*rands(3,1))));
                      [sp31,sp32,sp33,sp34,sp35,sp36,sp37]+se3ToVec(MatrixLog6(posture_matrix(d_par*rands(3,1),d_par*rands(3,1))));
                      [sp41,sp42,sp43,sp44,sp45,sp46,sp47]+se3ToVec(MatrixLog6(posture_matrix(d_par*rands(3,1),d_par*rands(3,1))));
                      [sp51,sp52,sp53,sp54,sp55,sp56,sp57]+se3ToVec(MatrixLog6(posture_matrix(d_par*rands(3,1),d_par*rands(3,1))));};
d_par1 = 1e-6;
screw_local_T_init = {[sp11,sp12,sp13,sp14,sp15,sp16]+(rand(6,6)-rand(6,6))*d_par1*0;
                      [sp21,sp22,sp23,sp24,sp25,sp26,sp27]+(rand(6,7)-rand(6,7))*d_par1;
                      [sp31,sp32,sp33,sp34,sp35,sp36,sp37]+(rand(6,7)-rand(6,7))*d_par1;
                      [sp41,sp42,sp43,sp44,sp45,sp46,sp47]+(rand(6,7)-rand(6,7))*d_par1;
                      [sp51,sp52,sp53,sp54,sp55,sp56,sp57]+(rand(6,7)-rand(6,7))*d_par1;};

clear sp11 sp12 sp13 sp14 sp15 sp16 sp21 sp22 sp23 sp24 sp25 sp26 sp27 sp31 sp32 sp33 sp34 sp35 sp36 sp37 sp41 sp42 sp43 sp44 sp45 sp46 sp47 sp51 sp52 sp53 sp54 sp55 sp56 sp57
clear theta_real1 theta_real2 theta_real3 theta_real4 theta_real5
clear T01 T12 T23 T34 T45 T56 T67
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 全局坐标系下的关节旋量
screw_ideal = cell(1,5);
screw_ideal_init0 = cell(1,5);
screw_real = cell(1,5);
for i = 1:5
    screw_ideal{i} = local2globalPOE(screw_local{i},screw_local_T{i});
    screw_ideal_init0{i} = local2globalPOE(screw_local{i},screw_local_T_init{i});
    screw_real{i} = local2globalPOE(screw_local{i},screw_local_T_real{i});
end
T_end_ideal = zeros(4*5,4);
T_end_real = zeros(4*5,4);
T_end_init = zeros(4*5,4);
for i = 1:5
    T_end_ideal(4*i-3:4*i,:) = local_POE(screw_local_T{i});
    T_end_real(4*i-3:4*i,:) = local_POE(screw_local_T_real{i});
    T_end_init(4*i-3:4*i,:) = local_POE(screw_local_T_init{i});
    [theta{i},success] = IKinSpace(screw_ideal{i},T_end_ideal(4*i-3:4*i,:),T,theta0{i},1e-12,1e-12);
    [theta{i},success1] = IKinSpace(screw_ideal_init0{i},T_end_init(4*i-3:4*i,:),T,theta0{i},1e-12,1e-12);
%     [success success1]
end
% screw_local 关节旋量在局部坐标系下的表征（单位旋量）
% screw_local_T 相邻关节之间的转换矩阵（旋量）
% screw_local_T_real 带误差的相邻关节之间的转换矩阵（旋量）
% screw_ideal 全局坐标系下的关节旋量（旋量）
% screw_real 全局坐标系下带误差的关节旋量（旋量）
% T_end_ideal 初始位姿下的支链末端位姿
% T_end_real 初始位姿下的带误差的支链末端位姿
% theta 初始位姿下的关节转角
% 旋量轴可以用旋量表示
% 相邻关节之间的坐标变化可以用旋量表示







