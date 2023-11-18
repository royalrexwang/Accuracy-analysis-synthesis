% 目标精度范围（例如，末端执行器位置误差±0.01单位）
target_accuracy = 0.001;

% 机器人臂长
int_arm_length = 0.1; % 假设臂长为1单位

% 初始关节角度估计
initial_joint_angle = 0; % 弧度

% 定义优化选项
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

% 定义目标函数
objective = @(arm_length) objectiveFunction(initial_joint_angle, arm_length, target_accuracy);

% 定义非线性约束
nonlcon = [];

% 定义角度限制（例如，-π到π）
lb = -pi;
ub = pi;

% 使用fmincon求解
optimal_joint_angle = fmincon(objective, int_arm_length, [], [], [], [], -1, 1, nonlcon, options);

% 显示结果
fprintf('Optimal joint angle: %f radians\n', optimal_joint_angle);

function error = objectiveFunction(joint_angle, arm_length, target_accuracy)
    % 计算末端执行器的理想位置（假设理想位置为直接朝向x轴）
    ideal_position_x = 1;

    % 根据当前关节角度计算末端执行器的实际位置
    actual_position_x = arm_length * cos(joint_angle);
    actual_position_y = arm_length * sin(joint_angle);

    % 计算位置误差
    error = norm(norm([actual_position_x;actual_position_y] - [ideal_position_x;0]) - target_accuracy);
end
