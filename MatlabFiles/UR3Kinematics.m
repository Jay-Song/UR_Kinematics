clc
clear all
close all

%% UR3 Kinematics Configurations
% th3, th4 = 0~-180

global link;


% joint_angle_deg = [30 -25 -50 -20 25 10];
% joint_angle_rad = joint_angle_deg * pi /180.0
joint_angle_rad = [1.0472 -1.2833+1.5708 -0.7376 -2.6915 -1.5708 3.14]

% UR 3
% link(1).pose_from_prev = [0;  0.000; 0.000]; link(1).joint_angle = 0; link(1).joint_axis = [0; 0; 1]; link(1).joint_dir =  1; link(1).pos = [0; 0; 0]; link(1).rot = eye(3);
% link(2).pose_from_prev = [0; -0.120; 0.152]; link(2).joint_angle = 0; link(2).joint_axis = [0; 1; 0]; link(2).joint_dir = -1; link(2).pos = [0; 0; 0]; link(2).rot = eye(3);
% link(3).pose_from_prev = [0;  0.093; 0.244]; link(3).joint_angle = 0; link(3).joint_axis = [0; 1; 0]; link(3).joint_dir = -1; link(3).pos = [0; 0; 0]; link(3).rot = eye(3);
% link(4).pose_from_prev = [0; -0.083; 0.213]; link(4).joint_angle = 0; link(4).joint_axis = [0; 1; 0]; link(4).joint_dir = -1; link(4).pos = [0; 0; 0]; link(4).rot = eye(3);
% link(5).pose_from_prev = [0;  0.000; 0.083]; link(5).joint_angle = 0; link(5).joint_axis = [0; 0; 1]; link(5).joint_dir =  1; link(5).pos = [0; 0; 0]; link(5).rot = eye(3);
% link(6).pose_from_prev = [0; -0.082; 0.000]; link(6).joint_angle = 0; link(6).joint_axis = [0; 1; 0]; link(6).joint_dir = -1; link(6).pos = [0; 0; 0]; link(6).rot = eye(3);

% UR 10
% link(1).pose_from_prev = [0;  0.000; 0.000]; link(1).joint_angle = 0; link(1).joint_axis = [0; 0; 1]; link(1).joint_dir =  1; link(1).pos = [0; 0; 0]; link(1).rot = eye(3);
% link(2).pose_from_prev = [0; -0.176; 0.128]; link(2).joint_angle = 0; link(2).joint_axis = [0; 1; 0]; link(2).joint_dir = -1; link(2).pos = [0; 0; 0]; link(2).rot = eye(3);
% link(3).pose_from_prev = [0;  0.128; 0.612]; link(3).joint_angle = 0; link(3).joint_axis = [0; 1; 0]; link(3).joint_dir = -1; link(3).pos = [0; 0; 0]; link(3).rot = eye(3);
% link(4).pose_from_prev = [0; -0.116; 0.572]; link(4).joint_angle = 0; link(4).joint_axis = [0; 1; 0]; link(4).joint_dir = -1; link(4).pos = [0; 0; 0]; link(4).rot = eye(3);
% link(5).pose_from_prev = [0;  0.000; 0.116]; link(5).joint_angle = 0; link(5).joint_axis = [0; 0; 1]; link(5).joint_dir =  1; link(5).pos = [0; 0; 0]; link(5).rot = eye(3);
% link(6).pose_from_prev = [0; -0.092; 0.000]; link(6).joint_angle = 0; link(6).joint_axis = [0; 1; 0]; link(6).joint_dir = -1; link(6).pos = [0; 0; 0]; link(6).rot = eye(3);

% UR 10 positive y
link(1).pose_from_prev = [0;  0.000; 0.000]; link(1).joint_angle = 0; link(1).joint_axis = [0; 0; 1]; link(1).joint_dir =  1; link(1).pos = [0; 0; 0]; link(1).rot = eye(3);
link(2).pose_from_prev = [0;  0.176; 0.128]; link(2).joint_angle = 0; link(2).joint_axis = [0; 1; 0]; link(2).joint_dir = -1; link(2).pos = [0; 0; 0]; link(2).rot = eye(3);
link(3).pose_from_prev = [0;  0.128; 0.612]; link(3).joint_angle = 0; link(3).joint_axis = [0; 1; 0]; link(3).joint_dir = -1; link(3).pos = [0; 0; 0]; link(3).rot = eye(3);
link(4).pose_from_prev = [0;  0.116; 0.572]; link(4).joint_angle = 0; link(4).joint_axis = [0; 1; 0]; link(4).joint_dir = -1; link(4).pos = [0; 0; 0]; link(4).rot = eye(3);
link(5).pose_from_prev = [0;  0.000; 0.116]; link(5).joint_angle = 0; link(5).joint_axis = [0; 0; 1]; link(5).joint_dir =  1; link(5).pos = [0; 0; 0]; link(5).rot = eye(3);
link(6).pose_from_prev = [0; -0.092; 0.000]; link(6).joint_angle = 0; link(6).joint_axis = [0; 1; 0]; link(6).joint_dir = -1; link(6).pos = [0; 0; 0]; link(6).rot = eye(3);

%% UR3 Forward Kinematics
FK_Result = Forward(joint_angle_rad)

for i = 1:6
%     disp([num2str(i)]);
%     [link(i).rot link(i).pos ]
end

DrawRobot;
%% UR3 Inverse Kinematics
IK_result = Inverse(link(6).rot, link(6).pos, 0)


%% IK check
for i = 1:8
    FK_Result_for_check = Forward(IK_result(i, 1:6))
end

%% getJacobian & Nemurical IK
target_rot = FK_Result(1:3, 1:3);
target_pos = FK_Result(1:3, 4);

ja = [20 -15 -45 -10 20 5]' * pi/180.0;
Curr = Forward(ja');
curr_rot = Curr(1:3, 1:3);
curr_pos = Curr(1:3, 4);

I = eye(6,6);

for n = 1:10
    err = CalcErr(target_rot, target_pos, curr_rot, curr_pos);
    J = CalcJacobian; % Calc Jacobian from FK result
    
    % Numerical IK (Damped Least Square)
    ja = ja + J'*inv(J*J' + 0.001*I)*err;
    
    Curr = Forward(ja');
    curr_rot = Curr(1:3, 1:3);
    curr_pos = Curr(1:3, 4);
end


joint_angle_rad
ja'
target = [target_rot target_pos]
curr = [curr_rot curr_pos]
