clc
clear all
close all

%% UR3 Kinematics Configurations
% th3, th4 = 0~-180

global link;


joint_angle_deg = [-179.771 29.003 107.865 -26.539 96.354 28.929];
joint_angle_rad = joint_angle_deg * pi /180.0
% joint_angle_rad = [1.0472 -1.2833+1.5708 -0.7376 -2.6915 -1.5708 3.14]


% UR 10
link(1).pose_from_prev = [0;    0;   0]; link(1).joint_angle = 0; link(1).joint_axis = [0; 0; 1]; link(1).joint_dir =  1; link(1).pos = [0; 0; 0]; link(1).rot = eye(3);
link(2).pose_from_prev = [0; -176; 128]; link(2).joint_angle = 0; link(2).joint_axis = [0; 1; 0]; link(2).joint_dir = -1; link(2).pos = [0; 0; 0]; link(2).rot = eye(3);
link(3).pose_from_prev = [0;  128; 612]; link(3).joint_angle = 0; link(3).joint_axis = [0; 1; 0]; link(3).joint_dir = -1; link(3).pos = [0; 0; 0]; link(3).rot = eye(3);
link(4).pose_from_prev = [0; -116; 572]; link(4).joint_angle = 0; link(4).joint_axis = [0; 1; 0]; link(4).joint_dir = -1; link(4).pos = [0; 0; 0]; link(4).rot = eye(3);
link(5).pose_from_prev = [0;    0; 116]; link(5).joint_angle = 0; link(5).joint_axis = [0; 0; 1]; link(5).joint_dir =  1; link(5).pos = [0; 0; 0]; link(5).rot = eye(3);
link(6).pose_from_prev = [0;  -92;   0]; link(6).joint_angle = 0; link(6).joint_axis = [0; 1; 0]; link(6).joint_dir = -1; link(6).pos = [0; 0; 0]; link(6).rot = eye(3);


%% UR3 Forward Kinematics
FK_Result = Forward(joint_angle_rad)

%% UR3 Inverse Kinematics
IK_result = Inverse(link(6).rot, link(6).pos, 0)


