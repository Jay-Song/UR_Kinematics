clc
clear all
close all

%% UR3 Kinematics Configurations
% th3, th4 = 0~-180

global link;


 joint_angle_deg = [30 25 -45 10 5 10];
joint_angle_rad = joint_angle_deg * pi /180.0

link(1).link_offset = [0;  0.000; 0.000]; link(1).joint_angle = 0; link(1).joint_axis = [0; 0; 1]; link(1).joint_dir =  1; link(1).pos = [0; 0; 0]; link(1).rot = eye(3);
link(2).link_offset = [0; -0.120; 0.152]; link(2).joint_angle = 0; link(2).joint_axis = [0; 1; 0]; link(2).joint_dir = -1; link(2).pos = [0; 0; 0]; link(2).rot = eye(3);
link(3).link_offset = [0;  0.093; 0.244]; link(3).joint_angle = 0; link(3).joint_axis = [0; 1; 0]; link(3).joint_dir = -1; link(3).pos = [0; 0; 0]; link(3).rot = eye(3);
link(4).link_offset = [0; -0.083; 0.213]; link(4).joint_angle = 0; link(4).joint_axis = [0; 1; 0]; link(4).joint_dir = -1; link(4).pos = [0; 0; 0]; link(4).rot = eye(3);
link(5).link_offset = [0;  0.000; 0.083]; link(5).joint_angle = 0; link(5).joint_axis = [0; 0; 1]; link(5).joint_dir =  1; link(5).pos = [0; 0; 0]; link(5).rot = eye(3);
link(6).link_offset = [0; -0.082; 0.000]; link(6).joint_angle = 0; link(6).joint_axis = [0; 1; 0]; link(6).joint_dir = -1; link(6).pos = [0; 0; 0]; link(6).rot = eye(3);

%% UR3 Forward Kinematics
FK_Result = Forward(joint_angle_rad)

DrawRobot
%% UR3 Inverse Kinematics
IK_result = Inverse(link(6).rot, link(6).pos, 0)


%% IK check
for i = 1:8
    FK_Result_for_check = Forward(IK_result(i, 1:6))
end

