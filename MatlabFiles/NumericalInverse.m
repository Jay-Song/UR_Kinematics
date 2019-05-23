clc
clear all
close all

%% UR3 Kinematics Configurations
% th3, th4 = 0~-180

global link;


joint_angle_deg = [10 10 20 10 20];
joint_angle_rad = joint_angle_deg * pi /180.0

% UR 10 positive y
link(1).pose_from_prev = [0;  0.000; 0.000]; link(1).joint_angle = 0; link(1).joint_axis = [0; 0; 1]; link(1).joint_dir =  1; link(1).pos = [0; 0; 0]; link(1).rot = eye(3);
link(2).pose_from_prev = [0;  0.100; 0.000]; link(2).joint_angle = 0; link(2).joint_axis = [0; 0; 1]; link(2).joint_dir = -1; link(2).pos = [0; 0; 0]; link(2).rot = eye(3);
link(3).pose_from_prev = [0;  0.100; 0.000]; link(3).joint_angle = 0; link(3).joint_axis = [0; 0; 1]; link(3).joint_dir = -1; link(3).pos = [0; 0; 0]; link(3).rot = eye(3);
link(4).pose_from_prev = [0;  0.100; 0.000]; link(4).joint_angle = 0; link(4).joint_axis = [0; 0; 1]; link(4).joint_dir = -1; link(4).pos = [0; 0; 0]; link(4).rot = eye(3);
link(5).pose_from_prev = [0;  0.100; 0.000]; link(5).joint_angle = 0; link(5).joint_axis = [0; 0; 1]; link(5).joint_dir = -1; link(4).pos = [0; 0; 0]; link(4).rot = eye(3);

%% UR3 Forward Kinematics
FK_Result = Forward(joint_angle_rad)

% DrawRobot;


%% getJacobian & Nemurical IK
target_rot = FK_Result(1:3, 1:3);
target_pos = FK_Result(1:3, 4);

ja = [1 1 1 1 1]' * pi/180.0;
Curr = Forward(ja');
curr_rot = Curr(1:3, 1:3);
curr_pos = Curr(1:3, 4);

I = eye(6,6);

for n = 1:20
    err = CalcErr(target_rot, target_pos, curr_rot, curr_pos);
    J = CalcJacobian; % Calc Jacobian from FK result
    
    % Numerical IK (Damped Least Square)
    ja = ja + J'*inv(J*J' + 0.001*I)*err;
    
    if (ja(3) > 5.0*pi/180.0)
        ja(3) = 5.0*pi/180.0;
    elseif (ja(3) < -5.0*pi/180.0)
        ja(3) = -5.0*pi/180.0;
    end

    Curr = Forward(ja');
    curr_rot = Curr(1:3, 1:3);
    curr_pos = Curr(1:3, 4);
end

ja'
 Curr = Forward(ja')

 %% getJacobian & Weight Matrix and Nemurical IK
target_rot = FK_Result(1:3, 1:3);
target_pos = FK_Result(1:3, 4);

ja = [1 1 1 1 1]' * pi/180.0;
Curr = Forward(ja');
curr_rot = Curr(1:3, 1:3);
curr_pos = Curr(1:3, 4);

I = eye(6,6);

for n = 1:20
    err = CalcErr(target_rot, target_pos, curr_rot, curr_pos);
    J = CalcJacobian; % Calc Jacobian from FK result
    
    % Numerical IK (Damped Least Square)
    ja = ja + J'*inv(J*J' + 0.001*I)*err;
    
    if (ja(3) > 5.0*pi/180.0)
        ja(3) = 5.0*pi/180.0;
    elseif (ja(3) < -5.0*pi/180.0)
        ja(3) = -5.0*pi/180.0;
    end

    Curr = Forward(ja');
    curr_rot = Curr(1:3, 1:3);
    curr_pos = Curr(1:3, 4);
end

ja'
 Curr = Forward(ja')
