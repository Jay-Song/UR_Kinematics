clc
clear all
close all

%% UR3 Kinematics Configurations
% th3, th4 = 0~-180

global link;


joint_angle_deg = [-5 -10 -5 -10 -5 -10 -5];
joint_angle_rad = joint_angle_deg * pi /180.0

% UR 10 positive y
link(1).pose_from_prev = [0;  0.000; 0.000]; link(1).joint_angle = 0; link(1).joint_axis = [0; 0; 1]; link(1).joint_dir = 1; link(1).pos = [0; 0; 0]; link(1).rot = eye(3);
link(2).pose_from_prev = [0;  0.000; 0.360]; link(2).joint_angle = 0; link(2).joint_axis = [0; 1; 0]; link(2).joint_dir = 1; link(2).pos = [0; 0; 0]; link(2).rot = eye(3);
link(3).pose_from_prev = [0;  0.000; 0.210]; link(3).joint_angle = 0; link(3).joint_axis = [0; 0; 1]; link(3).joint_dir = 1; link(3).pos = [0; 0; 0]; link(3).rot = eye(3);
link(4).pose_from_prev = [0;  0.000; 0.210]; link(4).joint_angle = 0; link(4).joint_axis = [0; 1; 0]; link(4).joint_dir = 1; link(4).pos = [0; 0; 0]; link(4).rot = eye(3);
link(5).pose_from_prev = [0;  0.000; 0.200]; link(5).joint_angle = 0; link(5).joint_axis = [0; 0; 1]; link(5).joint_dir = 1; link(5).pos = [0; 0; 0]; link(5).rot = eye(3);
link(6).pose_from_prev = [0;  0.000; 0.200]; link(6).joint_angle = 0; link(6).joint_axis = [0; 1; 0]; link(6).joint_dir = 1; link(6).pos = [0; 0; 0]; link(6).rot = eye(3);
link(7).pose_from_prev = [0;  0.000; 0.126]; link(7).joint_angle = 0; link(7).joint_axis = [0; 0; 1]; link(7).joint_dir = 1; link(7).pos = [0; 0; 0]; link(7).rot = eye(3);

%% UR3 Forward Kinematics
FK_Result = Forward(joint_angle_rad)

% DrawRobot;

 %% getJacobian & Weight Matrix and Nemurical IK
target_rot = FK_Result(1:3, 1:3);
target_pos = FK_Result(1:3, 4);

ja = [-1 -1 -1 -1 -1 -1 -1]' * pi/180.0;
Curr = Forward(ja')
curr_rot = Curr(1:3, 1:3);
curr_pos = Curr(1:3, 4);

I = eye(length(link),length(link));

j2_max = pi;
j2_min = -pi;

for n = 1:50
    err = CalcErr(target_rot, target_pos, curr_rot, curr_pos);
    J = CalcJacobian; % Calc Jacobian from FK result
    
    % Numerical IK (Damped Least Square)
    W = zeros(length(link),length(link));
    W0 = 50.0;
    ti = 0.02;
        
    if (ja(2) > j2_max)
        W(2,2) = W0;
    elseif((ja(2) <= j2_max) && (ja(2) >= j2_max - ti))
        W(2,2) = W0*0.5*(1+cos(pi*(j2_max - ja(2))/ti));
    elseif(ja(2) > j2_min + ti) && (ja(2) < j2_max - ti)
        W(2,2) = 0;
    elseif(ja(2) >= j2_min) && (ja(2) <= j2_min + ti)
        W(2,2) = W0*0.5*(1+cos(pi*(ja(2) - j2_min)/ti));
    else
       W(2,2) = W0;
    end


    ja_delta = inv(J'*J + W + 0.001*I)*J'*err;
    ja = ja + ja_delta;
    
    Curr = Forward(ja');
    curr_rot = Curr(1:3, 1:3);
    curr_pos = Curr(1:3, 4);
end

ja'
 Curr = Forward(ja')
