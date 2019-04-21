function [ FK_result ] = Forward( joint_angle_rad )

global link;

pos = [0 ; 0; 0];
rot = eye(3);

for i = 1:6
    link(i).joint_angle = joint_angle_rad(i);
    ja = link(i).joint_axis * link(i).joint_dir*link(i).joint_angle;
   
    % t(0->i) = t(0->i-1) * R(i-1->i)*pose_from_previous
    pos = pos + rot*link(i).pose_from_prev;
    
    % Only one elements must be one and Others must be zero in the joint_axis
    % R(0->i) = R(0->i-1) * R(i-1->i)
    rot = rot * Rz(ja(3)) * Ry(ja(2)) * Rx(ja(1));
    
    link(i).pos = pos;
    link(i).rot = rot;
end

FK_result = [link(6).rot link(6).pos;
                   0 0 0    1       ];

end

