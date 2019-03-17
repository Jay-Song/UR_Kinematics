function [ FK_result ] = Forward( joint_angle_rad )

global link;

pos = [0 ; 0; 0];
rot = eye(3);

for i = 1:6
    link(i).joint_angle = joint_angle_rad(i);
    
    ja = link(i).joint_axis * link(i).joint_dir*link(i).joint_angle;

    pos = pos + rot*link(i).link_offset;
    rot = rot*Rz(ja(3))* Ry(ja(2)) * Rz(ja(1));
    
    link(i).pos = pos;
    link(i).rot = rot;
end

FK_result = [link(6).rot link(6).pos;
                   0 0 0    1       ];

end

