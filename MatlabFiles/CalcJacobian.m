function J = CalcJacobian(  )

global link;

J = zeros(6,6);

target = link(6).pos;
for i = 1: 6
   joint_axis = link(i).joint_axis * link(i).joint_dir;
   
   a = link(i).rot * joint_axis; %joint axis in world frame
   
   J(:, i) = [cross(a, target - link(i).pos); a];
    
end


end

