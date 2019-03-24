function [ IK_result ] = Inverse2( rot, pos, q6_des )

global link;

IK_result = zeros(8, 6);
a2 = link(2).link_offset(3);
a3 = link(3).link_offset(3);
a4 = link(4).link_offset(3);
a5 = link(5).link_offset(3);

d2 = link(2).link_offset(2);
d3 = link(3).link_offset(2);
d5 = link(4).link_offset(2);
d6 = link(6).link_offset(2);
p5 = rot*[0; -(d6); 0] + pos;

% get angle 1
d = d2+d3+d5;
R = sqrt(p5(1)*p5(1) + p5(2)*p5(2));

th1_1 = atan2(p5(2), p5(1)) - asin(d/R);

if (abs(th1_1) < eps)
    th1_1 = 0;
end

IK_result(1:4, 1) = th1_1;

th1_2 = atan2(p5(2), p5(1)) + asin(d/R) + pi;
if (abs(th1_2) < eps)
    th1_2 = 0;
end

IK_result(5:8, 1) = th1_2;

% get angle 5
c5 = -rot(1,2)*sin(th1_1) + rot(2,2)*cos(th1_1);
th5_1 = acos(c5);
if(abs(th5_1) < eps)
    th5_1 = 0;
end

c5 = -rot(1,2)*sin(th1_2) + rot(2,2)*cos(th1_2);
th5_2 = acos(c5);
if(abs(th5_2) < eps)
    th5_2 = 0;
end

IK_result(1:2, 5) = th5_1;
IK_result(3:4, 5) = -th5_1;
IK_result(5:6, 5) = th5_2;
IK_result(7:8, 5) = -th5_2;

% get angle 6 & angle 2, 3, 4
for i = 1:2:8
% for i = 1:4
    th1 = IK_result(i, 1);
    th5 = IK_result(i, 5);
   
    th6 = 0;
    if abs(th5) > eps
        c6 = (-rot(1,1)*sin(th1) + rot(2,1)*cos(th1))/sin(th5);
        s6 = (-rot(1,3)*sin(th1) + rot(2,3)*cos(th1))/sin(th5);
        th6 = atan2(s6, c6);
    else
        th5 = 0;
        th6 = q6_des;
    end
    IK_result(i:i+1,6) = th6;
    
    R5 = rot * Ry(-th6);
    p4 = pos - [rot(1,2); rot(2,2); rot(3,2)]*(d6) - [R5(1,3); R5(2,3); R5(3,3)]*(a5);
    p4 = Rz(-th1)*p4;
    p4(3) = p4(3) - a2;

    if (a3 + a4 - sqrt(p4(1)*p4(1) + p4(3)*p4(3))) < 0
        disp(['error ', num2str(i)])
        continue;
    end
    
    th3_1 =   pi - acos(  (a3*a3 + a4*a4 - (p4(1)*p4(1) + p4(3)*p4(3)))/(2*a3*a4) );
    th3_2 = - pi + acos(  (a3*a3 + a4*a4 - (p4(1)*p4(1) + p4(3)*p4(3)))/(2*a3*a4) );
    
    IK_result(i, 3) = th3_1;
    IK_result(i+1, 3) = th3_2;
    
     th2_1 = 0.5*pi - ( atan2( p4(3), p4(1) ) + atan2( a4*sin(th3_1), a3 + a4*sin(th3_1) ) );
     th2_2 = ( atan2( p4(3), p4(1) ) + atan2( a4*sin(th3_2), a3 + a4*sin(th3_2) ) );
     
     IK_result(i, 2) = th2_1;
     IK_result(i+1, 2) = th2_2;
     
     R34 = Ry(-th3_1)*Ry(-th2_1)*Rz(-th1)*rot*Ry(-th6)*Rz(-th5);
     th4_1 = atan2(R34(1,3), R34(1,1));

     R34 = Ry(-th3_2)*Ry(-th2_2)*Rz(-th1)*rot*Ry(-th6)*Rz(-th5);
     th4_2 = atan2(R34(1,3), R34(1,1));
     
     IK_result(i, 4) = th4_1;
     IK_result(i+1, 4) = th4_2;
end

for i = 1:8
    for j = 1:6
        IK_result(i,j) = IK_result(i,j) * link(j).joint_dir;
    end
end

end

