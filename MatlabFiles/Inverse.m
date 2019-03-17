function [ IK_result ] = Inverse( rot, pos, q6_des )

global link;

th1 = 0;
th2 = 0;
th3 = 0;
th4 = 0;
th5 = 0;
th6 = 0;

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
d = abs(d2+d3+d5);
% p5
R = sqrt(p5(1)*p5(1) + p5(2)*p5(2))
% atan2(p5(2), p5(1)) 
% atan2(d/R, sqrt(abs(R*R - d*d))/R)


th1 = atan2(p5(2), p5(1)) + atan2(d/R, sqrt(abs(R*R - d*d))/R)

%if (d2+d3+d5) < 0
%    th1 = atan2(p5(2), p5(1)) + atan2(d/R, sqrt(abs(R*R - d*d))/R);
%else
%    th1 = atan2(p5(2), p5(1)) - atan2(d/R, sqrt(abs(R*R - d*d))/R);
%end

th1 = atan2(p5(2), p5(1)) - atan2(d/R, sqrt(abs(R*R - d*d))/R) +pi

if th1 > pi
    th1 = th1 - 2*pi;
elseif th1 < -pi
    th1 = th1 + 2*pi;
end

% get angle 5
c5 = -rot(1,2)*sin(th1) + rot(2,2)*cos(th1);
th5 = acos(c5);

% get angle 6 & angle 2, 3, 4
if abs(th5) > 2.0*eps
    c6_ = (-rot(1,1)*sin(th1) + rot(2,1)*cos(th1));
    s6_ = (-rot(1,3)*sin(th1) + rot(2,3)*cos(th1));
    th6 = atan2(s6_, c6_);
else
    th5 = 0;
    th6 = q6_des;
end

R5 = rot * Ry(-th6);
p4 = pos - [rot(1,2); rot(2,2); rot(3,2)]*(d6) - [R5(1,3); R5(2,3); R5(3,3)]*(a5);
p4 = Rz(-th1)*p4;
p4(3) = p4(3) - a2;
    
th3 = pi - acos(  (a3*a3 + a4*a4 - (p4(1)*p4(1) + p4(3)*p4(3)))/(2*a3*a4) );
th2 = 0.5*pi - ( atan2( p4(3), p4(1) ) + atan2( a4*sin(th3), a3 + a4*sin(th3) ) );

R34 = Ry(-th3)*Ry(-th2)*Rz(-th1)*rot*Ry(-th6)*Rz(-th5);
th4 = atan2(R34(1,3), R34(1,1));
  

IK_result = [th1, th2, th3, th4, th5, th6];

for i = 1:6
IK_result(i) = IK_result(i) * link(i).joint_dir;
end

end

