function result = DrawRobot( )
global link;

hold on;
grid on;

X = [0 0 0 0 0 0 0 0 0];
Y = [0 0 0 0 0 0 0 0 0];
Z = [0 0 0 0 0 0 0 0 0];

a2 = link(2).link_offset(3);
a3 = link(3).link_offset(3);
a4 = link(4).link_offset(3);
a5 = link(5).link_offset(3);



X(1) = link(1).pos(1);  Y(1) = link(1).pos(2);  Z(1) = link(1).pos(3);    
X(2) = X(1) + link(1).rot(1,3)*a2; Y(2) = Y(1) + link(1).rot(2,3)*a2; Z(2) = Z(1) + link(1).rot(3,3)*a2;   
X(3) = link(2).pos(1);  Y(3) = link(2).pos(2);  Z(3) = link(2).pos(3);  
X(4) = X(3) + link(2).rot(1,3)*a3; Y(4) = Y(3) + link(2).rot(2,3)*a3; Z(4) = Z(3) + link(2).rot(3,3)*a3;
X(5) = link(3).pos(1);  Y(5) = link(3).pos(2);  Z(5) = link(3).pos(3);
X(6) = X(5) + link(3).rot(1,3)*a4; Y(6) = Y(5) + link(3).rot(2,3)*a4; Z(6) = Z(5) + link(3).rot(3,3)*a4;
X(7) = link(4).pos(1);  Y(7) = link(4).pos(2);  Z(7) = link(4).pos(3);
X(8) = link(5).pos(1);  Y(8) = link(5).pos(2);  Z(8) = link(5).pos(3);
X(9) = link(6).pos(1);  Y(9) = link(6).pos(2);  Z(9) = link(6).pos(3);

plot3(X(1),Y(1),Z(1),'ro');
plot3(X(1:3),Y(1:3),Z(1:3),'b');
plot3(X(3),Y(3),Z(3),'ro');
plot3(X(3:5),Y(3:5),Z(3:5), 'b');
plot3(X(5),Y(5),Z(5),'ro');
plot3(X(5:9),Y(5:9),Z(5:9), 'b');
plot3(X(7),Y(7),Z(7),'ro');
plot3(X(8),Y(8),Z(8),'ro');
plot3(X(9),Y(9),Z(9),'go');

%plot3(X(4),Y(4),Z(4), 'r0');

xlabel('x');
ylabel('y');
zlabel('z');

xlim([-2 2]);
ylim([-2 2]);
zlim([-0.5 1.5]);

view([0 0 1])

result = 1;
end

