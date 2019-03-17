function R = Rz( angle_rad )

c = cos(angle_rad);
s = sin(angle_rad);

R = [c -s  0;
     s  c  0;
     0  0  1];
 
end

