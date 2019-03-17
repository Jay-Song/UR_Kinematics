function R = Ry( angle_rad )

c = cos(angle_rad);
s = sin(angle_rad);

R = [c 0  s;
     0 1  0;
    -s 0  c];
 
end

