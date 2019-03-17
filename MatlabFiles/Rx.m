function R = Rx( angle_rad )

c = cos(angle_rad);
s = sin(angle_rad);

R = [1 0  0;
     0 c -s;
     0 s  c];
 
end

