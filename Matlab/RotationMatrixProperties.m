clear

a = 0.3;
R1 = [ cos(a),-sin(a), 0     ;
       sin(a), cos(a), 0     ;
       0     , 0     , 1     ];
b = 0.3;
R2 = [ cos(b), 0     ,-sin(b);
       0     , 1     , 0     ;
       sin(b), 0     , cos(b)];
c = 0.3;
R3 = [ 1     , 0     , 0     ;
       0     , cos(c),-sin(c);
       0     , sin(c), cos(c)];
   
R = R1*R2*R3;

trace = R(1,1)+R(2,2)+R(3,3);
rot = acos((trace-1)/2);

[V,D] = eig(R);
R_scale = real(V*(D^0.5)*inv(V));

trace_scale = R_scale(1,1)+R_scale(2,2)+R_scale(3,3);
rot_scale = acos((trace_scale-1)/2);