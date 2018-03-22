clear

Xb = [0, 2, 1, 4, 1;
      0, 2, 3, 2, 2;
      0, 2, 2, 1, 1];
  
x_std = 0.2;
y_std = 0.2;
z_std = 0.2;
trans_scale = 1;
rot_scale = 1;

a = 0.3;
R1ba = [ cos(a),-sin(a), 0     ;
         sin(a), cos(a), 0     ;
         0     , 0     , 1     ];
b = 0.3;
R2ba = [ cos(b), 0     ,-sin(b);
         0     , 1     , 0     ;
         sin(b), 0     , cos(b)];
c = 0.3;
R3ba = [ 1     , 0     , 0     ;
         0     , cos(c),-sin(c);
         0     , sin(c), cos(c)];

Rba = R1ba*R2ba*R3ba;
     
Tba = [1;2;1];

CentXb = [mean(Xb(1,:));
          mean(Xb(2,:));
          mean(Xb(3,:))];

for i = 1:length(Xb)
    Xex(:,i) = (Rba*(Xb(:,i)-CentXb))+CentXb+Tba;
    Xa(:,i) = Xex(:,i)+[randn*x_std;randn*y_std;randn*z_std];
end



CentXa = [mean(Xa(1,:));
          mean(Xa(2,:));
          mean(Xa(3,:))];
      
H = zeros(3);
      
for i = 1:length(Xb)
    H = H + (Xa(:,i)-CentXa)*(Xb(:,i)-CentXb)';    
end

[Usvd,Ssvd,Vsvd] = svd(H);
Rab = real(Vsvd*Usvd');

[Veig,Deig] = eig(Rab);
Rab_scale = real(Veig*(Deig^rot_scale)*inv(Veig));

Tab = CentXb-CentXa;
Tab_scale = Tab*trans_scale;


for i = 1:length(Xb)
    Xc(:,i) = (Rab_scale*(Xa(:,i)-CentXa))+CentXa+Tab_scale;
end

plot3(Xb(1,:),Xb(2,:),Xb(3,:),'*r',Xex(1,:),Xex(2,:),Xex(3,:),'*b',Xa(1,:),Xa(2,:),Xa(3,:),'*k',Xc(1,:),Xc(2,:),Xc(3,:),'*g');
%%
q_old = rotm2quat(Rba);

angle = acos((Rba(1,1)+Rba(2,2)+Rba(3,3)-1)/2);
axis(1,1) = Rba(3,2)-Rba(2,3);
axis(2,1) = Rba(1,3)-Rba(3,1);
axis(3,1) = Rba(2,1)-Rba(1,2);
axis = (1/(2*sin(angle)))*axis;
q_new(1,1) = cos(angle/2);
q_new(1,2) = axis(1,1)*sin(angle/2);
q_new(1,3) = axis(2,1)*sin(angle/2);
q_new(1,4) = axis(3,1)*sin(angle/2);

scale = 0.5;
[Veig,Deig] = eig(Rba);
Rab_scaled = real(Veig*(Deig^scale)*inv(Veig));
q_scaled_old = rotm2quat(Rab_scaled);


q_scaled_new(1,1) = cos(angle*scale/2);
q_scaled_new(1,2) = axis(1,1)*sin(angle*scale/2);
q_scaled_new(1,3) = axis(2,1)*sin(angle*scale/2);
q_scaled_new(1,4) = axis(3,1)*sin(angle*scale/2);









