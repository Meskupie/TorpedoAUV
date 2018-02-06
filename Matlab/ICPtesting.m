clear

Xb = [0, 2, 1, 0, 0, 0, 0, 0;
      0, 2, 3, 0, 0, 0, 0, 0];
  
x_std = 0.2;
y_std = 0.2;
trans_scale = 1;
rot_scale = 1;

a = 0.3;
Rba = [cos(a),-sin(a);
      sin(a), cos(a)];
Tba = [1;1];

CentXb = [mean(Xb(1,:));
          mean(Xb(2,:))];

for i = 1:3
    Xex(:,i) = (Rba*(Xb(:,i)-CentXb))+CentXb+Tba;
    Xa(:,i) = Xex(:,i)+[randn*x_std;randn*y_std];
end

for i = 4:length(Xb)
    Xa(:,i) = Xa(:,1);
end


CentXa = [mean(Xa(1,:));
          mean(Xa(2,:))];
      
H = zeros(2);
      
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

plot(Xb(1,:),Xb(2,:),'*r',Xex(1,:),Xex(2,:),'*b',Xa(1,:),Xa(2,:),'*k',Xc(1,:),Xc(2,:),'*g');

