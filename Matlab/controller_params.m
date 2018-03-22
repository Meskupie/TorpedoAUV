%% Drag Coefficients
clear all;close all;clc;

p = 1000;
rw = linspace(0,1.5,100);
pw = linspace(0,0.75,100);
yw = linspace(0,0.75,100);

%% Roll Torque
%1: tube, frame, vertduct, camera
%2: frame, tube
%3: duct

w_roll1 = 0.253;
w_roll2 = 0.229;
w_roll3 = 0.184;

r_roll1 = 0.044;
r_roll2 = 0.057;
r_roll3 = 0.125;

Cd_roll1 = 1.0;
Cd_roll2 = 1.0;
Cd_roll3 = 1.0;

Tr_coeff = 4*0.5*p/3*(Cd_roll1*w_roll1*(r_roll1^3)+Cd_roll2*w_roll2*(r_roll2^3-r_roll1^3)+Cd_roll3*w_roll3*(r_roll3^3-r_roll2^3));
Tr = Tr_coeff.*(rw.^2);

%% Pitch Torque
%1: tube, frame
%2: duct, vert duct
%3: camera

l_pitch1 = 0.184;
l_pitch2 = 0.229;
l_pitch3 = 0.253;

w_pitch1 = 0.57;
w_pitch2 = 0.125;
w_pitch3 = 0.044;

Cd_pitch1 = 1.0;
Cd_pitch2 = 1.0;
Cd_pitch3 = 1.0;

Tp_coeff = 4*0.5*p/3*(Cd_pitch1*w_pitch1*(l_pitch1^3)+Cd_pitch2*w_pitch2*(l_pitch2^3-l_pitch1^3)+Cd_pitch3*w_pitch3*(l_pitch3^3-l_pitch2^3));
Tp = Tp_coeff.*(pw.^2);


%% Yaw Torque
%1: tube
%2: frame
%3: duct
%4: camera

l_yaw1 = 0.130;
l_yaw2 = 0.175;
l_yaw3 = 0.233;
l_yaw4 = 0.253;

h_yaw1 = 0.056;
h_yaw2 = 0.030;
h_yaw3 = 0.038;
h_yaw4 = 0.020;

Cd_yaw1 = 1.0;
Cd_yaw2 = 1.0;
Cd_yaw3 = 1.0;
Cd_yaw4 = 1.0;

Ty_coeff = 4*0.5*p/3*(Cd_yaw1*h_yaw1*(l_yaw1^3)+Cd_yaw2*h_yaw2*(l_yaw2^3-l_yaw1^3)+Cd_yaw3*h_yaw3*(l_yaw3^3-l_yaw2^3)+Cd_yaw4*h_yaw4*(l_yaw4^3-l_yaw3^3));
Ty = Ty_coeff.*(yw.^2);

%% Plotting
figure(1)
plot(pw,Tp,rw,Tr,yw,Ty);
