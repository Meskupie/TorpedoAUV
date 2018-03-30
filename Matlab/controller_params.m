p = 1000;

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

%% Pitch Torque
%1: tube, frame
%2: duct, vert duct
%3: camera

l_pitch1 = 0.184;
l_pitch2 = 0.229;
l_pitch3 = 0.253;

w_pitch1 = 0.057;
w_pitch2 = 0.125;
w_pitch3 = 0.044;

Cd_pitch1 = 1.0;
Cd_pitch2 = 1.0;
Cd_pitch3 = 1.0;

Tp_coeff = 4*0.5*p/3*(Cd_pitch1*w_pitch1*(l_pitch1^3)+Cd_pitch2*w_pitch2*(l_pitch2^3-l_pitch1^3)+Cd_pitch3*w_pitch3*(l_pitch3^3-l_pitch2^3));

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

% Translation
m = 3.8;

area_front = 15900/1000000;
Cd_drag_front = 1.0;

area_side = 45500/1000000;
Cd_drag_side = 1.0;

area_top = 74200/1000000;
Cd_drag_top = 1.0;

bx = 0.5*p*Cd_drag_front*area_front;% assume thrust at 30N forward. Assume 3m/s SS. b = 100/3^2
by = 0.5*p*Cd_drag_side*area_side;% guess
bz = 0.5*p*Cd_drag_top*area_top;% guess
% bx_v0 = 0.5;
% by_v0 = 0.25;
% bz_v0 = 0.25;
bx_v0 = 0.0;
by_v0 = 0.0;
bz_v0 = 0.0;

% Rotation
Ixx = 0.0098;% assume I = m*r^2/2 = 3*(0.1)^2/2 
Iyy = 0.075;% assume I = m/12(3r^2+h^2) = 3/12(3*0.08^2+0.35^2)
Izz = 0.08;% same as above
br = Tr_coeff;
bp = Tp_coeff;
bw = Ty_coeff;
% br_v0 = 15*pi/180;
% bp_v0 = 15*pi/180;
% bw_v0 = 15*pi/180;
br_v0 = 0;
bp_v0 = 0;
bw_v0 = 0;

% Thrusters
sign = 1; %use this to reverse the direction of the vertical thrusters from the original direction
a = 0.0165; % mapping of thrust(n) to torque torque(n.m) = a*Trust, t = a*T. Taken from T200 data

TpU = 8;  % upper limit of primary thrusters(n)
TpL = -8; % lower limit of primary thrusters(n)
TsU = 4;   % upper limit of secondary thrusters(n)
TsL = -4;  % lower limit of secondary thrusters(n)
Tp_min_input = 0.1;
Tp_deadband = 0.05;
Ts_min_input = 0.1;
Ts_deadband = 0.05;

% TpU = 1000;  % upper limit of primary thrusters(n)
% TpL = -1000; % lower limit of primary thrusters(n)
% TsU = 1000;   % upper limit of secondary thrusters(n)
% TsL = -1000;  % lower limit of secondary thrusters(n)
% 
% Tp_min_input = 0.0;
% Tp_deadband = 0.0;
% Ts_min_input = 0.0;
% Ts_deadband = 0.0;
% Other
dt = 0.05;

% Shape
lax = 0.2;
lay = 0.1;
lbx = 0.175;
b = 20*pi/180;
syms c_sym 
c = 8;%double(solve(cos(c_sym)*a-sin(c_sym)*lbx == 0,c_sym));
%c = 0;
%disp(['Twist vertical thrusters by ',num2str(c*180/pi), ' degrees'])