%% Set Veriables
%clear
path = '/Users/meskupie/Desktop/torpedoauv/Android_Torpedo/android_apps-kinetic/teleop/src/main/res/raw/';
%path = '../Android_Torpedo/android_apps-kinetic/teleop/src/main/res/raw/';
filename = 'test.txt';

% Translation
m = 4;
bx = 12;% assume thrust at 30N forward. Assume 3m/s SS. b = 100/3^2
by = 16;% guess
bz = 18;% guess
bx_v0 = 0.5;
by_v0 = 0.25;
bz_v0 = 0.25;

% Rotation
Ixx = 0.015;% assume I = m*r^2/2 = 3*(0.1)^2/2 
Iyy = 0.035;% assume I = m/12(3r^2+h^2) = 3/12(3*0.08^2+0.35^2)
Izz = 0.035;% same as above
br = 0.03;
bp = 0.05; % assume pitch thrusters produce 30N -> 3nm together. Assume SS at 1.5 flips/s. b = 3/(3*pi)^2
bw = 0.05;
br_v0 = 90*pi/180;
bp_v0 = 90*pi/180;
bw_v0 = 90*pi/180;

% Thrusters
sign = 1; %use this to reverse the direction of the vertical thrusters from the original direction
a = 0.0165; % mapping of thrust(n) to torque torque(n.m) = a*Trust, t = a*T. Taken from T200 data

TpU = 25;  % upper limit of primary thrusters(n)
TpL = -22; % lower limit of primary thrusters(n)
TsU = 15;   % upper limit of secondary thrusters(n)
TsL = -14;  % lower limit of secondary thrusters(n)
%TpU = 100;
%TpL = -100;
%TsU = 100;
%TsL = -100;

Tp_min_input = 0.2;
Tp_deadband = 0.1;
Ts_min_input = 0.2;
Ts_deadband = 0.1;

% Other
dt = 0.02;

% Shape
lax = 0.15;
lay = 0.1;
lbx = 0.12;
b = 20*pi/180;
syms c_sym 
c = double(solve(cos(c_sym)*a-sin(c_sym)*lbx == 0,c_sym));
%c = 0;
%disp(['Twist vertical thrusters by ',num2str(c*180/pi), ' degrees'])

%% Build Matrices
% X = [X dX Y dY Z dZ R dR P dP W dW]'
% U = [Ta Tb Tc Td Te Tf]'
% Ac=[0 1 0 0 0 0 0 0 0 0 0 0;
%     0 (-2*bx*bx_v0/m) 0 0 0 0 0 0 0 0 0 0;
%     0 0 0 1 0 0 0 0 0 0 0 0;
%     0 0 0 (-2*by*by_v0/m) 0 0 0 0 0 0 0 0;
%     0 0 0 0 0 1 0 0 0 0 0 0;
%     0 0 0 0 0 (-2*bz*bz_v0/m) 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 1 0 0 0 0;
%     0 0 0 0 0 0 0 (-2*br*br_v0/Ixx) 0 0 0 0;
%     0 0 0 0 0 0 0 0 0 1 0 0;
%     0 0 0 0 0 0 0 0 0 (-2*bp*bp_v0/Iyy) 0 0;
%     0 0 0 0 0 0 0 0 0 0 0 1;
%     0 0 0 0 0 0 0 0 0 0 0 (-2*bw*bw_v0/Izz)];
% 
% Bc=[0 0 0 0 0 0;
%     cos(b)/m cos(b)/m cos(b)/m cos(b)/m 0 0;
%     0 0 0 0 0 0;
%     sin(b)/m -sin(b)/m -sin(b)/m sin(b)/m -sin(c)/m sin(c)/m;
%     0 0 0 0 0 0;
%     0 0 0 0 cos(c)/m cos(c)/m;
%     0 0 0 0 0 0;
%     a*cos(b)/Ixx a*cos(b)/Ixx -a*cos(b)/Ixx -a*cos(b)/Ixx 0 0;
%     0 0 0 0 0 0;
%     a*sin(b)/Iyy -a*sin(b)/Iyy a*sin(b)/Iyy -a*sin(b)/Iyy -(cos(c)*lbx+sign*sin(c)*a)/Iyy (cos(c)*lbx-sign*sin(c)*a)/Iyy;
%     0 0 0 0 0 0;
%     (lay*cos(b)+lax*sin(b))/Izz -(lay*cos(b)+lax*sin(b))/Izz (lay*cos(b)+lax*sin(b))/Izz -(lay*cos(b)+lax*sin(b))/Izz (sign*cos(c)*a-sin(c)*lbx)/Izz -(sign*cos(c)*a-sin(c)*lbx)/Izz];

Ad=[1 dt 0 0 0 0 0 0 0 0 0 0;
    0 (1-2*bx*bx_v0*dt/m) 0 0 0 0 0 0 0 0 0 0;
    0 0 1 dt 0 0 0 0 0 0 0 0;
    0 0 0 (1-2*by*by_v0*dt/m) 0 0 0 0 0 0 0 0;
    0 0 0 0 1 dt 0 0 0 0 0 0;
    0 0 0 0 0 (1-2*bz*bz_v0*dt/m) 0 0 0 0 0 0;
    0 0 0 0 0 0 1 dt 0 0 0 0;
    0 0 0 0 0 0 0 (1-2*br*br_v0*dt/Ixx) 0 0 0 0;
    0 0 0 0 0 0 0 0 1 dt 0 0;
    0 0 0 0 0 0 0 0 0 (1-2*bp*bp_v0*dt/Iyy) 0 0;
    0 0 0 0 0 0 0 0 0 0 1 dt;
    0 0 0 0 0 0 0 0 0 0 0 (1-2*bw*bw_v0*dt/Izz)];

Bd=[0 0 0 0 0 0;
    cos(b)*dt/m cos(b)*dt/m cos(b)*dt/m cos(b)*dt/m 0 0;
    0 0 0 0 0 0;
    sin(b)*dt/m -sin(b)*dt/m -sin(b)*dt/m sin(b)*dt/m -sin(c)*dt/m sin(c)*dt/m;
    0 0 0 0 0 0;
    0 0 0 0 cos(c)*dt/m cos(c)*dt/m;
    0 0 0 0 0 0;
    a*cos(b)*dt/Ixx a*cos(b)*dt/Ixx -a*cos(b)*dt/Ixx -a*cos(b)*dt/Ixx 0 0;
    0 0 0 0 0 0;
    a*sin(b)*dt/Iyy -a*sin(b)*dt/Iyy a*sin(b)*dt/Iyy -a*sin(b)*dt/Iyy -(cos(c)*lbx+sin(c)*a)*dt/Iyy (cos(c)*lbx-sin(c)*a)*dt/Iyy;
    0 0 0 0 0 0;
    (lay*cos(b)+lax*sin(b))*dt/Izz -(lay*cos(b)+lax*sin(b))*dt/Izz (lay*cos(b)+lax*sin(b))*dt/Izz -(lay*cos(b)+lax*sin(b))*dt/Izz (cos(c)*a-sin(c)*lbx)*dt/Izz -(cos(c)*a-sin(c)*lbx)*dt/Izz];


% Are things controllable?
Cmd_rank = rank(ctrb(Ad,Bd));

%% Define controllers

X_pos_cost = 80;
X_vel_cost = 0;
Y_pos_cost = 10;
Y_vel_cost = 0.2;
Z_pos_cost = 25;
Z_vel_cost = 0.2;
R_pos_cost = 5;
R_vel_cost = 2;
P_pos_cost = 6;
P_vel_cost = 0.5;
W_pos_cost = 0.3;
W_vel_cost = 0.06;
Ta_cost = 2;
Tb_cost = 2;
Tc_cost = 2;
Td_cost = 2;
Te_cost = 5;
Tf_cost = 5;

% Scale costs by their expected valid range. This helps to linearize the tuning process

% Q matrix
X_pos_range = 1;
X_vel_range = 3;
YZ_pos_range = 0.5;
YZ_vel_range = 1.5;
RPW_pos_range = 40/180*pi;
RPW_vel_range = 200/180*pi;
Q_diag = [  X_pos_cost*(1/X_pos_range)^2
            X_vel_cost*(1/X_vel_range)^2
            Y_pos_cost*(1/YZ_pos_range)^2
            Y_vel_cost*(1/YZ_vel_range)^2
            Z_pos_cost*(1/YZ_pos_range)^2
            Z_vel_cost*(1/YZ_vel_range)^2
            R_pos_cost*(1/RPW_pos_range)^2
            R_vel_cost*(1/RPW_vel_range)^2
            P_pos_cost*(1/RPW_pos_range)^2
            P_vel_cost*(1/RPW_vel_range)^2
            W_pos_cost*(1/RPW_pos_range)^2
            W_vel_cost*(1/RPW_vel_range)^2];
Q = diag(Q_diag);

% R matrix
Tp_range = 30;
Ts_range = 15;
R_diag = [  Ta_cost*(1/Tp_range)^2
            Tb_cost*(1/Tp_range)^2
            Tc_cost*(1/Tp_range)^2
            Td_cost*(1/Tp_range)^2
            Te_cost*(1/Ts_range)^2
            Tf_cost*(1/Ts_range)^2];
R = diag(R_diag);

Kd = dlqr(Ad, Bd, Q, R);
Kd = round(Kd,4);

%% Build Controller File for Import
fileID = fopen([path,filename],'w');
for i = 1:12
    for j = 1:12
        fprintf(fileID,'%f\n',Ad(i,j));
    end
end
fprintf(fileID,'\n');
for i = 1:12
    for j = 1:6
        fprintf(fileID,'%f\n',Bd(i,j));
    end
end
fprintf(fileID,'\n');
for i = 1:6
    for j = 1:12
        fprintf(fileID,'%f\n',Kd(i,j));
    end
end
fclose(fileID);

%% Plot Individual Step Responses
t_f = 2;
vx = 2;
rpw_a = 60;
yz_d = 0.5;
%figure('name','Individual Responses of System','NumberTitle','off')
figure(1);

subplot(2,3,1)
setpoint = [(0:dt:t_f)',linspace(0,vx*t_f,t_f/dt+1)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
plot(setpoint(:,1),setpoint(:,2),':r',setpoint(:,1),ones(t_f/dt+1,1)*vx,':b',sim_state.time,sim_state.signals.values(:,1),'-r',sim_state.time,sim_state.signals.values(:,2),'-b');
title('X axis response (+ -> forward)'); xlabel('time (s)'); ylabel('value (m or m/s)'); legend('Setpoint Pos','Setpoint Vel','X Position','X Velocity');
axis([0,t_f,-0.5,6]);

subplot(2,3,2)
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*yz_d,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
plot(setpoint(:,1),setpoint(:,4),':r',sim_state.time,sim_state.signals.values(:,3),'-r',sim_state.time,sim_state.signals.values(:,4),'-b')
title('Y axis response (+ -> right)'); xlabel('time (s)'); ylabel('value (m or m/s)'); legend('Setpoint','Y Position','Y Velocity')
axis([0,t_f,-0.5,1.5]);

subplot(2,3,3)
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*yz_d,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
plot(setpoint(:,1),setpoint(:,6),':r',sim_state.time,sim_state.signals.values(:,5),'-r',sim_state.time,sim_state.signals.values(:,6),'-b')
title('Z axis response (+ -> down)'); xlabel('time (s)'); ylabel('value (m or m/s)'); legend('Setpoint','Z Position','Z Velocity')
axis([0,t_f,-0.5,1.5]);

subplot(2,3,4)
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
plot(setpoint(:,1),setpoint(:,8),':r',sim_state.time,sim_state.signals.values(:,7),'-r',sim_state.time,sim_state.signals.values(:,8),'-b')
title('Roll axis response (+ -> Clockwise)'); xlabel('time (s)'); ylabel('value (deg or deg/s)'); legend('Setpoint','Roll Angle','Roll Angular Rate')
axis([0,t_f,-0.75,6]);

subplot(2,3,5)
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
plot(setpoint(:,1),setpoint(:,10),':r',sim_state.time,sim_state.signals.values(:,9),'-r',sim_state.time,sim_state.signals.values(:,10),'-b')
title('Pitch axis response (+ -> Pitch Up)'); xlabel('time (s)'); ylabel('value (deg or deg/s)'); legend('Setpoint','Pitch Angle','Pitch Angular Rate')
axis([0,t_f,-0.75,6]);

subplot(2,3,6)
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
plot(setpoint(:,1),setpoint(:,12),':r',sim_state.time,sim_state.signals.values(:,11),'-r',sim_state.time,sim_state.signals.values(:,12),'-b')
title('Yaw axis response (+ -> Clockwise)'); xlabel('time (s)'); ylabel('value (deg or deg/s)'); legend('Setpoint','Yaw Angle','Yaw Angular Rate')
axis([0,t_f,-0.75,6]);

%% Plot Comparison to All Steps Done Together
setpoint = [(0:dt:t_f)',linspace(0,vx*t_f,t_f/dt+1)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*yz_d,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*yz_d,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
combined_sim_state = sim_state;
combined_sim_u = sim_u;
combined_sim_u_clipped = sim_u_clipped;
figure(2);

subplot(2,3,1)
setpoint = [(0:dt:t_f)',linspace(0,vx*t_f,t_f/dt+1)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
plot(setpoint(:,1),setpoint(:,2),':r',setpoint(:,1),ones(t_f/dt+1,1)*vx,':b',sim_state.time,sim_state.signals.values(:,1),'--r',sim_state.time,sim_state.signals.values(:,2),'--b',combined_sim_state.time,combined_sim_state.signals.values(:,1),'-r',combined_sim_state.time,combined_sim_state.signals.values(:,2),'-b');
title('X axis response (+ -> forward)'); xlabel('time (s)'); ylabel('value (m or m/s)'); legend('Setpoint Pos','Setpoint Vel','X Position Individual','X Velocity Individual','X Position Combined','X Velocity Combined');
axis([0,t_f,-0.5,6]);

subplot(2,3,2)
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*yz_d,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
plot(setpoint(:,1),setpoint(:,4),':r',sim_state.time,sim_state.signals.values(:,3),'--r',sim_state.time,sim_state.signals.values(:,4),'--b',combined_sim_state.time,combined_sim_state.signals.values(:,3),'-r',combined_sim_state.time,combined_sim_state.signals.values(:,4),'-b')
title('Y axis response (+ -> right)'); xlabel('time (s)'); ylabel('value (m or m/s)'); legend('Setpoint','Y Position Individual','Y Velocity Individual','Y Position Combined','Y Velocity Combined')
axis([0,t_f,-0.5,1.5]);

subplot(2,3,3)
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*yz_d,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
plot(setpoint(:,1),setpoint(:,6),':r',sim_state.time,sim_state.signals.values(:,5),'--r',sim_state.time,sim_state.signals.values(:,6),'--b',combined_sim_state.time,combined_sim_state.signals.values(:,5),'-r',combined_sim_state.time,combined_sim_state.signals.values(:,6),'-b')
title('Z axis response (+ -> down)'); xlabel('time (s)'); ylabel('value (m or m/s)'); legend('Setpoint','Z Position Individual','Z Velocity Individual','Z Position Combined','Z Velocity Combined')
axis([0,t_f,-0.5,1.5]);

subplot(2,3,4)
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
plot(setpoint(:,1),setpoint(:,8),':r',sim_state.time,sim_state.signals.values(:,7),'--r',sim_state.time,sim_state.signals.values(:,8),'--b',combined_sim_state.time,combined_sim_state.signals.values(:,7),'-r',combined_sim_state.time,combined_sim_state.signals.values(:,8),'-b')
title('Roll axis response (+ -> Clockwise)'); xlabel('time (s)'); ylabel('value (deg or deg/s)'); legend('Setpoint','Roll Angle Individual','Roll Angular Rate Individual','Roll Angle Combined','Roll Angular Rate Combined')
axis([0,t_f,-0.75,6]);

subplot(2,3,5)
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
plot(setpoint(:,1),setpoint(:,10),':r',sim_state.time,sim_state.signals.values(:,9),'--r',sim_state.time,sim_state.signals.values(:,10),'--b',combined_sim_state.time,combined_sim_state.signals.values(:,9),'-r',combined_sim_state.time,combined_sim_state.signals.values(:,10),'-b')
title('Pitch axis response (+ -> Pitch Up)'); xlabel('time (s)'); ylabel('value (deg or deg/s)'); legend('Setpoint','Pitch Angle Individual','Pitch Angular Rate Individual','Pitch Angle Combined','Pitch Angular Rate Combined')
axis([0,t_f,-0.75,6]);

subplot(2,3,6)
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
plot(setpoint(:,1),setpoint(:,12),':r',sim_state.time,sim_state.signals.values(:,11),'--r',sim_state.time,sim_state.signals.values(:,12),'--b',combined_sim_state.time,combined_sim_state.signals.values(:,11),'-r',combined_sim_state.time,combined_sim_state.signals.values(:,12),'-b')
title('Yaw axis response (+ -> Clockwise)'); xlabel('time (s)'); ylabel('value (deg or deg/s)'); legend('Setpoint','Yaw Angle Individual','Yaw Angular Rate Individual','Yaw Angle Combined','Yaw Angular Rate Combined')
axis([0,t_f,-0.75,6]);

%% Plot Thruster Commands When Steping
%setpoint = [(0:dt:t_f)',linspace(0,vx*t_f,t_f/dt+1)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
%setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*yz_d,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
%setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*yz_d,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
%setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
%setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
%setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0];

axis_range = 35;

figure(3);

setpoint = [(0:dt:t_f)',linspace(0,vx*t_f,t_f/dt+1)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
subplot(2,3,1)
plot(sim_u_clipped.time,sim_u_clipped.signals.values(:,1),'-r',sim_u_clipped.time,sim_u_clipped.signals.values(:,2),'-b',sim_u_clipped.time,sim_u_clipped.signals.values(:,3),'-g',sim_u_clipped.time,sim_u_clipped.signals.values(:,4),'-m',sim_u_clipped.time,sim_u_clipped.signals.values(:,5),'--r',sim_u_clipped.time,sim_u_clipped.signals.values(:,6),'--b')
title('X axis response (+ -> Forward)'); xlabel('time (s)'); ylabel('Thrust (N)'); legend('Main-Ta','Main-Tb','Main-Tc','Main-Td','Secondary-Te','Secondary-Tf')
axis([0,t_f,-axis_range,axis_range]);
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*yz_d,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
subplot(2,3,2)
plot(sim_u_clipped.time,sim_u_clipped.signals.values(:,1),'-r',sim_u_clipped.time,sim_u_clipped.signals.values(:,2),'-b',sim_u_clipped.time,sim_u_clipped.signals.values(:,3),'-g',sim_u_clipped.time,sim_u_clipped.signals.values(:,4),'-m',sim_u_clipped.time,sim_u_clipped.signals.values(:,5),'--r',sim_u_clipped.time,sim_u_clipped.signals.values(:,6),'--b')
title('Y axis response (+ -> right)'); xlabel('time (s)'); ylabel('Thrust (N)'); legend('Main-Ta','Main-Tb','Main-Tc','Main-Td','Secondary-Te','Secondary-Tf')
axis([0,t_f,-axis_range,axis_range]);
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*yz_d,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
subplot(2,3,3)
plot(sim_u_clipped.time,sim_u_clipped.signals.values(:,1),'-r',sim_u_clipped.time,sim_u_clipped.signals.values(:,2),'-b',sim_u_clipped.time,sim_u_clipped.signals.values(:,3),'-g',sim_u_clipped.time,sim_u_clipped.signals.values(:,4),'-m',sim_u_clipped.time,sim_u_clipped.signals.values(:,5),'--r',sim_u_clipped.time,sim_u_clipped.signals.values(:,6),'--b')
title('Z axis response (+ -> down)'); xlabel('time (s)'); ylabel('Thrust (N)'); legend('Main-Ta','Main-Tb','Main-Tc','Main-Td','Secondary-Te','Secondary-Tf')
axis([0,t_f,-axis_range,axis_range]);
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
subplot(2,3,4)
plot(sim_u_clipped.time,sim_u_clipped.signals.values(:,1),'-r',sim_u_clipped.time,sim_u_clipped.signals.values(:,2),'-b',sim_u_clipped.time,sim_u_clipped.signals.values(:,3),'-g',sim_u_clipped.time,sim_u_clipped.signals.values(:,4),'-m',sim_u_clipped.time,sim_u_clipped.signals.values(:,5),'--r',sim_u_clipped.time,sim_u_clipped.signals.values(:,6),'--b')
title('Roll axis response (+ -> Clockwise)'); xlabel('time (s)'); ylabel('Thrust (N)'); legend('Main-Ta','Main-Tb','Main-Tc','Main-Td','Secondary-Te','Secondary-Tf')
axis([0,t_f,-axis_range,axis_range]);
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
subplot(2,3,5)
plot(sim_u_clipped.time,sim_u_clipped.signals.values(:,1),'-r',sim_u_clipped.time,sim_u_clipped.signals.values(:,2),'-b',sim_u_clipped.time,sim_u_clipped.signals.values(:,3),'-g',sim_u_clipped.time,sim_u_clipped.signals.values(:,4),'-m',sim_u_clipped.time,sim_u_clipped.signals.values(:,5),'--r',sim_u_clipped.time,sim_u_clipped.signals.values(:,6),'--b')
title('Pitch axis response (+ -> Pitch Up)'); xlabel('time (s)'); ylabel('Thrust (N)'); legend('Main-Ta','Main-Tb','Main-Tc','Main-Td','Secondary-Te','Secondary-Tf')
axis([0,t_f,-axis_range,axis_range]);
setpoint = [(0:dt:t_f)',ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*0,ones(t_f/dt+1,1)*rpw_a*pi/180,ones(t_f/dt+1,1)*0];
sim('LQR_sim_non_linear_model.slx');
subplot(2,3,6)
plot(sim_u_clipped.time,sim_u_clipped.signals.values(:,1),'-r',sim_u_clipped.time,sim_u_clipped.signals.values(:,2),'-b',sim_u_clipped.time,sim_u_clipped.signals.values(:,3),'-g',sim_u_clipped.time,sim_u_clipped.signals.values(:,4),'-m',sim_u_clipped.time,sim_u_clipped.signals.values(:,5),'--r',sim_u_clipped.time,sim_u_clipped.signals.values(:,6),'--b')
title('Yaw axis response (+ -> Clockwise)'); xlabel('time (s)'); ylabel('Thrust (N)'); legend('Main-Ta','Main-Tb','Main-Tc','Main-Td','Secondary-Te','Secondary-Tf')
axis([0,t_f,-axis_range,axis_range]);

%% Plot Thruster Commands When Stepping Together
figure(4);
plot(combined_sim_u_clipped.time,combined_sim_u_clipped.signals.values(:,1),'-r',combined_sim_u_clipped.time,combined_sim_u_clipped.signals.values(:,2),'-b',combined_sim_u_clipped.time,combined_sim_u_clipped.signals.values(:,3),'-g',combined_sim_u_clipped.time,combined_sim_u_clipped.signals.values(:,4),'-m',combined_sim_u_clipped.time,combined_sim_u_clipped.signals.values(:,5),'--r',combined_sim_u_clipped.time,combined_sim_u_clipped.signals.values(:,6),'--b')
title('Combined Thruster Response'); xlabel('time (s)'); ylabel('Thrust (N)'); legend('Main-Ta','Main-Tb','Main-Tc','Main-Td','Secondary-Te','Secondary-Tf')
