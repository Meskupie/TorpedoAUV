%%
clear
load('T200_thruster_data.mat')

%%
% does the motor math make sense?
% compare battery voltage with power/current. Should be the same...
T200.forward.dvoltage = T200.forward.power./(T200.forward.current);
plot(1:41,T200.forward.dvoltage,'-r',1:41,T200.forward.voltage,'-b');
%looks good! (sorta)

%%
% lets try to calculate the motor torque a couple ways and compare.
% P = w*T = V*I, therefore, T = P/w, T = V*I/w
% a*w * (1/a)*T = V * I, (1/a)*T = I
T200.forward.torque = T200.forward.power./(T200.forward.rpm/60*2*pi);
a = T200.forward.torque./T200.forward.current;


%% 
figure(1)
plot(T200.forward.current,T200.forward.thrustkg)
xlabel('Torque (A)')
ylabel('Thrust (Kg)')

%%
figure(2)
a = 60;
plot(T200.forward.rpm,a*T200.forward.torque,'r-'); hold on
plot(T200.forward.rpm,T200.forward.thrustkg.*9.80665,'b-'); hold off
xlabel('RPM')

%%
figure(3)
plot(T200.forward.thrustkg,a*T200.forward.torque,'r-');


