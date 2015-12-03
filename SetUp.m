%% Final
%Aero 560
%Morgan Yost and Trent Voris

%% Set up for simulink
%clear all
ecc = .001;
a = 7800; %km
inc = 42; %degrees
raan = 10; %degrees
omega = 22; %degrees
theta =  0;
mu = 398600; %km^2 something
[rvect,vvect]=COES2rvd(a,ecc,inc,raan, omega, theta);
R_0 = rvect;
V_0 = vvect;
x = [1 0 0];
y = [0 1 0];
z = [0 0 1];
h_a = cross(rvect, vvect);
k_hat = -rvect/norm(rvect);
j_hat = -h_a/norm(h_a);
i_hat = cross(j_hat, k_hat);
Q= [i_hat j_hat k_hat]; %inertial to lvlh
DCM = [dot(x, i_hat) dot(x, j_hat) dot(x, k_hat);
       dot(y, i_hat) dot(y, j_hat) dot(y, k_hat);
       dot(z, i_hat) dot(z, j_hat) dot(z, k_hat)];
w =[5*pi/180; 0; -2*pi/180]; %rad/sec in body frame
wbn0 =DCM*w; %body to inertial assuming body is aligned with lvlh to start
%initial euler angles in ECI assuming s/c starts aligned with LVLH
m = 6.5;
r = .15;
h = .5;
J = [(1/12)*(3*r^2 +4*h^2) 0 0; 0 (1/12)*(3*r^2 +4*h^2) 0; 0 0 .5*m*r^2]; %kg m^2
Jinv = inv(J);
T = 2*pi*sqrt(a^3/mu); %sec
tprop = T; %one period
q = DCM2quat(DCM);
eta0 = q(1);
epsilon0 = q(2:4);
ts = 30; %s
zeta  = .65;
nf = log(.02)/(ts*zeta);
kd = J*2*zeta*nf;
kp = J*2*nf^2;
n= 1/T;
%% Run Simulink
sim('satPropWThrusterControl');
%% Make Plots
%{
close all
figure(1)
hold on
plot(quat.time,quat.data(:, 1))
plot(quat.time,quat.data(:, 2))
plot(quat.time,quat.data(:, 3))
plot(quat.time,quat.data(:, 4))
legend('eta', 'q1', 'q2', 'q3')
title('Spacecraft Quaternions for one Orbit')
xlabel('Time (s)')

figure(2)
hold on
plot(wbn.time, wbn.Data(:,1))
plot(wbn.time, wbn.Data(:,2))
plot(wbn.time, wbn.Data(:,3))
ylabel('Rad/s')
xlabel('s')
title('Body to Inertial Angular Velocity')
legend('omega x', 'omega y', 'omega z')

figure(3)
hold on
plot(q_command.time,quat.data(:, 1))
plot(q_command.time,quat.data(:, 2))
plot(q_command.time,quat.data(:, 3))
plot(q_command.time,quat.data(:, 4))
legend('eta', 'q1', 'q2', 'q3')
title('Command Quaternions for one Orbit')
xlabel('Time (s)')

figure(4)
hold on
plot(wbn.time, wbn.Data(:,1))
plot(wbn.time, wbn.Data(:,2))
plot(wbn.time, wbn.Data(:,3))
ylabel('Nm')
xlabel('s')
title('Commanded torque')
legend('X', 'y', 'omega z')

len = length(Tc.time)/2;
fprintf('Max torque on X axis %f \n', max(Tc.data(:,1)))
fprintf('Max torque on Y axis %f \n', max(Tc.data(:,2)))
fprintf('Max torque on Z axis %f \n', max(Tc.data(:,3)))
fprintf('When we ignore the 360 deg swivel\n')
fprintf('Max torque on X axis %f \n', max(Tc.data(1:len,1)))
fprintf('Max torque on Y axis %f \n', max(Tc.data(1:len,2)))
fprintf('Max torque on Z axis %f \n', max(Tc.data(1:len,3)))
fprintf('An ion thruster could deliver this kind of torque because\n');
fprintf('they are typically .5 N thrusters and a %f Nm torque would \n',...
    max(Tc.data(1:len,2)));
fprintf('be reasonable\n');

%}
