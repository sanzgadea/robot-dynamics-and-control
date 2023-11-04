% RO47001 Robot Dynamics & Control
% Department of Cognitive Robotics
% Faculty of Mechanical, Maritime and Materials Engineering
% Delft University of Technology, The Netherlands
%
clc; clear; close all;
%
%
%% Parameters
g = 9.81;                           % gravity constant, m/s^2
l_f = 1.10;                         % distance from front axle to CoG, m
l_r = 1.57;                         % distance from rear axle to CoG, m
L = l_f + l_r;                      % wheelbase, m
m = 1600;                           % full mass, kg
Izz = 2100;                         % inertia moment of vehicle about vertical axis, kgm^2
CwF = 57000;                        % front WHEEL cornering stiffness, N/rad
CwR = 47000;                        % rear WHEEL cornering stiffness, N/rad
n_points = 5000;                    % number of data points
t = linspace(0,5,n_points);         % simulation time, s
w = linspace(0.1,4,n_points)*2*pi;  % frequency range, rad
%
%% Question 1
% Calculate yaw rate response gain Gyaw versus longitudinal speed in the range from 0 to 200 km/h for the following configurations:
% 	a) baseline vehicle (provided data above);
%  	b) rear cornering stiffness of baseline configuration reduced by 25%;
% 	c) rear cornering stiffness of baseline configuration increased by 25%.
% Calculate understeer gradient Kus for each configuration and characteristic/critical speeds.
% Solution 1
V_x = linspace(0,200/3.6,1000);         % speed range, m/s

% YOUR CODE IS HERE

CaF = 2*CwF;
CaR = 2*CwR;

% Baseline 
CaR_b = CaR;
K_us(1) = (m*g/L) * (l_r/CaF - l_f/CaR_b);
r_ss(1,:) = V_x ./ (L + (K_us(1).*V_x.^2)./g);

% Reduced - Oversteer 
CaR_o = CaR * 0.75;
K_us(2) = (m*g/L) * (l_r/CaF - l_f/CaR_o);
r_ss(2,:) = V_x ./ (L + (K_us(2).*V_x.^2)./g);
V_char = sqrt((g*L)/-K_us(2));

% Increased - Understeer
CaR_u = CaR * 1.25;
K_us(3) = (m*g/L) * (l_r/CaF - l_f/CaR_u);
r_ss(3,:) = V_x ./ (L + (K_us(3).*V_x.^2)./g);
V_crit = sqrt((g*L)/K_us(3));


% Postprocessing
figure(2);
set(gcf,'Color','white');
hold all
plot(V_x*3.6,r_ss(1,:))
plot(V_x*3.6,r_ss(2,:))
plot(V_x*3.6,r_ss(3,:))
grid on
ylim([0 20])
xlabel('Longitudinal velocity, km/h');
ylabel('Yaw rate gain, 1/s');
set(gca,'FontSize',16)
legend('baseline', 'oversteer', 'understeer');
grid on;
%
%% Question 2-3
% Calculate and plot step and frequency response of the provided vehicle model:
% 	a) at 50 km/h;
%  	b) at 100 km/h;
% state vector x = [v r]
% control vector u = [delta]
% output vector y = [ay r beta v]
% delta - road steering angle, rad
% ay - lateral acceleration, m/s^2
% r - yaw rate, rad/s
% v - lateral velocity, m/s
% beta - sideslip angle, rad
Vx = 100 / 3.6;                  % vehicle speed, m/s
%
% HINT to set up 5 deg of road  wheel angle for step input after one sec: 
% delta = zeros(size(t)); ind = (t > 1) ; delta(ind) = 5 / 180*pi;
% 
% Prepare state space model of a bicycle model
%
%
% YOUR CODE IS HERE

Vx_list = [100/3.6 50/3.6];

r = zeros(length(Vx_list), length(t));
r_mag = zeros(length(Vx_list), length(w));
r_phase = zeros(length(Vx_list), length(w));

for i = 1:length(Vx_list)

    Vx = Vx_list(i);
    % State matrix
    A = [(-(CaF+CaR)/(m*Vx)) ((-l_f*CaF+l_r*CaR)/(m*Vx)-Vx); (l_r*CaR-l_f*CaF)/(Izz*Vx) -(l_f.^2*CaF+l_r.^2*CaR)/(Izz*Vx)];  
    % Control (input) matrix
    B = [CaF/m; l_f*CaF/Izz];  
    % Output matrix
    C = [(-(CaF+CaR)/(m*Vx)) (l_r*CaR-l_f*CaF)/(m*Vx); 0 1; 1/Vx 0; 1 0];
    % Feedthrough matrix
    D = [CaF/m; 0; 0; 0];  
    
    sys = ss(A, B, C, D);
    
    % Step response
    %
    % YOUR CODE IS HERE
    
    delta = zeros(size(t));
    ind = (t > 1);
    delta(ind) = 5 / 180 * pi; 
    
    % Step Response at 100km/h
    y = lsim(sys, delta, t);
    r(i,:) = y(:,2); 
    
 
    % Bode plot
    %
    % YOUR CODE IS HERE
    %
    
    H = freqresp(sys,w);
    r_mag(i,:) = 20*log10(abs(H(2, :)));
    r_phase(i,:) = rad2deg(angle(H(2, :)));
  

end

% Postprocessing
figure();
set(gcf,'Color','white');
plot(t,r,'LineWidth',1.5)
xlabel('Time, s');
ylabel('yaw rate, rad/s');
set(gca,'FontSize',16)
grid on
legend('100 km/h', '50 km/h')
%

% Postprocessing
figure();
subplot(2,1,1)
set(gcf,'Color','white');
semilogx(w/(2*pi),r_mag,'LineWidth',1.5)
xlim([0.1 4])
set(gca,'XTick',[0.1 0.3 1 2 3])
xlabel('Frequency, Hz');
ylabel('Magnitude');
set(gca,'FontSize',16)
legend('100 km/h', '50 km/h')
grid on;
subplot(2,1,2)
semilogx(w/(2*pi),r_phase,'LineWidth',1.5)
xlim([0.1 4])
set(gca,'XTick',[0.1 0.3 1 2 3])
xlabel('Frequency, Hz');
ylabel('Phase');
legend('100 km/h', '50 km/h')
grid on;
set(gca,'FontSize',16)


%
% Question 4
% Obstacle avoidance: PID control
Vx = 50 / 3.6; % vehicle speed, m/s
% Prepare state space model of vehicle-road model
%

A = [0 1 0 0; 0 -(CaF+CaR)/(m*Vx) (CaF+CaR)/m (l_r*CaR - l_f*CaF)/(m*Vx); 0 0 0 1; 0 (l_r*CaR - l_f*CaF)/(Izz*Vx) (l_f*CaF - l_r*CaR)/Izz -(l_f.^2*CaF+l_r.^2*CaR)/(Izz*Vx)];
B1 = [0; CaF/m; 0; l_f*CaF/Izz];
B2 = [0; (l_r*CaR - l_f*CaF)/(m*Vx)-Vx; 0; -(l_f.^2*CaF+l_r.^2*CaR)/(Izz*Vx)];

%%
% Reference lateral position
yd = -sin(2*pi*0.5*t);                  % sine signal definition
ind = (t < 1) | (t > 4); yd(ind) = 0;   % zero offsets
[b,a] = butter(1, 0.02 / (2 * pi));     % filter design
yd = filtfilt( b , a ,yd');             % trajectory smoothness
%
% PID design and tuning
%
% YOUR CODE IS HERE

Kp = 50; 
Ki = 1;
Kd = 10;

sys_cl = feedback(pid(Kp,Ki,Kd)*ss(A, B1, [1 0 0 0], 0), 1);

%
% Simulation of closed-loop system
y = lsim(sys_cl,yd,t);
% Calculate control input
demandP = Kp * (yd-y);
demandI = Ki * cumtrapz(yd-y) * 1e-3;
demandD = Kd * gradient(yd-y) / 1e-3;
u_demand = demandP + demandI + demandD;
%
% Postprocessing
figure
subplot(3,1,1)
set(gcf,'Color','white');
hold all
plot(t,yd,'b');
plot(t,y,'r');
xlabel('Time (s)')
ylabel('Lat. disp (m)')
legend('reference','actual')
set(gca,'FontSize',16)
grid on
subplot(3,1,2)
plot(t,yd-y,'b');
xlabel('Time (s)')
ylabel('Lat. offset (m)')
set(gca,'FontSize',16)
grid on
subplot(3,1,3)
plot(t,u_demand*180/pi,'b');
xlabel('Time (s)')
ylabel('Road wheel angle (deg)')
set(gca,'FontSize',16)
grid on