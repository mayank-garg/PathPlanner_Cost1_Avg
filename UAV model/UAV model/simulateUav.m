% simulate the UAV for given inputs

clear all; close all; clc

global uav Rmin dt nUav iUav maneuver hndl W0 W1

setUavParameters

setAutopilotGains

% number of UAVs
nUav = 1;

% initialize UAV states
uav = initUavStates;

% minimum turn radius corresponding to 12 deg/sec psidot
Rmin = uav(1).V0/(12*pi/180);

% initialize plot
% plotUav()
trackUav()

W0 = [0 0];
W1 = [-800 800];
theta = atan2(W1(2)-W0(2), W1(1)-W0(1));
theta_c = ang_wrap(atan2(W0(2)-W1(2), W0(1)-W1(1)));
plot([W0(1) W1(1)], [W0(2) W1(2)],'r'); hold on;

hndl = plot(800,800,'db');


% time of simulation
Tsim = 50;

% time step
dt = 1;

% type of maneuver
maneuver = 'stline';

u=[];
for t = 0:dt:Tsim-dt

       
    for iUav = 1:nUav
    
    % plot the UAV    
   % trackUav(iUav,uav(iUav).x0)
    
    % UAV dynamics
    [tout,x] = ode45(@uavDynamics,[t t+dt],uav(iUav).x0);
    uav(iUav).x0 = x(end,:);
        u = [u;x(end,:)];
    end
end
plot(u(:,1),u(:,2),'b');