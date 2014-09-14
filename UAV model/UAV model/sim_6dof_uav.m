function uk = sim_6dof_uav(uavpos,uk)

% simulate the UAV for given inputs and using wind information

global uav Rmin dt nUav iUav Ceffort Derror crf derr

setUavParameters

setAutopilotGains

% number of UAVs
nUav = 1;
iUav = 1;

% initialize UAV states
uav = initUavStates_mission(uavpos);

% minimum turn radius corresponding to 12 deg/sec psidot
Rmin = uav(1).V0/(12*pi/180);



% time step
dt = .1;

% type omaneuver = 'stline';

crf = [];
derr = [];

% UAV dynamics
%options=odeset('reltol',1e-6,'abstol',1e-6);
[~,x] = ode45(@uavDynamics_mission,[0 dt],uav.x0);
uk = [uk; x(end,:)];
Ceffort = [Ceffort crf(end)];
Derror = [Derror derr(end)];