function uav_ = initUavStates
% Initialize the states of UAV. 

% initial north and east of UAV
pn0 = 5;%500*cos(rand*2*pi);
pe0 = 0;%500*sin(rand*2*pi);

% destination x and y
   uav_.xt = 400*cos(rand*2*pi);
   uav_.yt = 400*sin(rand*2*pi);

   
% initial heading 
psi0 = pi/2;%2*pi*rand;

   
% other initial states    
V0 = 25;
h0 = 50;
alpha0 = 0.002375;
beta0 = 0;
gamma0 = 0;
phi0 = 0;
phidot0 = 0;
thetadot0 = 0;
psidot0 = 0;

theta0 = alpha0 + gamma0;



% assign the above values to UAV state structure

uav_.V0 = V0;  % initial airspeed
uav_.h0 = h0;  % altitude to hold
uav_.theta0 = theta0; % initial pitch

uav_.V = V0;


% rotation rates

rotRates = [1    0          -sin(theta0); 
            0  cos(phi0)  sin(phi0)*cos(theta0); 
            0 -sin(phi0)  cos(phi0)*cos(theta0)]*[phidot0 thetadot0 psidot0]';

        p0 = rotRates(1);
        q0 = rotRates(2);
        r0 = rotRates(3);

        
% UAV states        
        
uav_.x0 = [...
        pn0;...          % pn:     x-position (m)
        pe0;...          % pe:     y-position (m)
        h0;...         % h:     altitude (m)
        uav_.V0*cos(alpha0)*cos(beta0);... % u:     velocity along body x-axis (m/s)
        uav_.V0*sin(beta0);...             % v:     velocity along body y-axis (m/s)
        uav_.V0*sin(alpha0)*cos(beta0);... % w:     velocity along body z-axis (m/s)
        phi0;...          % phi:   roll angle
        theta0;...     % theta: pitch angle
        psi0;...          % psi:   yaw angle
        p0;...          % p:     roll rate
        q0;...          % q:     pitch rate
        r0;...          % r:     yaw rate
        0;              % velocity hold controller state
        0;              % altitude hold controller state
        0;              % theta hold controller state
        0;              % attitude hold controller state
       ];

   
end