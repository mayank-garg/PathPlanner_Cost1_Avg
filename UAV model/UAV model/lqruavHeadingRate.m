function psidot = lqruavHeadingRate(uav, wn, we)
% function to find heading rate for a guidance law taking the wind effects
% also into account

global Rmin dt maneuver W0 W1

% assign states to simple variables for ease of coding

x = uav.x0;
px   = x(1);
py   = x(2);
v   = norm(x(4:6));
psi = x(9);

% target position
%xt   = uav.xt;
%yt   = uav.yt;

% varables
%eps = 1e-6;  % tolerence
%R   = Rmin;    % minimum radius of turn

% heading algorithm

%% type of maneuver


%% Control initialization
q22 = 1;

uwd = [v*cos(psi)+wn v*sin(psi)+we];
kiang = atan2(uwd(2), uwd(1));
vg = sqrt(uwd(1)^2+uwd(2)^2);
z = [px py];

% lqr guidance
theta = atan2(W1(2)-W0(2), W1(1)-W0(1));
R = sqrt((z(1)-W0(1))^2+(z(2)-W0(2))^2);
theta_M = atan2(z(2)-W0(2),z(1)-W0(1));
d = R*sin(theta-theta_M);

% ddot
vd = vg*sin(ang_wrap(kiang-theta));% + vwind*sin(awind-theta);

q11    =  abs(2/(2-abs(d)));
d = -d;

%u =  (exp(k*d/2) + sqrt(2*exp(k*d/2 + 1)*vd));
u = -(sqrt(q11)*d+sqrt(2*sqrt(q11)+q22)*vd);

%uavpos(1) = uavpos(1) + v*cos(uavpos(3)*dt+ vwind*cos(awind)*dt; % update the new uav x position
%uavpos(2) = uavpos(2) + v*sin(uavpos(3))*dt+ vwind*sin(awind)*dt; % update the new uav y positon

% limit the accelaration to the minimum turning radius
if abs(u) > vg^2/Rmin
    if u > 0
        u = vg^2/Rmin;
    else
        u = - vg^2/Rmin;
    end
end

% k = 1;
% psi_d = u/(k*vg)+kiang;
% psidot = k*(psi_d-psi);%
psidot = 0.8*u/vg;% update the heading direction
%plot(uavpos(1), uavpos(2),'k.');
%drawnow;
%uk  = [uk; uavpos u d];
%end
end


% % follow Dubins curve
% theta = atan2(yt-py,xt-px);
% if (theta<0) theta = theta + 2*pi; end
% turnDirection = sum(cross([cos(theta) sin(theta) 0], [cos(psi) sin(psi) 0]));
% if turnDirection > 0
%     xc = px + R*sin(psi);
%     yc = py - R*cos(psi);
%     dpsiSign = -1;
% else
%     xc = px - R*sin(psi);
%     yc = py + R*cos(psi);
%     dpsiSign = 1;
% end
% 
% ast = abs(psi-theta);
% if ast > pi
%     ast = 2*pi-ast;
% end
% 
% if ast < eps
%     dpsiSign = 0;
% end
% 
% kp = 0.3;
% 
% if ((xt-xc)^2+(yt-yc)^2)>R^2
%     psidot = dpsiSign*min(v/R,kp*ast/dt);
% else
%     psidot = -dpsiSign*min(v/R,kp*ast/dt);
% end
