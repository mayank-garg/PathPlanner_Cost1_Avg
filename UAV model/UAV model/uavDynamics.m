function dx = uavDynamics(t,x)
% dynamics of UAV is implemented here

global C uav iUav pid phid vwind awind

vwind = 0;
awind = 0;
% initialize derivative vector
dx = zeros(15,1);

  % UAV states
  pn     = x(1);  % inertial north coordinate (m)
  pe     = x(2);  % inertial east coordinate (m)
  h      = x(3);  % altitude in inertial coordinates (m)
  u      = x(4);  % velocity along body x-axis (m/s)
  v      = x(5);  % velocity along body y-axis (m/s)
  w      = x(6);  % velocity along body z-axis (m/s)
  phi    = x(7);  % roll angle
  theta  = x(8);  % pitch angle
  psi    = x(9);  % yaw angle
  p      = x(10); % roll rate
  q      = x(11); % pitch rate
  r      = x(12); % yaw rate
  xiDV   = x(13);   % velocity hold ctrl state
  xiDh   = x(14); % altitude hold ctrl state 
  xiDtheta = x(15); % theta hold ctrl state
  xiDphi = x(16);   % attitude hold ctrl state
  
  
   % calculate alpha, beta, and velocity
   alpha = atan2(w,u);
   beta  = atan2(v,sqrt(u^2 + w^2));
   V = sqrt(u^2 + v^2 + w^2);

   
   uav(iUav).V = V;

   % guidance
  wn      = vwind*cos(awind); % wind along the north coordinate
  we      = vwind*sin(awind); % wind along the east coordinate
%   psidotd = uavHeadingRate_rabbit(uav(iUav), wn, we);
   psidotd = lqruavHeadingRate(uav(iUav), wn, we);
%   psidotd = nlgluavHeadingRate(uav(iUav), wn, we);
%   psidotd = plosuavHeadingRate(uav(iUav), wn, we);
%   psidotd = vfuavHeadingRate(uav(iUav), wn, we);
%   psidotd = vfuavHeadingRate_circle(uav(iUav), wn, we);
%   psidotd = lqruavHeadingRate_circle(uav(iUav), wn, we);
%    psidotd = plosuavHeadingRate_circle(uav(iUav), wn, we);
%     psidotd = nlgluavHeadingRate_circle(uav(iUav), wn, we);
%psidotd = rabbituavHeadingRate_circle(uav(iUav), wn, we);
phid = 1.9*psidotd;
    
   
     % UAV inputs
  [delta_t,delta_e,delta_a,delta_r] = uavAutopilot(V,h,theta,q,phi,p,xiDV,xiDh,xiDtheta,xiDphi,phid);


%   delta_t
  
   % calculate logitudinal force derivatives (depends on alpha)
   CX0        = -C.CD_0 * cos(alpha)        + C.CL_0 * sin(alpha);
   CXalpha    = -C.CD_alpha * cos(alpha)    + C.CL_alpha * sin(alpha);
   CXq        = -C.CD_q * cos(alpha)        + C.CL_q * sin(alpha);
   CXelevator = -C.CD_elevator * cos(alpha) + C.CL_elevator * sin(alpha);
   
   CZ0        = -C.CD_0 * sin(alpha)        - C.CL_0 * cos(alpha);
   CZalpha    = -C.CD_alpha * sin(alpha)    - C.CL_alpha * cos(alpha);
   CZq        = -C.CD_q * sin(alpha)        - C.CL_q * cos(alpha);
   CZelevator = -C.CD_elevator * sin(alpha) - C.CL_elevator * cos(alpha);
   
   % calculate the forces
   fx = -C.m*C.g*sin(theta)          + 0.5*C.rho*V^2*C.S * ( CX0     + (CXalpha * alpha)  + ((CXq * C.cbar * q)/V)   + (CXelevator*delta_e) ) + 0.5*C.rho*C.Sprop * ((C.kmotor*delta_t)^2 - V^2);
   fy =  C.m*C.g*cos(theta)*sin(phi) + 0.5*C.rho*V^2*C.S * ( C.CY_0  + (C.CY_beta * beta) + ((C.CY_p * C.b * p)/(2*V)) + ((C.CY_r * C.b * r)/(2*V)) + (C.CY_aileron * delta_a) + (C.CY_rudder * delta_r) );
   fz =  C.m*C.g*cos(theta)*cos(phi) + 0.5*C.rho*V^2*C.S * ( CZ0     + (CZalpha * alpha)  + ((CZq * C.cbar * q)/V)   + (CZelevator*delta_e) );
   
   % calculate moments
   l = 0.5*C.rho*V^2*C.S * (C.b/2) * ( C.Cl_0  + (C.Cl_beta * beta)   + (C.Cl_p * C.b * p)/(2*V)  + (C.Cl_r * C.b * r)/(2*V) + (C.Cl_aileron * delta_a) + (C.Cl_rudder * delta_r) );
   m = 0.5*C.rho*V^2*C.S * C.cbar  * ( C.Cm_0  + (C.Cm_alpha * alpha) + ((C.Cm_q * C.cbar * q)/V) + (C.Cm_elevator * delta_e) );
   n = 0.5*C.rho*V^2*C.S * (C.b/2) * ( C.Cn_0  + (C.Cn_beta * beta)   + (C.Cn_p * C.b * p)/(2*V)  + (C.Cn_r * C.b * r)/(2*V) + (C.Cn_aileron * delta_a) + (C.Cn_rudder * delta_r) );
   
   % The UAV Dynamics
   
   % Navigation Equations
   pndot = cos(theta)*cos(psi) * u + (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)) * v + (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * w + wn;
   pedot = cos(theta)*sin(psi) * u + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)) * v + (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * w + we;
   hdot  = sin(theta) * u - sin(phi)*cos(theta) * v - cos(phi)*cos(theta) * w;
   
   % Force Equations
   udot = r*v - q*w + fx/C.m;
   vdot = p*w - r*u + fy/C.m;
   wdot = q*u - p*v + fz/C.m;
   
   % Kinematic Equations
   phidot   = p + q * sin(phi)*tan(theta) + r * cos(phi)*tan(theta);
   thetadot = q * cos(phi) - r * sin(phi);
   psidot   = q * sin(phi)/cos(theta) + r * cos(phi)/cos(theta);
   
   % Moment Equations
   pdot = C.Gamma1*p*q - C.Gamma2*q*r + C.Gamma3*l + C.Gamma4*n;
   qdot = C.Gamma5*p*r - C.Jxz/C.Jy*(p^2 - r^2) + m/C.Jy;
   rdot = C.Gamma7*p*q - C.Gamma1*q*r + C.Gamma4*l + C.Gamma8*n;

   % controller state
   xiDVdot = uav(iUav).V0-V;
   xiDhdot = uav(iUav).h0-h;
   xiDthetadot = (uav(iUav).h0-h)*pid.khP+xiDh*pid.khI-(theta-uav(iUav).theta0)*180/pi;
%    xiDthetadot = (uav.h0-h)*pid.khP-(theta)*180/pi;
   xiDphidot = (phid-phi)*180/pi;
   
  xdot = [pndot; pedot; hdot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot; xiDVdot; xiDhdot; xiDthetadot; xiDphidot];

  dx = xdot;