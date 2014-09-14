%% Set the parameters of UAV
% Here we set UAV parameters which includes includes 
% moment of inertias, stability and control derivatives,
% propeller charateristics, uav dimensions, etc.

% set the parameters global so that it can be accessed by functions
global C

%% Mass

C.m = 1.56; % mass (kg)

%% Moment of inertias

% inertia in kg-m^2
C.Jx = 0.114700;
C.Jy = 0.057600;
C.Jz = 0.171200;
C.Jxz = 0.001500;

%%  Airframe physical parameters
  
C.b    = 1.422400;  % wingspan (m)
C.cbar = 0.330200;  % mean cord
C.S    = 0.258900;  % wing area

%%  propeller characteristics  
  
C.Sprop = pi*(0.1)^2; % prop area
C.kmotor = 20;
C.C_throttle = 0.8;   % aerodynamic efficiency of propeller

%% aerodynamic coefficients for linear force model
  
  C.CL_0        = 0.280000;
  C.CD_0        = 0.030000;
  C.Cm_0        = 0.000000;
  C.CL_alpha    = 3.450000;
  C.CD_alpha    = 0.300000;
  C.Cm_alpha    = -0.380000;
  C.CL_q        = 0.000000;
  C.CD_q        = 0.000000;
  C.Cm_q        = -3.600000;
  C.CL_elevator = -0.360000;
  C.CD_elevator = 0.000000;
  C.Cm_elevator = 0.500000;
  C.CY_0        = 0.000000;
  C.Cl_0        = 0.000000;
  C.Cn_0        = 0.000000;
  C.CY_beta     = -0.980000;
  C.Cl_beta     = -0.120000;
  C.Cn_beta     = 0.250000;
  C.CY_p        = 0.000000;
  C.Cl_p        = -0.260000;
  C.Cn_p        = 0.022000;
  C.CY_r        = 0.000000;
  C.Cl_r        = 0.140000;
  C.Cn_r        = -0.350000;
  C.CY_aileron  = 0.000000;
  C.Cl_aileron  = 0.080000;
  C.Cn_aileron  = 0.060000;
  C.CY_rudder   = -0.170000;
  C.Cl_rudder   = 0.105000;
  C.Cn_rudder   = -0.032000;

%%  Coefficients associated with inverse of J

  C.Gamma  = C.Jx*C.Jz - C.Jxz^2;
  C.Gamma1 = C.Jxz*(C.Jx - C.Jy + C.Jz)/C.Gamma;
  C.Gamma2 = (C.Jz*(C.Jz-C.Jy) + C.Jxz^2)/C.Gamma;
  C.Gamma3 = C.Jz/C.Gamma;
  C.Gamma4 = C.Jxz/C.Gamma;
  C.Gamma5 = (C.Jz-C.Jx)/C.Jy;
  C.Gamma6 = 1/C.Jy;
  C.Gamma7 = ((C.Jx-C.Jy)*C.Jx + C.Jxz^2)/C.Gamma;
  C.Gamma8 = C.Jx/C.Gamma;
  
%% Converting coefficients so that the roll and yaw channels decouple

  C.Cp_0       = C.Gamma3*C.Cl_0       + C.Gamma4*C.Cn_0;
  C.Cp_beta    = C.Gamma3*C.Cl_beta    + C.Gamma4*C.Cn_beta;
  C.Cp_p       = C.Gamma3*C.Cl_p       + C.Gamma4*C.Cn_p;
  C.Cp_r       = C.Gamma3*C.Cl_r       + C.Gamma4*C.Cn_r;
  C.Cp_aileron = C.Gamma3*C.Cl_aileron + C.Gamma4*C.Cn_aileron;
  C.Cp_rudder  = C.Gamma3*C.Cl_rudder  + C.Gamma4*C.Cn_rudder;

  C.Cr_0       = C.Gamma4*C.Cl_0       + C.Gamma7*C.Cn_0;
  C.Cr_beta    = C.Gamma4*C.Cl_beta    + C.Gamma7*C.Cn_beta;
  C.Cr_p       = C.Gamma4*C.Cl_p       + C.Gamma7*C.Cn_p;
  C.Cr_r       = C.Gamma4*C.Cl_r       + C.Gamma7*C.Cn_r;
  C.Cr_aileron = C.Gamma4*C.Cl_aileron + C.Gamma7*C.Cn_aileron;
  C.Cr_rudder  = C.Gamma4*C.Cl_rudder  + C.Gamma7*C.Cn_rudder;
  
%% Environmental properties

C.g = 9.806650; % gravity (m/s^2)

C.rho  = 1.268200;  % air density