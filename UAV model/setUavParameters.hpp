#ifndef setParameter
#define setParameter

#define pi 3.142857
#include<cmath>

struct param
{
    float m = 1.56; //Mass (kg)
    //Inertia in Kg-m^2
    float Jx = 0.114700;
    float Jy = 0.057600;
    float Jz = 0.171200;
    float Jxz = 0.001500;

    //Airframe physical parameters

    float b    = 1.422400;  // wingspan (m)
    float cbar = 0.330200;  // mean cord
    float S    = 0.258900;  // wing area

    //propeller characteristics


    float Sprop = pi*(pow(0.1,2)); // prop area
    float kmotor = 20;
    float C_throttle = 0.8;   // aerodynamic efficiency of propeller

    //aerodynamic coefficients for linear force model

      float CL_0        = 0.280000;
      float CD_0        = 0.030000;
      float Cm_0        = 0.000000;
      float CL_alpha    = 3.450000;
      float CD_alpha    = 0.300000;
      float Cm_alpha    = -0.380000;
      float CL_q        = 0.000000;
      float CD_q        = 0.000000;
      float Cm_q        = -3.600000;
      float CL_elevator = -0.360000;
      float CD_elevator = 0.000000;
      float Cm_elevator = 0.500000;
      float CY_0        = 0.000000;
      float Cl_0        = 0.000000;
      float Cn_0        = 0.000000;
      float CY_beta     = -0.980000;
      float Cl_beta     = -0.120000;
      float Cn_beta     = 0.250000;
      float CY_p        = 0.000000;
      float Cl_p        = -0.260000;
      float Cn_p        = 0.022000;
      float CY_r        = 0.000000;
      float Cl_r        = 0.140000;
      float Cn_r        = -0.350000;
      float CY_aileron  = 0.000000;
      float Cl_aileron  = 0.080000;
      float Cn_aileron  = 0.060000;
      float CY_rudder   = -0.170000;
      float Cl_rudder   = 0.105000;
      float Cn_rudder   = -0.032000;

    //Coefficients associated with inverse of J

      float Gamma  = Jx*Jz - pow(Jxz,2);
      float Gamma1 = Jxz*(Jx - Jy + Jz)/Gamma;
      float Gamma2 = (Jz*(Jz-Jy) + pow(Jxz,2))/Gamma;
      float Gamma3 = Jz/Gamma;
      float Gamma4 = Jxz/Gamma;
      float Gamma5 = (Jz-Jx)/Jy;
      float Gamma6 = 1/Jy;
      float Gamma7 = ((Jx-Jy)*Jx + pow(Jxz,2))/Gamma;
      float Gamma8 = Jx/Gamma;

    //Converting coefficients so that the roll and yaw channels decouple

      float Cp_0       = Gamma3*Cl_0       + Gamma4*Cn_0;
      float Cp_beta    = Gamma3*Cl_beta    + Gamma4*Cn_beta;
      float Cp_p       = Gamma3*Cl_p       + Gamma4*Cn_p;
      float Cp_r       = Gamma3*Cl_r       + Gamma4*Cn_r;
      float Cp_aileron = Gamma3*Cl_aileron + Gamma4*Cn_aileron;
      float Cp_rudder  = Gamma3*Cl_rudder  + Gamma4*Cn_rudder;

      float Cr_0       = Gamma4*Cl_0       + Gamma7*Cn_0;
      float Cr_beta    = Gamma4*Cl_beta    + Gamma7*Cn_beta;
      float Cr_p       = Gamma4*Cl_p       + Gamma7*Cn_p;
      float Cr_r       = Gamma4*Cl_r       + Gamma7*Cn_r;
      float Cr_aileron = Gamma4*Cl_aileron + Gamma7*Cn_aileron;
      float Cr_rudder  = Gamma4*Cl_rudder  + Gamma7*Cn_rudder;

    //Environmental properties

    double g = 9.806650; // gravity (m/s^2)

    double rho  = 1.268200;  // air density

};

#endif
