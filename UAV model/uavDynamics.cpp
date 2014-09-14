#include "setUavParameters.hpp"
#include "setAutopilotGains.cpp"
#include "initUavStates.hpp"
#include "lqruavHeadingRate.hpp"
#include<math.h>
#include "uavAutopilot.hpp"
#include<stdio.h>
#include <stdlib.h>
#include<iostream>

using namespace std;
double* uavDynamics(param* C, double t,double* x,autoPilot* pid, UavStates* uav, double* W0, double* W1, double Rmin)
{

  double *dx;// = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  double vwind = 0;
  double awind = 0;
    // UAV states
  double pn     = *(x+0);  // inertial north coordinate (m)
  double pe     = *(x+1);  // inertial east coordinate (m)
  double h      = *(x+2);  // altitude in inertial coordinates (m)
  double u      = *(x+3);  // velocity along body x-axis (m/s)
  double v      = *(x+4);  // velocity along body y-axis (m/s)
  double w      = *(x+5);  // velocity along body z-axis (m/s)
  double phi    = *(x+6);  // roll angle
  double theta  = *(x+7);  // pitch angle
  double psi    = *(x+8);  // yaw angle
  double p      = *(x+9); // roll rate
  double q      = *(x+10); // pitch rate
  double r      = *(x+11); // yaw rate
  double xiDV   = *(x+12);   // velocity hold ctrl state
  double xiDh   = *(x+13); // altitude hold ctrl state
  double xiDtheta = *(x+14); // theta hold ctrl state
  double xiDphi = *(x+15);   // attitude hold ctrl state

   // calculate alpha, beta, and velocity
   double alpha = atan2(w,u);
   double beta  = atan2(v,sqrt((pow(u,2) + pow(w,2))));
   double V = sqrt(pow(u,2) + pow(v,2) + pow(w,2));

   uav->V = V;
    // guidance
  double wn      = vwind*cos(awind); // wind along the north coordinate
  double we      = vwind*sin(awind); // wind along the east coordinate
  double psidotd;

  psidotd = lqruavHeadingRate(uav, wn, we, W0,W1, Rmin);

  double phid = 1.9*psidotd;

    // UAV inputs

    double* deltas;
    deltas = uavAutopilot(uav,pid,V,h,theta,q,phi,p,xiDV,xiDh,xiDtheta,xiDphi,phid);
    double delta_t = *(deltas+0);
    double delta_e = *(deltas+1);
    double delta_a = *(deltas+2);
    double delta_r = *(deltas+3);

    //   delta_t

   // calculate logitudinal force derivatives (depends on alpha)
   double CX0        = -C->CD_0 * cos(alpha)        + C->CL_0 * sin(alpha);
   double CXalpha    = -C->CD_alpha * cos(alpha)    + C->CL_alpha * sin(alpha);
   double CXq        = -C->CD_q * cos(alpha)        + C->CL_q * sin(alpha);
   double CXelevator = -C->CD_elevator * cos(alpha) + C->CL_elevator * sin(alpha);

   double CZ0        = -C->CD_0 * sin(alpha)        - C->CL_0 * cos(alpha);
   double CZalpha    = -C->CD_alpha * sin(alpha)    - C->CL_alpha * cos(alpha);
   double CZq        = -C->CD_q * sin(alpha)        - C->CL_q * cos(alpha);
   double CZelevator = -C->CD_elevator * sin(alpha) - C->CL_elevator * cos(alpha);

   // calculate the forces
   double fx = -C->m*C->g*sin(theta)          + 0.5*C->rho*pow(V,2)*C->S * ( CX0     + (CXalpha * alpha)  + ((CXq * C->cbar * q)/V)   + (CXelevator*delta_e) ) + 0.5*C->rho*C->Sprop * (pow((C->kmotor*delta_t),2) - pow(V,2));
   double fy =  C->m*C->g*cos(theta)*sin(phi) + 0.5*C->rho*pow(V,2)*C->S * ( C->CY_0  + (C->CY_beta * beta) + ((C->CY_p * C->b * p)/(2*V)) + ((C->CY_r * C->b * r)/(2*V)) + (C->CY_aileron * delta_a) + (C->CY_rudder * delta_r) );
   double fz =  C->m*C->g*cos(theta)*cos(phi) + 0.5*C->rho*pow(V,2)*C->S * ( CZ0     + (CZalpha * alpha)  + ((CZq * C->cbar * q)/V)   + (CZelevator*delta_e) );

   // calculate moments
   double l = 0.5*C->rho*pow(V,2)*C->S * (C->b/2) * ( C->Cl_0  + (C->Cl_beta * beta)   + (C->Cl_p * C->b * p)/(2*V)  + (C->Cl_r * C->b * r)/(2*V) + (C->Cl_aileron * delta_a) + (C->Cl_rudder * delta_r) );
   double m = 0.5*C->rho*pow(V,2)*C->S * C->cbar  * ( C->Cm_0  + (C->Cm_alpha * alpha) + ((C->Cm_q * C->cbar * q)/V) + (C->Cm_elevator * delta_e) );
   double n = 0.5*C->rho*pow(V,2)*C->S * (C->b/2) * ( C->Cn_0  + (C->Cn_beta * beta)   + (C->Cn_p * C->b * p)/(2*V)  + (C->Cn_r * C->b * r)/(2*V) + (C->Cn_aileron * delta_a) + (C->Cn_rudder * delta_r) );

   // The UAV Dynamics

   // Navigation Equations
   double pndot = cos(theta)*cos(psi) * u + (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)) * v + (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * w + wn;
   double pedot = cos(theta)*sin(psi) * u + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)) * v + (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * w + we;
   double hdot  = sin(theta) * u - sin(phi)*cos(theta) * v - cos(phi)*cos(theta) * w;

   // Force Equations
   double udot = r*v - q*w + fx/C->m;
   double vdot = p*w - r*u + fy/C->m;
   double wdot = q*u - p*v + fz/C->m;

   // Kinematic Equations
   double phidot   = p + q * sin(phi)*tan(theta) + r * cos(phi)*tan(theta);
   double thetadot = q * cos(phi) - r * sin(phi);
   double psidot   = q * sin(phi)/cos(theta) + r * cos(phi)/cos(theta);

   // Moment Equations
   double pdot = C->Gamma1*p*q - C->Gamma2*q*r + C->Gamma3*l + C->Gamma4*n;
   double qdot = C->Gamma5*p*r - C->Jxz/C->Jy*(pow(p,2) - pow(r,2)) + m/C->Jy;
   double rdot = C->Gamma7*p*q - C->Gamma1*q*r + C->Gamma4*l + C->Gamma8*n;

   // controller state
   double xiDVdot = uav->V0-V;
   double xiDhdot = uav->h0-h;
   double xiDthetadot = (uav->h0-h)*pid->khP+xiDh*pid->khI-(theta-uav->theta0)*180/pi;

   double xiDphidot = (phid-phi)*180/pi;
    double xdot[16] = {pndot, pedot, hdot, udot, vdot, wdot, phidot, thetadot, psidot, pdot, qdot, rdot, xiDVdot, xiDhdot, xiDthetadot, xiDphidot};

    dx = xdot;

return dx;
}
