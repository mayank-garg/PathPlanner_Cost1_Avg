#include "setAutopilotGains.cpp"
#include "initUavStates.hpp"
#define pi 3.142857
#include "uavAutopilot.hpp"

double* uavAutopilot(UavStates* uav,autoPilot* pid, double V,double h,double theta,double q,double phi,double p,double xiDV,double xiDh,double xiDtheta,double xiDphi,double phid)
{
//// The Autopilot
// issues control commands

double deg2rad = pi/180;
double rad2deg = 180/pi;

// velocity hold
double dV = uav->V0-V;
double delta_t = saturate(dV*pid->kVP+xiDV*pid->kVI+1.0075,0.8,2);
// delta_t = saturate(dV*pid->kVP+xiDV*pid->kVI,0.8,2);

// altitude hold
double dh = uav->h0-h;
double theta_error = (dh*pid->khP+xiDh*pid->khI-(theta-uav->theta0)*rad2deg)*pid->kthetaP+xiDtheta*pid->kthetaI;
double q_error = theta_error-q*rad2deg*pid->kqP;
double delta_e = saturate(q_error+0.0018*rad2deg,-40,40)*deg2rad;

// attitude hold
double dphi = (phid - phi)*rad2deg;
double p_error = dphi*pid->kphiP+xiDphi*pid->kphiI-p*rad2deg*pid->kpP;
double delta_a = saturate(p_error,-40,40)*deg2rad;

double delta_r = 0;

double arr[4] = {delta_t,delta_e,delta_a,delta_r};

return arr;//{delta_t,delta_e,delta_a,delta_r};

}

double saturate(double unsatVal,double minVal,double maxVal)
{
double satVal;
if (unsatVal < minVal)
    satVal = minVal;
else if (unsatVal > maxVal)
    satVal = maxVal;
else
    satVal = unsatVal;
return satVal;

}
