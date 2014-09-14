#ifndef uavautoPilot
#define uavautoPilot

#include "setAutopilotGains.cpp"
#include "initUavStates.hpp"

double* uavAutopilot(UavStates* uav,autoPilot* pid, double V,double h,double theta,double q,double phi,double p,double xiDV,double xiDh,double xiDtheta,double xiDphi,double phid);
double saturate(double unsatVal,double minVal,double maxVal);

#endif // uavautoPilot
