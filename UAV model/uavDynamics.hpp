#ifndef Dynamcis
#define Dynamcis

#include "setUavParameters.hpp"
#include "setAutopilotGains.cpp"
#include "initUavStates.hpp"

double* uavDynamics(param*, double,double*,autoPilot*, UavStates*, double*, double*, double );

#endif
