#ifndef heading
#define heading

#include "initUavStates.hpp"
//#include "main.hpp"
double lqruavHeadingRate(UavStates* uav,double wn,double we,  double*, double*, double);
double norm(double x, double y, double z);
double ang_wrap(double ang);
#endif
