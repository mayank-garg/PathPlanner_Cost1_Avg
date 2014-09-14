#ifndef uavStates
#define uavStates

#include<vector>

struct UavStates
{
    double xt;
    double yt;
    double V0;
    double h0;
    double theta0;
    double V;
    double x0[16];
    double fig_position;

};

UavStates* initUavStates(double W1[2], double W0[2]);
/*
void trackUav(UavStates* uav);
double initializePlot(double theta,double phi,double psi,double x,double y,double h);
double uavPositionPlotInit(double phi,double theta,double psi,double mode,double x,double y,double h);
std::vector< std::vector < double > > uavAttitudeData();
*/

#endif // uavStates
