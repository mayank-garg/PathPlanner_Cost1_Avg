#include<math.h>
#include "lqruavHeadingRate.hpp"
#include<cmath>
#include<iostream>
#define pi 3.142857

double lqruavHeadingRate(UavStates* uav,double wn,double we, double* W0, double* W1, double Rmin)
{
    // function to find heading rate for a guidance law taking the wind effects
// also into account
// assign states to simple variables for ease of coding

double* x = uav->x0;
double px   = *(x+0);
double py   = *(x+1);
double v   = norm(*(x+3),*(x+4),*(x+5));
//std::cout<<"\nNorm = " <<v;
double psi = *(x+8);

double q22 = 1;

double uwd[2] = {v*cos(psi)+wn , v*sin(psi)+we};

double kiang = atan2(uwd[1], uwd[0]);
double vg = sqrt(pow(uwd[0],2)+pow(uwd[1],2));
double z[2] = {px, py};

// lqr guidance
double theta = atan2(W1[1]-W0[1], W1[0]-W0[0]);
double R = sqrt(pow((z[0]-W0[0]),2)+pow((z[1]-W0[1]),2));
double theta_M = atan2(z[1]-W0[1],z[0]-W0[0]);
double d = R*sin(theta-theta_M);

// ddot
double vd ;
 vd = vg*sin(ang_wrap(kiang-theta));// + vwind*sin(awind-theta);

double q11 ;
q11   =  std::abs(2/(2-std::abs(d)));
//std::cout<<"\nd = " <<d<<"\tq11 = " <<q11;

d = -d;

//u =  (exp(k*d/2) + sqrt(2*exp(k*d/2 + 1)*vd));
double u = -(sqrt(q11)*d+sqrt(2*sqrt(q11)+q22)*vd);

// limit the accelaration to the minimum turning radius
if (std::abs(u) > pow(vg,2)/Rmin)
{
    if (u > 0)
        u = pow(vg,2)/Rmin;
    else
        u = - pow(vg,2)/Rmin;
}

double psidot = u/vg;// update the heading direction

return psidot;
}

double norm(double x, double y, double z)
{
    return sqrt(pow(std::abs(x),2) + pow(std::abs(y),2) + pow(std::abs(z),2));

}

double ang_wrap(double ang)
{
    while (ang <= pi)
        ang = ang + 2*pi;
    while (ang > pi)
        ang = ang - 2*pi;

    return ang;
}
