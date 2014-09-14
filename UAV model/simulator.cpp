
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <stdlib.h>
#include<iostream>
#include<stdio.h>
#include<vector>

#include "setUavParameters.hpp"
#include "setAutopilotGains.cpp"
#include "initUavStates.hpp"
#define pi 3.142857
#include "uavDynamics.hpp"

#define dt 0.01
#define Tsim 700
using namespace std;

//Path Function defined in openGlStuff.hpp

struct pathSim
{
    double x;
    double y;
    double z = 0;
    double headAngle;
};

struct pathSim tempVal;
vector<struct pathSim> UavPath;


vector<pathSim> simulation(double pointsArr[][2], double heightArr[], float num, int &record)
{
    //double arrPoints[5][2] = {{0,0},{3000,3000},{3000,-3000},{-3000,-3000},{-3000,3000}};
    double W1[2];
    W1[0] = pointsArr[1][0];
    W1[1] = pointsArr[1][1];
    double W0[2];
    W0[0] = pointsArr[0][0];
    W0[1] = pointsArr[0][1];

    cout<<"\nW0 = "<<W0[0]<<" and "<<W0[1];
    cout<<"\nW1 = "<<W1[0]<<" and "<<W1[1];

    UavStates* uav = new UavStates;
    param* C = new param;
    autoPilot* pid = new autoPilot;
    float theta;
    float theta_c;

    uav = initUavStates(W1, W0);
    //uav->x0[8] = headingAngle;

    double Rmin = 1;

    Rmin = 1 * (uav->V0/(15*pi/180));
    cout<<"\nRmin : "<<Rmin;

    double *dx;

    double *k1, *k2, *k3, *k4;

    double temp[16];

    double t =0;
    int counter =0;

    tempVal.x = W0[0];
    tempVal.y = W0[1];
    tempVal.headAngle = uav->x0[8];

    UavPath.push_back(tempVal);
    counter++;
    float delta = 0;

    //while(!(abs(abs(UavPath[counter-1].x) - abs(W1[0]))<100  and abs(abs(UavPath[counter-1].y) - abs(W1[1]))<100) and t<Tsim)
    //while(t<Tsimt<Tsim
for(int a =1;a<num-1;a = a+1)
{
    delta = 0;
    while(!((UavPath[counter-1].x < (W1[0]+5)) and (UavPath[counter-1].x > (W1[0]-5)) and (UavPath[counter-1].y < (W1[1]+5)) and (UavPath[counter-1].y > (W1[1]-5))) and t<Tsim)
    {
        //K1 calculated
        k1 = uavDynamics(C, t, uav->x0, pid, uav, W0, W1, Rmin);
        for(int j =0;j<16;j++)
            {
                //k1[j] = dx[j];
                temp[j] = uav->x0[j] + k1[j]*dt/2;
            }

        //K2 Calculated
        k2 = uavDynamics(C, t, temp, pid, uav, W0, W1, Rmin);
        for(int j =0;j<16;j++)
            {
                //k2[j] = dx[j];
                temp[j] = uav->x0[j] + k2[j]*dt/2;
            }

        k3 = uavDynamics(C, t, temp, pid, uav, W0, W1, Rmin);
        for(int j =0;j<16;j++)
            {
                //k2[j] = dx[j];
                temp[j] = uav->x0[j] + k3[j]*dt;
            }

        k4 = uavDynamics(C, t, temp, pid, uav, W0, W1, Rmin);

        for(int j =0;j<16;j++)
            {
                uav->x0[j] = uav->x0[j] + dt*(k1[j]/6 +k2[j]/3 + k3[j]/3 + k4[j]/6);
            }
        t = t+dt;

        tempVal.x = (int)uav->x0[0];
        tempVal.y = (int)uav->x0[1];


        //if(delta > 0.5)
            //delta = delta + 0.001;
        //else
        //    delta = delta + 0.1;


        tempVal.z = heightArr[a-1] + delta* (heightArr[a] - heightArr[a-1]);

        tempVal.headAngle = uav->x0[8];

        UavPath.push_back(tempVal);

        W0[0] = uav->x0[0];//tempVal.x;
        W0[1] =  uav->x0[1];//tempVal.y;

        //cout<<"\nX cordinate : "<<UavPath[counter].x<<"\tY cordinate : "<<UavPath[counter].y;
        counter++;


    }
    t =0 ;

    W0[0] = pointsArr[a][0];
    W0[1] = pointsArr[a][1];
    W1[0] = pointsArr[a+1][0];
    W1[1] = pointsArr[a+1][1];
}
    //cout<<"\nX cordinate : "<<UavPath[0].x<<"\tY cordinate : "<<UavPath[0].y;
    //cout<<"\nX cordinate : "<<UavPath[counter-1].x<<"\tY cordinate : "<<UavPath[counter-1].y;
    //cout<<"\nCounter : "<<counter;

    record = counter;


    //getValues(UavPath, counter);


    return UavPath;
}



