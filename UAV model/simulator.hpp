#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <stdlib.h>
#include<iostream>
#include<stdio.h>

#include "setUavParameters.hpp"
#include "setAutopilotGains.cpp"
#include "initUavStates.hpp"
#define pi 3.142857
#include "uavDynamics.hpp"

#define dt 0.001
#define Tsim 500
using namespace std;

struct pathSim
{
    double x;
    double y;
    double z = 0;
    double headAngle;

};

vector<pathSim> simulation(double pointsArr[][2], double heightArr[], float num, int &record);

