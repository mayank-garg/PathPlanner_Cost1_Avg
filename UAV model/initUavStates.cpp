#include<cmath>
#define pi 3.142857
#include <stdlib.h>
#include<vector>
#include "initUavStates.hpp"


UavStates* initUavStates(double W1[2], double W0[2])
{
    //Initialize the states of UAV.
    UavStates* uav_ = new UavStates;

    //initial north and east of UAV
    double pn0 = W0[0]; //500*cos(rand*2*pi);
    double pe0 = W0[1]; //500*sin(rand*2*pi);

    //destination x and y
       uav_->xt = W1[0];//800*cos(rand()*2*pi);
       uav_->yt = W1[1];//800*sin(rand()*2*pi);

    //initial heading
    double psi0 = -pi; //2*pi*rand;


    // other initial states
    double V0 = 10;
    double h0 = 50;
    double alpha0 = 0.002375;
    double beta0 = 0;
    double gamma0 = 0;
    double phi0 = 0;
    double phidot0 = 0;
    double thetadot0 = 0;
    double psidot0 = 0;

    double theta0 = alpha0 + gamma0;

    //assign the above values to UAV state structure

    uav_->V0 = V0;  // initial airspeed
    uav_->h0 = h0;  // altitude to hold
    uav_->theta0 = theta0; // initial pitch

    uav_->V = V0;

    //rotation rates

    double p0 = (1*phidot0) + (0*thetadot0) + (psidot0*(-sin(theta0)));
    double q0 = (0*phidot0) + (cos(phi0)*thetadot0) + (sin(phi0)*cos(theta0*(-sin(theta0))));
    double r0 = (0*phidot0) + (-sin(phi0)*thetadot0) + (cos(phi0)*cos(theta0)*(-sin(theta0)));

    uav_->x0[0]= pn0;   //pn:     x-position (m)
    uav_->x0[1]= pe0;   //pe:     y-position (m)
    uav_->x0[2]= h0;    //h:     altitude (m)
    uav_->x0[3]= uav_->V0*cos(alpha0)*cos(beta0);  //u:     velocity along body x-axis (m/s)
    uav_->x0[4]= uav_->V0*sin(beta0);    // v:     velocity along body y-axis (m/s);
    uav_->x0[5]= uav_->V0*sin(alpha0)*cos(beta0); // w:     velocity along body z-axis (m/s);
    uav_->x0[6]= phi0;   // phi:   roll angle;
    uav_->x0[7]= theta0; //theta: pitch angle
    uav_->x0[8]= psi0;   // psi:   yaw angle
    uav_->x0[9]= p0;     // p:     roll rate
    uav_->x0[10]= q0;     //q:     pitch rate
    uav_->x0[11]= r0;     // r:     yaw rate
    uav_->x0[12]= 0;      // velocity hold controller state
    uav_->x0[13]= 0;      // altitude hold controller state
    uav_->x0[14]= 0;       // theta hold controller state
    uav_->x0[15]= 0;       // attitude hold controller state

    return uav_;

}


/*
void trackUav(struct UavStates* uav)
{
  //function to plot and thus visualize UAV attitude and height

    double phi = uav->x0[7];
    double theta = uav->x0[8];
    double psi = uav->x0[9];

    //initialize the plot
     uav->fig_position = initializePlot(theta,phi,psi,uav->x0[1],uav->x0[2],uav->x0[3]);

}

double initializePlot(double theta,double phi,double psi,double x,double y,double h)
{
    //initialize the figure
    figure(2), clf; hold on
    axis([-1e3, 1e3, -1e3, 1e3, -1e3, 1e3]*.5) // units are centimeters
    box on
    title('3D Potition-Attitude Plot')
    xlabel('North (m)')
    ylabel('West (m)')
    zlabel('Height (m)')

end

    figure(2)
    //initialize plot of UAV

    double fig_position = uavPositionPlotInit(phi,theta, psi, 0, x, y, h);


}

double uavPositionPlotInit(double phi,double theta,double psi,double mode,double x,double y,double h)
{
    // uavPlotInit:  plot UAV attitude at Euler angles phi, theta, psi
    std::vector< std::vector < double > > val;//[3][6];
    val = uavAttitudeData();

  [V,F,patchcolors] = uavAttitudeData();
  V = threeDRotate(V, phi, theta, psi)+ones(size(V,1),1)*[x -y h];
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);



}

std::vector< std::vector < double > > uavAttitudeData()
 {

  //uavData:  define patch faces of uav

  //the origin is at the center of the fuselage
  double h_fuse = 6.5;           // fuselage height (cm)
  double w_fuse = 9.5;           // fuselage width (cm)
  double l_fuse = 55.5;          // fuselage length (cm)

  //vertices of the fuselage
  double Vert_fuse[8][3] =
  {
    {l_fuse/2, w_fuse/2, h_fuse/2},
    {l_fuse/2, w_fuse/2, -h_fuse/2},
    {l_fuse/2, -w_fuse/2, -h_fuse/2},
    {l_fuse/2, -w_fuse/2, h_fuse/2},
    {-l_fuse/2, w_fuse/2, -h_fuse/2},
    {-l_fuse/2, w_fuse/2, h_fuse/2},
    {-l_fuse/2, -w_fuse/2, -h_fuse/2},
    {-l_fuse/2, -w_fuse/2, h_fuse/2}
  };

  //define faces of fuselage
  double Face_fuse[6][4] =
  {
    {1, 2, 3, 4},   // front
    {5, 6, 8, 7},   // back
    {2, 3, 7, 5},   // top
    {1, 2, 5, 6},   //right
    {3, 4, 8, 7},    //left
    {1, 4, 8, 6},  // bottom
  };


  // wing coefficients
  double w_recess = 6.5;
  double w_height = 5.5;
  double w_width = 71.5;
  double w_tipwidth = 25;
  double w_back = 30;

  double Vert_wing_r[6][4] =
  {
    {l_fuse/2-w_recess, w_fuse/2, w_height/2},
    {l_fuse/2-w_recess, w_fuse/2, -w_height/2},
    {-l_fuse/2, w_fuse/2, 0},
    {-w_back, w_fuse/2+w_width, w_height/2},
    {-w_back, w_fuse/2+w_width, -w_height/2},
    {-w_back-w_tipwidth, w_fuse/2+w_width, 0}
  }

  double Face_wing_r[4][4] =
 {
    {1, 2, 5, 4}, // front
    {4, 5, 6, 4}, // side
    {2, 3, 6, 5}, // top
    {1, 3, 6, 4} // bottom
 }
 for(int i =0;i<4;i++)
 {
     for(int j =0;j<4;j++)
        Face_wing_r[i][j] = Face_wing_r[i][j] + 8;
 }

  double Vert_wing_l[6][3] =
  {
    {l_fuse/2-w_recess, -w_fuse/2, w_height/2},
    {l_fuse/2-w_recess, -w_fuse/2, -w_height/2},
    {-l_fuse/2, -w_fuse/2, 0},
    {-w_back, -w_fuse/2-w_width, w_height/2},
    {-w_back, -w_fuse/2-w_width, -w_height/2},
    {-w_back-w_tipwidth, -w_fuse/2-w_width, 0}
  }
  double Face_wing_l[4][4] =
  {
    {1, 2, 5, 4}, // front
    {4, 5, 6, 4}, // side
    {2, 3, 6, 5}, // top
    {1, 3, 6, 4} // bottom
  }

for(int i =0;i<4;i++)
 {
     for(int j =0;j<4;j++)
        Face_wing_l[i][j] = Face_wing_l[i][j] + 14;
 }

  // rudder coefficients
  double r_height = 21;
  double r_top = 10;

  double Vert_rud_r[4][3] =
  {
    {-w_back, w_fuse/2+w_width, w_height/2},
    {-w_back-w_tipwidth, w_fuse/2+w_width, 0},
    {-w_back-w_tipwidth, w_fuse/2+w_width, -r_height},
    {-w_back-w_tipwidth+r_top, w_fuse/2+w_width, -r_height}
  }

  double Face_rud_r =  {1, 2, 3, 4}

for(int i =0;i<4;i++)
        Face_rud_r[i] = Face_rud_r[i] + 20;

  double Vert_rud_l[4][3] =
  {
    {-w_back, -w_fuse/2-w_width, w_height/2},
    {-w_back-w_tipwidth, -w_fuse/2-w_width, 0},
    {-w_back-w_tipwidth, -w_fuse/2-w_width, -r_height},
    {-w_back-w_tipwidth+r_top, -w_fuse/2-w_width, -r_height}
  }

  double Face_rud_l[4] = {1, 2, 3, 4};

 for(int i =0;i<4;i++)
        Face_rud_l[i] = Face_rud_l[i] + 24;

  double R_roll[3][3] =
  {
    {1, 0, 0},
    {0, -1, 0},
    {0, 0, -1}
  }

  V = [Vert_fuse; Vert_wing_r; Vert_wing_l; Vert_rud_r; Vert_rud_l]*R_roll*1/5;
  F = [Face_fuse; Face_wing_r; Face_wing_l; Face_rud_r; Face_rud_l];

  myred    = [1, 0, 0];
  mygreen  = [0, 1, 0];
  myblue   = [0, 0, 1];
  myyellow = [1, 1, 0];

  patchcolors = [...
    myblue;...   // fuselage front
    myblue;...   // back
    myyellow;... // top
    myblue;...   // right
    myblue;...   // left
    myblue;...   // bottom
    myblue;...   // right wing front
    mygreen;...  // side
    myblue;...   // top
    myred;...    // bottom
    myblue;...   // left wingfront
    mygreen;...  // side
    myblue;...   // top
    myred;...    // bottom
    mygreen;...  // right rudder
    mygreen;...  // left rudder
    ];

 }

threeDRotate(Vin,phi,theta,psi)
{
   // define rotation matrix
//   phi = phi+pi;
// psi = -psi;
phi = -phi;

  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;
//  R = R_yaw'*R_pitch'*R_roll';
//  R = R_roll'*R_pitch'*R_yaw';

  // rotate vertices
  Vout = (R*Vin')';

}

*/



