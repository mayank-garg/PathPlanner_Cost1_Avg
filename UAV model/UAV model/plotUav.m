function plotUav(t,x)
% function to plot and thus visualize UAV attitude and height
    
global fig_attitude fig_height

if nargin == 0
    
    phi = 0*pi/180;
    theta = 0*pi/180;
    h = 50;
    % initialize the plot    
     [fig_attitude,fig_height] = initializePlot(h,theta,phi);  
     
else    

% assign variables    
    phi = x(7);
    theta = x(8);
    h = x(3);


    % plot UAV Attitude
     uavAttitudePlot(fig_attitude, phi, theta, 0);
    % plot UAV height
     uavHeightPlot(fig_height, h, phi);
     
%     xlabel(['t = ' num2str(t)])
      
end
  
%% User defined functions
%----------------------------------------------------------------------
%----------------------------------------------------------------------
% 
%----------------------------------------------------------------------
%----------------------------------------------------------------------

%% Funtion to initialize plot
function [fig_attitude,fig_height] = initializePlot(h,theta,phi)

    % initialize the figure
    figure(1), clf
    
    % initialize plot of UAV height
    subplot('position',[0.75, 0.1, 0.15, 0.8])
    fig_height = uavHeightPlotInit(h, theta, 'normal');
    axis([-.15/.8*100/2, .15/.8*100/2, 0, 100]) % units are (centimeters x meter)
    box on
    xlabel('UAV wing (unitless)')
    ylabel('Height (meters)')
    title('Altitude and Roll')

    
    % initialize plot of UAV attitude
    subplot('position',[0.1, 0.1, 0.5, 0.8]);
    axis([-100, 100, -100, 100, -100, 100]) % units are centimeters
    axis off
    fig_attitude = uavAttitudePlotInit(phi, theta, 10*pi/180, 'normal');
    title('Attitude (roll, pitch, yaw)')
    view(0,0)
    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Attitude plotting functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle=uavAttitudePlotInit(phi, theta, psi, mode)
% uavPlotInit:  plot UAV attitude at Euler angles phi, theta, psi

  [V,F,patchcolors] = uavAttitudeData;
  V = threeDRotate(V, phi, theta, psi);
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function uavAttitudePlot(handle, phi, theta, psi)
  % uavPlot:  plot UAV attitude at Euler angles phi, theta, psi

  [V,F,patchcolors] = uavAttitudeData;
  V = threeDRotate(V, phi, theta, psi);

  set(handle,'Vertices',V,'Faces',F);
  drawnow

%%%%%%%%%%%%%%%%%%%%%%%
function Vout=threeDRotate(Vin,phi,theta,psi)
  % define rotation matrix
%   phi = phi+pi;
% psi = psi+pi/2;
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
%  R = R_yaw'*R_pitch'*R_roll';
%  R = R_roll'*R_pitch'*R_yaw';

  % rotate vertices
  Vout = (R*Vin')';

%%%%%%%%%%%%%%%%%%%%%%%
function [V,F,patchcolors] = uavAttitudeData
  % uavData:  define patch faces of uav

  % the origin is at the center of the fuselage
  h_fuse = 6.5;           % fuselage height (cm)
  w_fuse = 9.5;           % fuselage width (cm)
  l_fuse = 55.5;          % fuselage length (cm)

  % vertices of the fuselage
  Vert_fuse = [...
        l_fuse/2, w_fuse/2, h_fuse/2;...
        l_fuse/2, w_fuse/2, -h_fuse/2;...
        l_fuse/2, -w_fuse/2, -h_fuse/2;...
        l_fuse/2, -w_fuse/2, h_fuse/2;...
        -l_fuse/2, w_fuse/2, -h_fuse/2;...
        -l_fuse/2, w_fuse/2, h_fuse/2;...
        -l_fuse/2, -w_fuse/2, -h_fuse/2;...
        -l_fuse/2, -w_fuse/2, h_fuse/2];    
  % define faces of fuselage
  Face_fuse = [...
        1, 2, 3, 4;... % front
        5, 6, 8, 7;... % back
        2, 3, 7, 5;... % top
        1, 2, 5, 6;... % right 
        3, 4, 8, 7;... % left
        1, 4, 8, 6];   % bottom

  % wing coefficients
  w_recess = 6.5;
  w_height = 5.5;
  w_width = 71.5;
  w_tipwidth = 25;
  w_back = 30;

  Vert_wing_r = [...
        l_fuse/2-w_recess, w_fuse/2, w_height/2;...
        l_fuse/2-w_recess, w_fuse/2, -w_height/2;...
        -l_fuse/2, w_fuse/2, 0;...
        -w_back, w_fuse/2+w_width, w_height/2;...
        -w_back, w_fuse/2+w_width, -w_height/2;...
        -w_back-w_tipwidth, w_fuse/2+w_width, 0;...
    ];

  Face_wing_r = 8 + [...
        1, 2, 5, 4;... % front
        4, 5, 6, 4;... % side
        2, 3, 6, 5;... % top
        1, 3, 6, 4;... % bottom
    ];

  Vert_wing_l = [...
        l_fuse/2-w_recess, -w_fuse/2, w_height/2;...
        l_fuse/2-w_recess, -w_fuse/2, -w_height/2;...
        -l_fuse/2, -w_fuse/2, 0;...
        -w_back, -w_fuse/2-w_width, w_height/2;...
        -w_back, -w_fuse/2-w_width, -w_height/2;...
        -w_back-w_tipwidth, -w_fuse/2-w_width, 0;...
    ];

  Face_wing_l = 14 + [...
        1, 2, 5, 4;... % front
        4, 5, 6, 4;... % side
        2, 3, 6, 5;... % top
        1, 3, 6, 4;... % bottom
    ];

  % rudder coefficients
  r_height = 21;
  r_top = 10;
 
  Vert_rud_r = [...
        -w_back, w_fuse/2+w_width, w_height/2;...
        -w_back-w_tipwidth, w_fuse/2+w_width, 0;...
        -w_back-w_tipwidth, w_fuse/2+w_width, -r_height;...
        -w_back-w_tipwidth+r_top, w_fuse/2+w_width, -r_height;...
    ];

  Face_rud_r = 20 + [...
        1, 2, 3, 4;...
    ];

  Vert_rud_l = [...
        -w_back, -w_fuse/2-w_width, w_height/2;...
        -w_back-w_tipwidth, -w_fuse/2-w_width, 0;...
        -w_back-w_tipwidth, -w_fuse/2-w_width, -r_height;...
        -w_back-w_tipwidth+r_top, -w_fuse/2-w_width, -r_height;...
    ];

  Face_rud_l = 24 + [...
        1, 2, 3, 4;...
    ];

  R_roll = [1, 0, 0;
            0, -1, 0;
            0, 0, -1];
      
%   R_yaw = [0, 1, 0;
%           -1, 0, 0;
%            0, 0, 1];


        
  V = [Vert_fuse; Vert_wing_r; Vert_wing_l; Vert_rud_r; Vert_rud_l]*R_roll; 
  F = [Face_fuse; Face_wing_r; Face_wing_l; Face_rud_r; Face_rud_l];

  myred    = [1, 0, 0];
  mygreen  = [0, 1, 0];
  myblue   = [0, 0, 1];
  myyellow = [1, 1, 0];

  patchcolors = [...
    myblue;...   % fuselage front
    myblue;...   % back
    myyellow;... % top
    myblue;...   % right
    myblue;...   % left
    myblue;...   % bottom
    myblue;...   % right wing front
    mygreen;...  % side
    myblue;...   % top
    myred;...    % bottom
    myblue;...   % left wingfront
    mygreen;...  % side
    myblue;...   % top
    myred;...    % bottom
    mygreen;...  % right rudder
    mygreen;...  % left rudder
    ];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Height plotting functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle=uavHeightPlotInit(h, theta, mode)
% uavHeightPlotInit:  plot UAV height h at pitch angle theta

  l = .15/.8*100/4;
  p1 = [-l*sin(theta); h + l*cos(theta)];
  p2 = [+l*sin(theta); h - l*cos(theta)];
  
  handle = plot([p1(1); p2(1)], [p1(2); p2(2)],'EraseMode', mode);

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function uavHeightPlot(handle, h, theta)
  
  l = .15/.8*100/4;
  p1 = [-l*cos(theta); h+l*sin(theta)];
  p2 = [+l*cos(theta); h-l*sin(theta)];
  
  set(handle,'Xdata',[p1(1);p2(1)],'Ydata',[p1(2);p2(2)]);
  drawnow

