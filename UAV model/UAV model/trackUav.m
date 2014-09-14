function trackUav(iUav,x)
% function to plot and thus visualize UAV attitude and height
    
global nUav uav

if nargin == 0
    

    for iUav = 1:nUav
    phi = uav(iUav).x0(7);
    theta = uav(iUav).x0(8);
    psi = uav(iUav).x0(9);

        
        % initialize the plot    
     uav(iUav).fig_position = initializePlot(iUav,theta,phi,psi,uav(iUav).x0(1),uav(iUav).x0(2),uav(iUav).x0(3));
    end
     
else    

% assign variables    
    
    px = x(1);
    py = x(2);
    h = x(3);
    phi = x(7);
    theta = x(8);
    psi = x(9);


    % plot UAV 
     uavPositionPlot(uav(iUav).fig_position, phi, theta, psi, px, py, h);
      
end
  
%% User defined functions
%----------------------------------------------------------------------
%----------------------------------------------------------------------
% 
%----------------------------------------------------------------------
%----------------------------------------------------------------------

%% Funtion to initialize plot
function fig_position = initializePlot(iUav,theta,phi,psi,x,y,h)

global uav

if iUav == 1
    % initialize the figure
    figure(2), clf; hold on
    axis([-1e3, 1e3, -1e3, 1e3, -1e3, 1e3]*.5) % units are centimeters
    box on
    title('3D Potition-Attitude Plot')
    xlabel('North (m)')
    ylabel('West (m)')
    zlabel('Height (m)')

end
    
    figure(2)
    % initialize plot of UAV
%     plot3(uav(iUav).xt,-uav(iUav).yt,50,'r.')
    fig_position = uavPositionPlotInit(phi, theta, psi, 'normal',x,y,h);
    view(0,90)
    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Attitude plotting functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle=uavPositionPlotInit(phi, theta, psi, mode,x,y,h)
% uavPlotInit:  plot UAV attitude at Euler angles phi, theta, psi

  [V,F,patchcolors] = uavAttitudeData;
  V = threeDRotate(V, phi, theta, psi)+ones(size(V,1),1)*[x -y h];
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function uavPositionPlot(handle, phi, theta, psi, x, y, h)
  % uavPlot:  plot UAV attitude at Euler angles phi, theta, psi

  [V,F,patchcolors] = uavAttitudeData;
  V = threeDRotate(V, phi, theta, psi)+ones(size(V,1),1)*[x -y h];
  
  set(handle,'Vertices',V,'Faces',F);
  drawnow

%%%%%%%%%%%%%%%%%%%%%%%
function Vout=threeDRotate(Vin,phi,theta,psi)
  % define rotation matrix
%   phi = phi+pi;
% psi = -psi;
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
% 
      

  V = [Vert_fuse; Vert_wing_r; Vert_wing_l; Vert_rud_r; Vert_rud_l]*R_roll*1/5; 
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
