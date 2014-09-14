function [delta_t,delta_e,delta_a,delta_r]=uavAutopilot(V,h,theta,q,phi,p,xiDV,xiDh,xiDtheta,xiDphi,phid)
%% The Autopilot
% issues control commands

global uav pid iUav

deg2rad = pi/180;
rad2deg = 180/pi;

% velocity hold
dV = uav(iUav).V0-V;
delta_t = saturate(dV*pid.kVP+xiDV*pid.kVI+1.0075,0.8,2);
% delta_t = saturate(dV*pid.kVP+xiDV*pid.kVI,0.8,2);

% altitude hold
dh = uav(iUav).h0-h;
theta_error = (dh*pid.khP+xiDh*pid.khI-(theta-uav(iUav).theta0)*rad2deg)*pid.kthetaP+xiDtheta*pid.kthetaI;
q_error = theta_error-q*rad2deg*pid.kqP;
delta_e = saturate(q_error+0.0018*rad2deg,-40,40)*deg2rad;
% dh = uav.h0-h;
% theta_error = (dh*pid.khP-(theta)*rad2deg)*pid.kthetaP+xiDtheta*pid.kthetaI;
% q_error = theta_error-q*rad2deg*pid.kqP;
% delta_e = saturate(q_error,-40,40)*deg2rad;

% attitude hold
dphi = (phid - phi)*rad2deg;
p_error = dphi*pid.kphiP+xiDphi*pid.kphiI-p*rad2deg*pid.kpP;
delta_a = saturate(p_error,-40,40)*deg2rad;

delta_r = 0;

function satVal = saturate(unsatVal,minVal,maxVal)

if unsatVal < minVal
    satVal = minVal;
elseif unsatVal > maxVal
    satVal = maxVal;
else
    satVal = unsatVal;
end