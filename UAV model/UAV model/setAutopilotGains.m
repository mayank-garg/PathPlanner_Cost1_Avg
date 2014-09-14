%% Set Autopilot Gains

global pid

pid.kVP = 1/20;
pid.kVI = 0.1;

pid.khP = 0.7;
pid.khI = 0.03;

pid.kthetaP = 0.8;
pid.kthetaI = 0.03;

pid.kqP = 0.8;

pid.kpsiP = 0.4;
pid.kpsiI = 0.3;

pid.kphiP = 5;
pid.kphiI = .0;

pid.kpP = 0.75;