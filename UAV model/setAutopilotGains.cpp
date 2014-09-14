#ifndef auto
#define auto

struct autoPilot
{
    float kVP = 1/20;
    float kVI = 0.1;

    float khP = 0.7;
    float khI = 0.03;

    float kthetaP = 0.8;
    float kthetaI = 0.03;

    float kqP = 0.8;

    float kpsiP = 0.4;
    float kpsiI = 0.3;

    float kphiP = 5;
    float kphiI = .0;

    float kpP = 0.75;
};

#endif // autoPilot
