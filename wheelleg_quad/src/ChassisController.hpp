#ifndef CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLER_HPP
#include <math.h>

#define PI acos(-1)
#define POS_THRESHOLD 0.08

#define LF 3
#define RF 4
#define RH 2
#define LH 1
#define RF_FORWARD -1
#define RH_FORWARD -1
#define LH_FORWARD 1
#define LF_FORWARD 1
int leg_forward[4] = {1,-1,1,-1};

// LEG MODE
//repect to LF
const float WALK_LH = PI*3/4;
const float WALK_LF = 0; 
const float WALK_RH = PI/4;
const float WALK_RF = PI/2;
float walk_phase[4] = {WALK_LH,WALK_RH,WALK_LF,WALK_RF};
// float climb_phase[4] = {PI/2,PI/2,0,0};
float climb_phase[4] = {0,0,0,0};
float trot_phase[4] = {PI/2,0,0,PI/2};

enum LEG_MODE {
    NO_LEG_MODE = 0,
    WALK_MODE,
    TROT_MODE,
    CLIMB_MODE,
    MODE_NUM
};

enum ROBOT_STATE{
    NO_ROBOT_STATE = 0,
    WHEEL_STATE,
    LEG_STATE
};

#endif
