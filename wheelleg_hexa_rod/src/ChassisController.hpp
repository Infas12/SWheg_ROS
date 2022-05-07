#ifndef CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLER_HPP
#include <math.h>

#define PI acos(-1)
#define POS_THRESHOLD 0.08

#define MOTOR_NUM 6
// motor index: the letter U

#define WHEEL_MODE true
#define LEG_MODE false
//TODO:leg_forward
int leg_forward[6] = {1,1,1,-1,-1,-1};
float walk_phase[6] = {PI/2,0,PI/2,0,PI/2,0};
enum LEG_GAIT {
    NO_LEG_GAIT = 0,
    WALK_GAIT,
    GAIT_NUM
};

enum ROBOT_STATE{
    NO_ROBOT_STATE = 0,
    WHEEL_STATE,
    LEG_STATE
};

#endif
