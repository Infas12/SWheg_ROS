#include "../inc/M3508.hpp"
#include "../inc/Math.hpp"
#include "../inc/DjiMotorManager.hpp"
#include <iostream>
#include <math.h>
#define PI acos(-1)

// tendon version
const float M3508::RawPos2Rad = 0.000025847164546859f;  /* 2Pi / 8191 / (3591/187) / (34/22)   */
const float M3508::RawRpm2Rps = 0.00352856874672204f;    /* 2Pi / 60 / (3591/187) / (34/22)   */
const float M3508::PiDiv19 = 0.105857062401661f;      /* PI / (3591/187) / (34/22)  */
const float M3508::CurrentRatio = 0.00122070312;      /* 20/16384  */

// // linear actuator
// const float M3508::RawPos2Rad = 0.000039945617936054835f;  /* 2Pi / 8191 / (3591/187) / (22/22)   */
// const float M3508::RawRpm2Rps = 0.005453242608570419f;    /* 2Pi / 60 / (3591/187) / (22/22)   */
// const float M3508::PiDiv19 = 0.1635972782571126f;      /* PI / (3591/187) / (22/22)  */
// const float M3508::CurrentRatio = 0.00122070312;      /* 20/16384  */

void M3508::HandleNewMsg(can_frame msg)
{
    ecd = (uint16_t)(msg.data[0] << 8 | msg.data[1]);           
    speed_rpm = (int16_t)(msg.data[2] << 8 | msg.data[3]);     
    given_current = (int16_t)(msg.data[4] << 8 | msg.data[5]); 
    temperate = msg.data[6];  

    rotorLastPosition = rotorPosition;
    rotorPosition = ecd * RawPos2Rad - PiDiv19;
    positionFdb += Math::LoopFloatConstrain((rotorPosition - rotorLastPosition), - PiDiv19, PiDiv19);
    positionFdb = Math::LoopFloatConstrain(positionFdb, - PI, PI); 
    position = positionFdb;

    speedFdb = speed_rpm * RawRpm2Rps;
    
    currentFdb = given_current * CurrentRatio;
    // std::cout << speedFdb << std::endl;
    // std::cout << positionFdb << std::endl;

}

void M3508::Update()
{
    if (controlMode == POS_MODE)
    {
        positionSet = Math::LoopFloatConstrain(positionSet, - PI, PI); 
        pidPosition.ref = Math::LoopFloatConstrain((positionSet-positionFdb), - PI, PI); 
        pidPosition.fdb = 0;  
        // pidPosition.ref = positionSet;
        // pidPosition.fdb = positionFdb;

        pidPosition.UpdateResult();
        
        speedSet = pidPosition.result;
        
    }
    
    pidSpeed.ref = speedSet;
    pidSpeed.fdb = speedFdb;
    pidSpeed.UpdateResult();

    //calculate current
    currentSet = pidSpeed.result;
    currentSet = Math::FloatConstrain(currentSet,-10.0,10.0);
    if(controlMode == RELAX_MODE)
    {
        currentSet = 0.0f;
    }

    //mapping current back to -1000 to 1000     -16384~0~16384 to -20~0~20A
    int16_t currentSend = currentSet * 1000;
    DjiMotorManager::Instance() -> Add2Buffer(canId,currentSend);
}


