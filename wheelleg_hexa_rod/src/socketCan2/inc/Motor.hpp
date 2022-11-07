#ifndef MOTOR
#define MOTOR

class Motor
{
public:
    enum MotorControlModeType
    {
        RELAX_MODE,
        SPD_MODE,
        POS_MODE,        
    };

    float currentFdb = 0.0;
    float speedFdb = 0.0;
    float positionFdb = 0.0;
    
    float speedSet = 0.0;
    float positionSet = 0.0;
    float currentSet = 0.0;
    
    float position = 0.0;
    Motor(){};
    virtual void Update() = 0;

};

#endif