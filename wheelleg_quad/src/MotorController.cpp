#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "socketCan2/inc/CanManager.hpp"
#include "socketCan2/inc/DjiMotorManager.hpp"
#include "socketCan2/inc/M2006.hpp"
#include "socketCan2/inc/M3508.hpp"
#include "wheelleg_quad/JointData.h"
#include "wheelleg_quad/MotorData.h"
#include <fstream>

M3508 ChassisMotor[4];
wheelleg_quad::MotorData msgMotor; 
#define WHEEL_MODE true
#define LEG_MODE false

//定义回调函数
void controlCallback(const wheelleg_quad::JointData& msg){
    for(int i = 0; i < 4; i++){
        if(msg.JointMode[i] == WHEEL_MODE){
            ChassisMotor[i].controlMode = Motor::SPD_MODE; 
            ChassisMotor[i].speedSet = msg.JointData[i]; 
        }else{
            ChassisMotor[i].controlMode = Motor::POS_MODE; 
            ChassisMotor[i].positionSet = msg.JointData[i]; 
        }
    }
    // ROS_WARN("MODE: %d   DATA: %f ", msg.JointData[0], msg.JointData[0]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motorController");    
    ros::NodeHandle n;  
    ros::Subscriber control_sub = n.subscribe("control", 10, controlCallback);
    ros::Publisher motor_data_pub = n.advertise<wheelleg_quad::MotorData>("motor", 10);
    ros::Rate loop_rate(100);

    for(int i = 0; i < 4; i++)
    {
        ChassisMotor[i].Registration(0x201+i);
        ChassisMotor[i].pidSpeed.kp = 2; 
	    ChassisMotor[i].pidSpeed.ki = 0.01;
	    ChassisMotor[i].pidSpeed.kd = 0.001;//0.01
	    ChassisMotor[i].pidSpeed.maxOut = 10;
	    ChassisMotor[i].pidSpeed.maxIOut = 10;
        ChassisMotor[i].pidPosition.kp = 5;
	    ChassisMotor[i].pidPosition.ki = 0.0;
	    ChassisMotor[i].pidPosition.kd = 0.001;//0.01
	    ChassisMotor[i].pidPosition.maxOut = 10;
	    ChassisMotor[i].pidPosition.maxIOut = 10;
    }

    CanManager::Instance()->SetPortName("can0");
    CanManager::Instance()->Init();

    while (ros::ok())
    {   
        for(int i = 0; i < 4; i++){
            ChassisMotor[i].Update(); //Update Motor Control
        }
        for(int i = 0; i < 4; i++){
            msgMotor.current[i] = ChassisMotor[i].currentFdb;
            msgMotor.position[i] = ChassisMotor[i].positionFdb;
            msgMotor.speed[i] = ChassisMotor[i].speedFdb;
        }
        motor_data_pub.publish(msgMotor);

        DjiMotorManager::Instance()->Update();//Send Commands
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}