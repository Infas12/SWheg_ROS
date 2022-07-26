#include "ros/ros.h"
#include <std_msgs/String.h>
#include <iostream>
#include "socketCan2/inc/CanManager.hpp"
#include "socketCan2/inc/DjiMotorManager.hpp"
#include "socketCan2/inc/M2006.hpp"
#include "socketCan2/inc/M3508.hpp"
#include <wheelleg_control/WheelLegControlMsg.h>
#include <sensor_msgs/JointState.h>
#include <fstream>

#define MOTOR_NUM 6
M3508 ChassisMotor[MOTOR_NUM];
std::string chassisMotorName[MOTOR_NUM] = {"LF_Joint","LM_Joint","LB_Joint","RB_Joint","RM_Joint","RF_Joint"};
std::map<std::string, M3508*> chassisMotorNameMap;

void controlCallback(const wheelleg_control::WheelLegControlMsg& msg){

    for(int i = 0; i < msg.JointName.size(); ++i)
    {
        std::string name = msg.JointName[i];
        
        auto iter = chassisMotorNameMap.find(name);
        if(iter == chassisMotorNameMap.end()) //Handler not found
        {
            std::cerr << "ERROR: Motor for joint " << name  << "is not registered." << std::endl;
            exit(1);
        }
        
        M3508* motor = (M3508*) iter->second;

        // std::cout   << name      << std::endl 
        //             << "mode:"   << (int16_t)msg.JointMode[i] << std::endl 
        //             << "command:"<< msg.JointData[i] << std::endl;


        if(msg.JointMode[i]==0) //speed mode
        {
            motor->controlMode = Motor::SPD_MODE; 
            motor->speedSet = msg.JointData[i]; 
        }
        else if (msg.JointMode[i]==1)//pos mode
        {
            motor->controlMode = Motor::POS_MODE; 
            motor->positionSet = msg.JointData[i];             
        }
        else
        {
            std::cout << "ERROR: Joint Mode" << msg.JointMode[i] << "is not legal." << std::endl;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motorController");    
    ros::NodeHandle n;  
    ros::Subscriber control_sub = n.subscribe("/WheelLegHexapod/command", 10, controlCallback);
    ros::Publisher motor_data_pub = n.advertise<sensor_msgs::JointState>("/WheelLegHexapod/joint_states", 10);
    ros::Rate loop_rate(100);

    for(int i = 0; i < MOTOR_NUM; i++)
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
        chassisMotorNameMap[chassisMotorName[i]] = &ChassisMotor[i];
    }

    CanManager::Instance()->SetPortName("can0");
    CanManager::Instance()->Init();


    int index =0;
    while (ros::ok())
    {   
        sensor_msgs::JointState JointFdb;

        for(int i = 0; i < MOTOR_NUM; i++){
            ChassisMotor[i].Update(); //Update Motor Control
        }

        for(int i = 0; i < MOTOR_NUM; i++){
            
            std::string name = chassisMotorName[i];
            auto iter = chassisMotorNameMap.find(name);
            if(iter == chassisMotorNameMap.end()) //Handler not found
            {
                std::cerr << "ERROR: Motor for joint " << name << "is not registered." << std::endl;
                exit(1);
            }

            M3508* motor = (M3508*) iter->second;

            JointFdb.name.push_back(name);
            JointFdb.effort.push_back(motor -> currentFdb);
            JointFdb.position.push_back(motor -> positionFdb);
            JointFdb.velocity.push_back(motor -> speedFdb);

        }

        motor_data_pub.publish(JointFdb);

        DjiMotorManager::Instance()->Update();//Send Commands
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}