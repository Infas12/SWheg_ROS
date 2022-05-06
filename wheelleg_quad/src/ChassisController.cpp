#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include "./ChassisController.hpp"
#include "wheelleg_quad/JointData.h"
#include "wheelleg_quad/MotorData.h"

wheelleg_quad::JointData msgJoint; 
#define WHEEL_MODE true
#define LEG_MODE false

ROBOT_STATE robot_state = NO_ROBOT_STATE;
uint8_t button_back = 0;//wheel
uint8_t button_start = 0;//leg

float Vx = 0.0;
float Vy = 0.0;
float legVx = 0.0;

void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    legVx = joy->axes[3];//UD Axis stick right
    Vy = 5.0 * joy->axes[0];
    Vx = 5.0 * joy->axes[1];
    button_back = joy->buttons[6];
    button_start = joy->buttons[7];
    if (button_back== 1){//wheel
        robot_state = WHEEL_STATE;
        for(int i = 0; i<4;i++){
            msgJoint.JointMode[i] = WHEEL_MODE;
        }
    }else if(button_start ==1){//leg
        robot_state = LEG_STATE;
        for(int i = 0; i<4; i++){
            msgJoint.JointMode[i] = LEG_MODE;
        }
    }
    
}

void motorDataCallback(const wheelleg_quad::MotorData::ConstPtr& msg)
{
    // ROS_WARN("current:%f   position:%f  speed:%f ", msg->current[0], msg->position[0], msg->speed[0]);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chassisController");    
    ros::NodeHandle n;  
    ros::Subscriber joy_sub_ = n.subscribe<sensor_msgs::Joy>("joy", 10, &joystickCallback); 
    ros::Publisher control_pub = n.advertise<wheelleg_quad::JointData>("control", 10);
    ros::Subscriber motor_data_sub = n.subscribe<wheelleg_quad::MotorData>("motor", 10, &motorDataCallback); 
    ros::Rate loop_rate(100);

    while (ros::ok())
    {   
        if(robot_state == WHEEL_STATE){
            msgJoint.JointData[0] = Vx - Vy; //RightFront
            msgJoint.JointData[1] = -(Vx + Vy); //LeftFront
            msgJoint.JointData[2] = Vx - Vy; //RightRear
            msgJoint.JointData[3] = -(Vx + Vy); //LeftRear
            
        }else if(robot_state == LEG_STATE){
            for(int i = 0; i < 4; i++){
                msgJoint.JointData[i] += 0.1*legVx*leg_forward[i];

            }
        }

        control_pub.publish(msgJoint);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
