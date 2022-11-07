#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include "socketCan2/inc/CanManager.hpp"
#include "socketCan2/inc/DjiMotorManager.hpp"
#include "socketCan2/inc/M2006.hpp"
#include "socketCan2/inc/M3508.hpp"

#include "./ChassisController.hpp"
#include "wheelleg_quad/JointData.h"
#include <fstream>

using namespace std;

M3508 ChassisMotor[4];
ROBOT_STATE robot_state = NO_ROBOT_STATE;
int button_back = 0;//wheel
int button_start = 0;//leg

float Vy = 0.0;
float Vw = 0.0;


//parameters about gait
float legInital[4] = {};
float legGaitInit[4] = {};
float walkVx = 0.0;
float walkVy = 0.0;
bool legStateChange = false;
bool legChangeDone = false;
//gait button
bool axes6L = false;
bool axes7U = false;
ofstream ofile_axes6L;
ofstream ofile_axes7U;
LEG_MODE leg_mode = NO_LEG_MODE;

void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // cout<<"buttons: "<<joy->buttons[0]<<" "<<joy->buttons[1]<<" "<<joy->buttons[2]<<" "<<joy->buttons[3]<<" "<<joy->buttons[4]<<" "<<joy->buttons[5]<<" "<<joy->buttons[6]<<" "<<joy->buttons[7]<<" "<<joy->buttons[8]<<" "<<joy->buttons[9]<<" "<<joy->buttons[10]<<endl;
    // cout<<"axis  "<<joy->axes[0]<<" "<<joy->axes[1]<<" "<<joy->axes[2]<<" "<<joy->axes[3]<<" "<<joy->axes[4]<<" "<<joy->axes[5]<<" "<<joy->axes[6]<<" "<<joy->axes[7]<<endl;
    walkVx = joy->axes[3];//Left/Right Axis stick right
    Vw = 5.0 * joy->axes[0];
    Vy = 5.0 * joy->axes[1];
    button_back = joy->buttons[6];
    button_start = joy->buttons[7];
    if (button_back== 1){//wheel
        robot_state = WHEEL_STATE;
        ChassisMotor[0].controlMode = Motor::SPD_MODE; 
        ChassisMotor[1].controlMode = Motor::SPD_MODE;   
        ChassisMotor[2].controlMode = Motor::SPD_MODE;   
        ChassisMotor[3].controlMode = Motor::SPD_MODE;   
    }else if(button_start ==1){//leg
        robot_state = LEG_STATE;
        ChassisMotor[0].controlMode = Motor::POS_MODE; 
        ChassisMotor[1].controlMode = Motor::POS_MODE;
        ChassisMotor[2].controlMode = Motor::POS_MODE;
        ChassisMotor[3].controlMode = Motor::POS_MODE;      
    }else if(joy->axes[6] == 1){//cross key L
        if(axes6L == false){
            ofile_axes6L.open("motor_current.txt");
            axes6L = true;
        }else{
            axes6L = false;
            ofile_axes6L.close();
        }
    }else if(joy->axes[7] == 1){//cross key U
        if(axes7U == false){
            ofile_axes7U.open("motor_speed.txt");
            axes7U = true;
        }else{
            axes7U = false;
            ofile_axes7U.close();
        }
    }else if(joy->buttons[8] == 1){//power
        if(leg_mode == NO_LEG_MODE){
            leg_mode = WALK_MODE;
            cout<<"WALK_MODE"<<endl;
        }else if(leg_mode == WALK_MODE){
            leg_mode = TROT_MODE;
            cout<<"TROT_MODE"<<endl;
        }else if(leg_mode == TROT_MODE){
            leg_mode = CLIMB_MODE;
            cout<<"CLIMB_MODE"<<endl;
        }else if(leg_mode == CLIMB_MODE){
            leg_mode = NO_LEG_MODE;
            cout<<"NO_LEG_MODE"<<endl;
        }
    }
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "chassis");    
    ros::NodeHandle n;  
    ros::Subscriber joy_sub_ = n.subscribe<sensor_msgs::Joy>("joy", 10, &joystickCallback); 
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


    // int runTime = 0;
    while (ros::ok())
    {   
        if(robot_state == WHEEL_STATE){
            ChassisMotor[0].speedSet = Vy - Vw; //RightFront
            ChassisMotor[1].speedSet = -(Vy + Vw); //LeftFront
            ChassisMotor[2].speedSet = Vy - Vw; //RightRear
            ChassisMotor[3].speedSet = -(Vy + Vw); //LeftRear
            
            for(int i = 0; i < 4; i++)
            {
                ChassisMotor[i].Update(); //Update Motor Control
            }
        }else if(robot_state == LEG_STATE){
            // output the angle of four motors
            // if(runTime==50){
            //     ChassisMotor[0].positionSet=1;
            //     ChassisMotor[0].Update();
            //     std::cout << "set:"  <<  ChassisMotor[0].positionSet << "  "; 
            //     std::cout << "resent:"  << ChassisMotor[0].positionFdb << std::endl; 
            //     runTime=0;
            // }else{
            //     runTime++;
            // }
            
            if(legStateChange == false){
                for(int i = 0; i < 4; i++){
                    legInital[i] = ChassisMotor[i].positionFdb;
                    if(leg_mode == NO_LEG_MODE){
                        legGaitInit[i] = legInital[i];
                    }else if(leg_mode == WALK_MODE){
                        legGaitInit[i] = legInital[i] - leg_forward[i]*walk_phase[i];
                    }else if(leg_mode == TROT_MODE){
                        legGaitInit[i] = legInital[i] - leg_forward[i]*trot_phase[i];
                    }else if(leg_mode == CLIMB_MODE){
                        legGaitInit[i] = legInital[i] - leg_forward[i]*climb_phase[i];
                    }
                }
                legStateChange = true;
            }

            if(legChangeDone==false){
                for(int i = 0; i < 4; i++){
                    ChassisMotor[i].positionSet  = legGaitInit[i];
                    ChassisMotor[i].Update(); //Update Motor Control
                }
                int reachWalk = 0;
                for(int i = 0; i < 4; i++){
                    if (abs(ChassisMotor[i].positionSet-ChassisMotor[i].positionFdb)>POS_THRESHOLD){
                        reachWalk -= 1;
                    }
                }
                if(reachWalk == 0){
                    legChangeDone = true;
                }
            }else{
                for(int i = 0; i < 4; i++){
                    ChassisMotor[i].positionSet += 0.1*walkVx*leg_forward[i];
                    ChassisMotor[i].Update(); //Update Motor Control
                }
            }
        }

        if(axes6L == true){
            ofile_axes6L<<ChassisMotor[0].get_current()<<"  "<<ChassisMotor[1].get_current()<<"  "<<ChassisMotor[2].get_current()<<"  "<<ChassisMotor[3].get_current()<<endl;
        }   
        if(axes7U == true){
            ofile_axes7U<<ChassisMotor[0].get_speed()<<endl;
        }     

        DjiMotorManager::Instance()->Update();//Send Commands

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
