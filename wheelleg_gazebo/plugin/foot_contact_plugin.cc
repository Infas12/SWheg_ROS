/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <string>
#include <gazebo/common/Events.hh>
#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <geometry_msgs/WrenchStamped.h>


float getSqrNorm(const gazebo::msgs::Vector3d& vector)
{
    float normsqr = vector.x()*vector.x() + vector.y()*vector.y() + vector.z()*vector.z();
    return normsqr;
}

namespace gazebo
{
    class UnitreeFootContactPlugin : public SensorPlugin
    {
        public:
        UnitreeFootContactPlugin() : SensorPlugin(){}
        ~UnitreeFootContactPlugin(){}

        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
        {
            this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor); // Make sure the parent sensor is valid.        
            if (!this->parentSensor){
                gzerr << "UnitreeFootContactPlugin requires a ContactSensor.\n";
                return;
            }
            this->contact_namespace = "contact/";
            this->rosnode = new ros::NodeHandle(this->contact_namespace);
            // add "visual" is for the same name of draw node
            this->force_pub = this->rosnode->advertise<geometry_msgs::WrenchStamped>("/visual/"+_sensor->Name()+"/the_force", 100);
            // Connect to the sensor update event.
            this->update_connection = this->parentSensor->ConnectUpdated(std::bind(&UnitreeFootContactPlugin::OnUpdate, this));
            this->parentSensor->SetActive(true); // Make sure the parent sensor is active.
            count = 0;
            Fx = 0;
            Fy = 0;
            Fz = 0;
            ROS_INFO("Load %s plugin.", _sensor->Name().c_str());
        }

        private:
        void OnUpdate()
        {
            
            msgs::Contacts contacts;
            contacts = this->parentSensor->Contacts();
            count = contacts.contact_size();
            int validContactCount = 0;

            for (unsigned int i = 0; i < count; ++i){
                
                if(contacts.contact(i).position_size() != 1){
                    //ROS_ERROR("Contact count isn't correct!!!!");
                }

                for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j){                 

                    // if(getSqrNorm(contacts.contact(i).wrench(j).body_1_wrench().force())<0.001)
                    // {
                    //     continue;
                    // }

                    validContactCount++;

                    // std::cout << "Collision between[" << contacts.contact(i).collision1()
                    //           << "] and [" << contacts.contact(i).collision2() << std::endl;
                    
                    // std::cout << "contact" << j+1 << "of" << contacts.contact(i).position_size() <<" Wrench:"
                    //           << contacts.contact(i).wrench(j).body_1_wrench().force().x() << " "
                    //           << contacts.contact(i).wrench(j).body_1_wrench().force().y() << " "
                    //           << contacts.contact(i).wrench(j).body_1_wrench().force().z() << " "
                    //           << contacts.contact(i).wrench(j).body_1_wrench().torque().x() << " "
                    //           << contacts.contact(i).wrench(j).body_1_wrench().torque().y() << " "
                    //           << contacts.contact(i).wrench(j).body_1_wrench().torque().z() << "\n";
                    
                    // std::cout << j << "  Position:"
                    //             << contacts.contact(i).position(j).x() << " "
                    //             << contacts.contact(i).position(j).y() << " "
                    //             << contacts.contact(i).position(j).z() << "\n";
                    // std::cout << "   Normal:"
                    //             << contacts.contact(i).normal(j).x() << " "
                    //             << contacts.contact(i).normal(j).y() << " "
                    //             << contacts.contact(i).normal(j).z() << "\n";
                    // std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";

                    Fx += contacts.contact(i).wrench(j).body_1_wrench().force().x(); // Notice: the force is in local coordinate, not in world or base coordnate.
                    Fy += contacts.contact(i).wrench(j).body_1_wrench().force().y();
                    Fz += contacts.contact(i).wrench(j).body_1_wrench().force().z();

                }

                //std::cout<< "Force Magnitude" << Fx*Fx + Fy*Fy + Fz*Fz << std::endl;
            }

            if(validContactCount != 0){
                force.wrench.force.x = Fx/double(count);
                force.wrench.force.y = Fy/double(count);
                force.wrench.force.z = Fz/double(count);
            }else{
                force.wrench.force.x = 0;
                force.wrench.force.y = 0;
                force.wrench.force.z = 0;
                force.wrench.torque.x = 0; //Using torque field to store contact point infomation
                force.wrench.torque.y = 0;
                force.wrench.torque.z = 0;
            }

            count = 0;
            Fx = 0;
            Fy = 0;
            Fz = 0;
            this->force_pub.publish(force);
        }

        private:
            ros::NodeHandle* rosnode;
            ros::Publisher force_pub;
            event::ConnectionPtr update_connection;
            std::string contact_namespace;
            sensors::ContactSensorPtr parentSensor;      
            geometry_msgs::WrenchStamped force;
            int count = 0;
            double Fx=0, Fy=0, Fz=0;
            double Px=0, Py=0, Pz=0;
    };
    GZ_REGISTER_SENSOR_PLUGIN(UnitreeFootContactPlugin)
}
    