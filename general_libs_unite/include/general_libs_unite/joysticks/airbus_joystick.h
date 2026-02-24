/**
 *    @file  airbus_joystick.h
 *    @brief  Class handler for airbus joystick in ROS2
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  9-Jan-2023
 *    Modification 24-April-2025
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#ifndef _AIRBUS_JOYSTICK_
#define _AIRBUS_JOYSTICK_

#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <memory>

using namespace std;

class airbus_joystick_bravo5_CP : public rclcpp::Node{
      public:
            double  knob = 0;
            double  vel_max = 0.15; //m/s 
            double  teleop_VelX =0.0;   
            double  teleop_VelY =0.0;
            double  teleop_VelZ =0.0;    
            bool enableBaseMotion = false;
            bool goHome = false;
            bool makeReading = false;
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;

            airbus_joystick_bravo5_CP() : Node("joystick") {
                  // Declare the subscriber
                        sub_joy = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&airbus_joystick_bravo5_CP::joy_twist_callback, this, std::placeholders::_1));
            }
            void joy_twist_callback(const sensor_msgs::msg::Joy::SharedPtr msg){ 
                        double knob = msg->axes[3];        
                        double vel = (1 + knob)*vel_max/2;      
                        enableBaseMotion = (bool)(msg->buttons[0]);
                        makeReading = (bool)(msg->buttons[2]);
                        goHome = (bool)(msg->buttons[1]);
                        if (enableBaseMotion){        
                              teleop_VelX = -msg->axes[0]*vel;
                              teleop_VelY = msg->axes[1]*vel;
                              teleop_VelZ = -msg->axes[2]*vel;
                        }
                        else{
                              teleop_VelX = 0;
                              teleop_VelY = 0;
                              teleop_VelZ = 0;          
                        }
                  }
};


#endif 