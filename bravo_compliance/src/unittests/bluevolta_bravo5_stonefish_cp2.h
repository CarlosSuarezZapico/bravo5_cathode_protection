/**
 *    @file  bluevolta_bravo5_stonefish.h
 *    @brief Library for stonefish simulator on Bluevolta and Bravo5
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  22-Aug-2024
 *    Modification 16-Sep-2024
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#ifndef _BLUEVOLTA_BRAVO5_STONEFISH_
#define _BLUEVOLTA_BRAVO5_STONEFISH_

#include <vector>
#include <thread>
#include <unistd.h>
#include <chrono>
#include <math.h>
#include <cstring>
#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <iomanip> 
#include <exception>
#include <cerrno>
#include <cmath>
#include <tuple>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp> 
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "stonefish_ros2/msg/dvl.hpp"
#include "stonefish_ros2/msg/ins.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "uvm_falcon_bravo/utils/general_utils.h"
#include "uvm_falcon_bravo/interaction/interaction_control.h"
#include "uvm_falcon_bravo/collision/collisions.h"

#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp" 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"

using namespace std;
using namespace general_utils;
using namespace Eigen;
using namespace pinocchio;

/**
 * @brief bravo class interface with stonefish simulator
 */
class bravo_sim_stonefish : public rclcpp::Node{
    public:
        Eigen::Vector<double, 6> current_wrench_ee;
        Eigen::Vector<double, 4> current_joint_position, joint_cmd;
        Eigen::Vector<double, 4> pinocchio_stonefish_bias;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr          pubJointsStateSim;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       subJointFdb;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr  subFT;


    public:
        bravo_sim_stonefish() : rclcpp::Node("bravo_node"){
            //There is a bias between the URDF and the SCN file used in stonefish
            pinocchio_stonefish_bias << M_PI, M_PI/2, M_PI/2, 0.0;
            joint_cmd << 0.0, 0.0, 0.0, 0.0;
            current_joint_position << 0.0, 0.0, 0.0, 0.0;
            current_wrench_ee << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            //& PUBLISHERS AND SUBSCRIBERS
            pubJointsStateSim  = this->create_publisher<sensor_msgs::msg::JointState>("/bluevolta/servos/joint_setpoints", 10);
            subJointFdb        = this->create_subscription<sensor_msgs::msg::JointState>("/bluevolta/servos/joint_states", 10, std::bind(&bravo_sim_stonefish::jointFdb_callback, this, std::placeholders::_1));
            subFT              = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/bravo/FT", 10, std::bind(&bravo_sim_stonefish::FT_callback, this, std::placeholders::_1));
        }

        bool is_in_desired_configuration(double tolerance, Eigen::Vector<double, 4> desired_configuration){
            Eigen::Vector<double, 4> error; 
            error = desired_configuration - current_joint_position;
            RCLCPP_WARN_ONCE(this->get_logger(), "Error 0;", error(0), "1:", error(1), "2:", error(2), "3:", error(3));
            if ((abs(error(0))<tolerance) && (abs(error(1))<tolerance) && (abs(error(2))<tolerance) && (abs(error(3))<tolerance)){
                RCLCPP_WARN_ONCE(this->get_logger(), "Already in desired configuration");
                return true;
            } 
            else{
                return false;
            }
        }

        Eigen::Vector<double, 4> pinocchio2stonefish(Eigen::Vector<double, 4> pinocchio_in){
            Eigen::Vector<double, 4> stonefish_out;
            stonefish_out = pinocchio_in - pinocchio_stonefish_bias;
            return stonefish_out;
        }
        
        Eigen::Vector<double, 4> stonefish2pinocchio(Eigen::Vector<double, 4> stonefish_in){
            Eigen::Vector<double, 4> pinocchio_out;
            pinocchio_out = stonefish_in + pinocchio_stonefish_bias;
            return pinocchio_out;
        }

        void going2joint_pos(Eigen::Vector<double, 4> goal_joint_position, double joint_vel, double sampling_time){            
            // Calculate the difference between the goal and current joint positions
            Eigen::Vector<double, 4> delta_joint = goal_joint_position - current_joint_position;            
            // Normalize the delta to get direction of movement and scale by velocity
            Eigen::Vector<double, 4> delta_joint_normalized = delta_joint.normalized();            
            // Interpolate by moving towards the goal configuration
            joint_cmd = joint_cmd + delta_joint_normalized * joint_vel * sampling_time;
            // Make sure we don't overshoot the goal
            for (int i = 0; i < 4; i++) {
                if (std::abs(joint_cmd[i] - goal_joint_position[i]) < std::abs(0.02)) {
                    joint_cmd[i] = goal_joint_position[i];  // Clamp to goal if overshooting
                }
            }
            // Output the joint command
            set_joint_pos(joint_cmd);
        }
        
        void FT_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg){
            current_wrench_ee << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;            
        }

        void jointFdb_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
            current_joint_position << msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5];            
        }
        
        void set_joint_pos(Eigen::Vector<double, 4> joint_pos){
            sensor_msgs::msg::JointState msg_joint;
			msg_joint.header.stamp = rclcpp::Clock().now();    
			msg_joint.name.push_back("bluevolta/bravo/joint1");
			msg_joint.name.push_back("bluevolta/bravo/joint2");
			msg_joint.name.push_back("bluevolta/bravo/joint3");
			msg_joint.name.push_back("bluevolta/bravo/joint4");
			msg_joint.position.push_back(joint_pos(0));
			msg_joint.position.push_back(joint_pos(1));
			msg_joint.position.push_back(joint_pos(2));
			msg_joint.position.push_back(joint_pos(3));
			pubJointsStateSim->publish(msg_joint);
        }
};

/**
 * @brief Bluevolta class interface with stonefish simulator
 */

class bluevolta_sim_stonefish : public rclcpp::Node{

    private:
        Eigen::Matrix<double, 4, 7> collocationMatrix;
        Eigen::Matrix<double, 7, 4> invCollocationMatrix; 
        Eigen::Matrix<double, 4, 1> desired_effort;
        Eigen::Matrix<double, 7, 1> UnitaryX, UnitaryY, UnitaryZ, UnitaryAngX, UnitaryAngY, UnitaryAngZ;
        Eigen::Vector<double, 4> PD_prev_error; 
        double PD_sampling_time = 0.005;

    
    public:
        Eigen::Vector3d dvl_velocity, gyro_angular, orientation;
        float P_gain = 2.0;
        float P_gain_yaw = 1;

        float angle4547 = 51.5442; //Angle for T4, T5, T4, T7
        float angle123 = 13.22; //Angle for T1, T2, T3

        std::chrono::high_resolution_clock::time_point PD_last_sampling_time;
        
        //& PUBLISHERS AND SUBSCRIBERS
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr                            subGoalPoint;
        rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr                             subDVL;
        rclcpp::Subscription<stonefish_ros2::msg::INS>::SharedPtr                             subINS;
        rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr       subGyro;
        rclcpp::Publisher<std_msgs::msg::Float44MultiArray>::SharedPtr                        pubFalconThruster;

    public: 
        bluevolta_sim_stonefish() : rclcpp::Node("bluevolta_node"){
            desired_effort << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            dvl_velocity << 0.0, 0.0, 0.0;  
            orientation << 0.0, 0.0, 0.0;
            gyro_angular << 0.0, 0.0, 0.0;
             
            //& ALLOCATION MATRIX       
            double angle4547_rad = angle4547 * 3.141592454 / 180.0;
            double angle123_rad = angle123 * 3.141592454 / 180.0;

            collocationMatrix << 0.0, 0.0, 0.0, -cos(angle4547_rad), -cos(angle4547_rad), -cos(angle4547_rad), -cos(angle4547_rad), //x
                                 0.0, 0.0, 0.0, sin(angle4547_rad), -sin(angle4547_rad), -sin(angle4547_rad), sin(angle4547_rad), //y
                                 cos(angle123_rad), cos(angle123_rad), cos(angle123_rad), 0.0, 0.0, 0.0, 0.0, //z 
                                 (0.28*sin(angle123_rad)+0.34*cos(angle123_rad)), -(0.28*sin(angle123_rad)+0.34*cos(angle123_rad)), 0.0, 0.0, 0.0, 0.0, 0.0, //ang_x
                                 -0.25, -0.25, (0.3*sin(angle123_rad)+0.4*cos(angle123_rad)), 0.0, 0.0, 0.0, 0.0, //ang_y 
                                 0.0, 0.0, 0.0, (0.31*cos(angle4547_rad)+0.52*sin(angle4547_rad)), -(0.31*cos(angle4547_rad)+0.52*sin(angle4547_rad)), (0.27*cos(angle4547_rad)+0.4*sin(angle4547_rad)), -(0.27*cos(angle4547_rad)+0.4*sin(angle4547_rad)); //ang_z
            invCollocationMatrix = collocationMatrix.completeOrthogonalDecomposition().pseudoInverse();

            desired_effort << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            Eigen::Matrix<double, 7, 1> thruster_cmd = invCollocationMatrix * desired_effort;
            Eigen::Matrix<double, 7, 1> thruster_unitaryX = thruster_cmd / thruster_cmd.norm(); 
            UnitaryX << thruster_unitaryX(0), thruster_unitaryX(1), thruster_unitaryX(2), thruster_unitaryX(3), thruster_unitaryX(4), thruster_unitaryX(5), thruster_unitaryX(6);            
            
            desired_effort << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
            thruster_cmd = invCollocationMatrix * desired_effort;
            Eigen::Matrix<double, 7, 1> thruster_unitaryY = thruster_cmd / thruster_cmd.norm(); 
            UnitaryY << thruster_unitaryY(0), thruster_unitaryY(1), thruster_unitaryY(2), thruster_unitaryY(3), thruster_unitaryY(4), thruster_unitaryY(5), thruster_unitaryY(6);

            desired_effort << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
            thruster_cmd = invCollocationMatrix * desired_effort;
            Eigen::Matrix<double, 7, 1> thruster_unitaryZ = thruster_cmd / thruster_cmd.norm(); 
            UnitaryZ << thruster_unitaryZ(0), thruster_unitaryZ(1), thruster_unitaryZ(2), thruster_unitaryZ(3), thruster_unitaryZ(4), thruster_unitaryZ(5), thruster_unitaryZ(6);

            desired_effort << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
            thruster_cmd = invCollocationMatrix * desired_effort;
            Eigen::Matrix<double, 7, 1> thruster_unitaryAngX = thruster_cmd / thruster_cmd.norm(); 
            UnitaryAngX << thruster_unitaryAngX(0), thruster_unitaryAngX(1), thruster_unitaryAngX(2), thruster_unitaryAngX(3), thruster_unitaryAngX(4), thruster_unitaryAngX(5), thruster_unitaryAngX(6);

            desired_effort << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
            thruster_cmd = invCollocationMatrix * desired_effort;
            Eigen::Matrix<double, 7, 1> thruster_unitaryAngY = thruster_cmd / thruster_cmd.norm(); 
            UnitaryAngY << thruster_unitaryAngY(0), thruster_unitaryAngY(1), thruster_unitaryAngY(2), thruster_unitaryAngY(3), thruster_unitaryAngY(4), thruster_unitaryAngY(5), thruster_unitaryAngY(6);

            desired_effort << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
            thruster_cmd = invCollocationMatrix * desired_effort;
            Eigen::Matrix<double, 7, 1> thruster_unitaryAngZ = thruster_cmd / thruster_cmd.norm(); 
            UnitaryAngZ << thruster_unitaryAngZ(0), thruster_unitaryAngZ(1), thruster_unitaryAngZ(2), thruster_unitaryAngZ(3), thruster_unitaryAngZ(4), thruster_unitaryAngZ(5), thruster_unitaryAngZ(6);

            PD_last_sampling_time = std::chrono::high_resolution_clock::now();
            
            //& PUBLISHERS AND SUBSCRIBERS
            pubFalconThruster  = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bluevolta/controller/thruster_setpoints_sim", 10);
            subGoalPoint       = this->create_subscription<geometry_msgs::msg::Twist>("/airbus/falcon/twist", 10, std::bind(&bluevolta_sim_stonefish::twist_bluevolta_callback, this, std::placeholders::_1));
            subDVL             = this->create_subscription<stonefish_ros2::msg::DVL>("/bluevolta/dvl", 10, std::bind(&bluevolta_sim_stonefish::DVL_callback, this, std::placeholders::_1) );
            subGyro            = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/bluevolta/gyro", 10, std::bind(&bluevolta_sim_stonefish::gyro_callback, this, std::placeholders::_1) );
            subINS             = this->create_subscription<stonefish_ros2::msg::INS>("/bluevolta/INS", 10, std::bind(&bluevolta_sim_stonefish::INS_callback, this, std::placeholders::_1) );
        }

        void twist_bluevolta_callback(const geometry_msgs::msg::Twist::SharedPtr msg){ 
            twist_control(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
        }

        void twist_control(double x, double y, double z, double yaw){
            double gain_X=0.0, gain_Y=0.0, gain_Z=0.0, gain_AngX=0.0, gain_AngY=0.0, gain_AngZ=0.0;
            bool computed_action_X = false;
            std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - PD_last_sampling_time;
            if (elapsed.count() > abs(PD_sampling_time)) {
                gain_X    = PD_controller(x,  -dvl_velocity(0), P_gain, 1, PD_sampling_time, PD_prev_error(0));
                gain_Y    = PD_controller(y,  dvl_velocity(1), P_gain, 1, PD_sampling_time, PD_prev_error(1));
                gain_Z    = PD_controller(z,  dvl_velocity(2), P_gain, 1, PD_sampling_time, PD_prev_error(2));
                gain_AngX = PD_controller(0.0, orientation(1), 1, 10, PD_sampling_time, PD_prev_error(3));
                gain_AngY = PD_controller(0.0, orientation(0), 2, 20, PD_sampling_time, PD_prev_error(4));
                gain_AngZ = PD_controller(yaw, -gyro_angular(2), P_gain_yaw, 10, PD_sampling_time, PD_prev_error(5));
                Eigen::Matrix<double, 7, 1> thruster1234567 = gain_X * UnitaryX + gain_Y * UnitaryY + gain_Z * UnitaryZ + gain_AngX * UnitaryAngX + gain_AngY * UnitaryAngY + gain_AngZ * UnitaryAngZ;
                if (std::isnan(thruster1234567(0))){ 
                    thruster1234567(0) = 0.0;
                }
                if (std::isnan(thruster1234567(1))){ 
                    thruster1234567(1) = 0.0;
                }
                if (std::isnan(thruster1234567(2))){ 
                    thruster1234567(2) = 0.0;
                }
                if (std::isnan(thruster1234567(3))){ 
                    thruster1234567(3) = 0.0;
                }
                if (std::isnan(thruster1234567(4))){ 
                    thruster1234567(4) = 0.0;
                }
                if (std::isnan(thruster1234567(5))){ 
                    thruster1234567(5) = 0.0;
                }
                if (std::isnan(thruster1234567(6))){ 
                    thruster1234567(6) = 0.0;
                }

                auto thrust_control_msg = std_msgs::msg::Float64MultiArray();
                thrust_control_msg.layout.dim.resize(1);
                thrust_control_msg.layout.dim[0].label  = "effort_bluevolta";
                thrust_control_msg.layout.dim[0].size   = 7;
                thrust_control_msg.layout.dim[0].stride = 7;
                thrust_control_msg.layout.data_offset   = 0;
                // Set the data
                thrust_control_msg.data = {thruster1234567(0), thruster1234567(1), thruster1234567(2), thruster1234567(3), thruster1234567(4), thruster1234567(5), thruster1234567(6)};
                pubFalconThruster->publish(thrust_control_msg);
                PD_last_sampling_time = std::chrono::high_resolution_clock::now();
                }
        }
        
        double PD_controller(double desired_value, double current_value, double P, double D, double sampling_time, double &prev_error){
            double output = 0.0;
            double error = desired_value - current_value; 
            prev_error = error; 
            double dev_error = (error - prev_error)/sampling_time;
            output = P * error + D * dev_error;
            return output;
        }

        std::tuple<bool, Eigen::Vector<double,4>> PD_controller_XYZyaw(Eigen::Vector<double,4> desired_value, Eigen::Vector<double,4> current_value, double P, double D, double sampling_time, std::chrono::high_resolution_clock::time_point last_call_time, Eigen::Vector<double,4> prev_error){
            std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - PD_last_sampling_time;
            Eigen::Vector<double,4> output;
            output << 0.0, 0.0, 0.0, 0.0;
            if (elapsed.count() > abs(sampling_time)) {
                Eigen::Vector<double,4> error = desired_value - current_value; 
                Eigen::Vector<double,4> dev_error = (error - prev_error)/elapsed.count();
                output = P * error + D * dev_error;
                PD_last_sampling_time =  std::chrono::high_resolution_clock::now();
                return std::make_tuple(true, output);
            }
            else{
                return std::make_tuple(false, output);
            }
        }
    
        void DVL_callback(const stonefish_ros2::msg::DVL::SharedPtr msg){
            dvl_velocity << -msg->velocity.z, msg->velocity.x, msg->velocity.y;             
        }

        void INS_callback(const stonefish_ros2::msg::INS::SharedPtr msg){
            orientation << msg->pose.roll, msg->pose.pitch, msg->pose.yaw;            
        }

        void gyro_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg){
            gyro_angular << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;            
        }
};

/**
 * @brief uvm_sim_stonefish class to handle whole-body motion control
 */
class uvm_sim_stonefish : public rclcpp::Node{
    private:
    	std::vector<std::string> joint_names;
        std::string frame_name;
        
        //& Base Penalties for mobile base
		double penaltyBaseX   = pow(10, 40);
        double penaltyBaseY   = pow(10, 40);
		double penaltyBaseZ   = pow(10, 40);
		double penaltyBaseYaw = pow(10, 40);  

        //& Filter parameter
        double vel_max = 0.15;
        double accX = 0.1; double accY=0.1; double accZ=0.1; double accYaw =0.2;

    
		//& Initial Joint States
        double transiet_interval = 10*(3.14/180);
        //q1 should be CONTINUOUS with the new upgrade
        double q_joint1_min = 0.0; double q_joint1_max = 6.1;    double q_joint1_iInf = q_joint1_min + transiet_interval; double q_joint1_iSup = q_joint1_max - transiet_interval; 
		double q_joint2_min = 0.0; double q_joint2_max = 3.14;   double q_joint2_iInf = q_joint2_min + transiet_interval; double q_joint2_iSup = q_joint2_max - transiet_interval; 
		double q_joint3_min = 0.0; double q_joint3_max = 3.14;   double q_joint3_iInf = q_joint3_min + transiet_interval; double q_joint3_iSup = q_joint3_max - transiet_interval; 
		
    public:
        Eigen::Vector<double,8> q;
        std::chrono::duration<double> elapsed_request;
		std::chrono::high_resolution_clock::time_point t1, t2, last_integration_time, finish_integration_time, start_integration_time, last_filter, start_bravo_integration_time, finish_bravo_integration_time, last_bravo_integration_time;
    	Eigen::Vector<double,8> joint_whole_integration, joint_vel_integration, secondary_vel;
        Eigen::Vector<double,4> joint_pos_bravo_integration, joint_vel_bravo_integration;
		Eigen::Vector<double, 6> primary_vel;
        double vel_pre_x =0.0; double vel_pre_y =0.0; double vel_pre_z =0.0; double vel_pre_yaw = 0.0;
        
        //& Pinocchion handler
        Model model_uvm;
        Data data_uvm;
        Model model_manipulator;
		Data data_manipulator;
        
        //& Handler for bravo and bluevolta
        bravo_sim_stonefish bravo;
        bluevolta_sim_stonefish bluevolta;

    public:
        uvm_sim_stonefish(const std::string urdf_filename_uvm, const std::string urdf_filename_manipulator, const std::string tool_link) : rclcpp::Node("bluevolta_bravo_stonefish_sim"), data_uvm(), data_manipulator(){	
            frame_name = tool_link;
            pinocchio::urdf::buildModel(urdf_filename_uvm, model_uvm);
            pinocchio::urdf::buildModel(urdf_filename_manipulator, model_manipulator);
            data_uvm = Data(model_uvm);
            data_manipulator = Data(model_manipulator); 
            joint_names = model_uvm.names; 
            q << 0.0, 0.0, 0.0, 0.0, 1.7, 2.7, 0.66, 3.2, 1.8, 0.0;
            joint_whole_integration = q; 
            t1 = std::chrono::high_resolution_clock::now();
            t2 = std::chrono::high_resolution_clock::now();
            last_filter = std::chrono::high_resolution_clock::now();
            start_integration_time = std::chrono::high_resolution_clock::now();
            start_bravo_integration_time = std::chrono::high_resolution_clock::now();
        }
        
        Eigen::Vector<double, 4> filter_base(double vel_base_x, double vel_base_y, double vel_base_z, double vel_base_yaw){
            Eigen::Vector<double, 4> out_velocity;
            std::chrono::duration<double> elapsed_filter= std::chrono::high_resolution_clock::now()-last_filter;
            double sampling_control_filter = elapsed_filter.count();
            double secondary_saturated_x   = VAL_SAT<double>(vel_base_x, vel_max, -vel_max);
            double secondary_saturated_y   = VAL_SAT<double>(vel_base_y, vel_max, -vel_max);
            double secondary_saturated_z   = VAL_SAT<double>(vel_base_z, vel_max, -vel_max);
            double secondary_saturated_yaw = VAL_SAT<double>(vel_base_yaw, 0.9, -0.9);
            out_velocity(0) = VAL_SAT<double>( secondary_saturated_x, vel_pre_x + accX*sampling_control_filter,  vel_pre_x - accX*sampling_control_filter);
            out_velocity(1) = VAL_SAT<double>( secondary_saturated_y, vel_pre_y + accY*sampling_control_filter,  vel_pre_y - accY*sampling_control_filter);
            out_velocity(2) = VAL_SAT<double>( secondary_saturated_z, vel_pre_z + accZ*sampling_control_filter,  vel_pre_z - accZ*sampling_control_filter);
            out_velocity(3) = VAL_SAT<double>( secondary_saturated_yaw, vel_pre_yaw + accYaw*sampling_control_filter,  vel_pre_yaw - accYaw*sampling_control_filter);
            vel_pre_x =out_velocity(0); vel_pre_y = out_velocity(1); vel_pre_z = out_velocity(2); vel_pre_yaw = out_velocity(3);
            last_filter =  std::chrono::high_resolution_clock::now();
            return out_velocity;
        }

        MatrixXd PseudoInverse(MatrixXd matrixInput){
            MatrixXd matrixInputInv;
            matrixInputInv = matrixInput.completeOrthogonalDecomposition().pseudoInverse();
            return matrixInputInv;
        }

        double cond_arm(Eigen::MatrixXd J){ 
            JacobiSVD<MatrixXd> svd(J);
            return svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
        }

		Eigen::Matrix<double, 3, 8> jacobian_whole_vehicle_xyz( Eigen::Vector<double, 8> q){
			Eigen::Vector<double,5> q_mod;
			q_mod << 0.0, q(4), q(5), q(6), q(7);
			std::string frame_name = "contact_point"; // Change this to your desired frame name
			pinocchio::FrameIndex frame_id = model_uvm.getFrameId(frame_name);			
			Eigen::MatrixXd J2(6, 5); // Jacobian matrix (6xN, where N is the number of joints)
			Eigen::MatrixXd J1(6, 3); // Jacobian matrix (6xN, where N is the number of joints)
			J1 <<1, 0, 0,
			     0, 1, 0,
				 0, 0, 1, 
				 0, 0, 0,
				 0, 0, 0,
				 0, 0, 0; 

			pinocchio::computeFrameJacobian (model_uvm, data_uvm, q_mod, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J2);
			Eigen::MatrixXd Jt(6, 8); // Jacobian matrix (6xN, where N is the number of joints)
			Jt << J1, J2;
			Eigen::MatrixXd Jt_return(3, 8);
			// Select rows 1, 2, 3, and 6 (indices 0, 1, 2, and 5) and assign them to Jt_local_return
			Jt_return.row(0) = Jt.row(0); // row 1
			Jt_return.row(1) = Jt.row(1); // row 2
			Jt_return.row(2) = Jt.row(2); // row 3
			return Jt_return;
		}

		Eigen::Matrix<double, 3, 4> local_jacobian_manipulator_xyz (Eigen::Vector<double,4> q){
			std::string frame_name = "contact_point"; // Change this to your desired frame name
			pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(frame_name);			
			Eigen::MatrixXd J(6, 4); // Jacobian matrix (6xN, where N is the number of joints)
			pinocchio::computeFrameJacobian (model_manipulator, data_manipulator, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
			Eigen::MatrixXd Jt_local(6, 4); // Jacobian matrix (6xN, where N is the number of joints)
			forwardKinematics(model_manipulator,data_manipulator,q);
			Eigen::Matrix3d R = data_manipulator.oMi[6].rotation();
			Matrix<double,3,3> zeros_m;
            zeros_m << 0, 0, 0, 0, 0, 0, 0, 0, 0;
			Matrix<double,6,6> matrixRotAux; 
            matrixRotAux << R.transpose(), zeros_m,  zeros_m, R.transpose();
			Jt_local = matrixRotAux*J;
			Eigen::MatrixXd Jt_local_return(3, 4);
			Jt_local_return.row(0) = Jt_local.row(0); // row 1
			Jt_local_return.row(1) = Jt_local.row(1); // row 2
			Jt_local_return.row(2) = Jt_local.row(2); // row 3
			return Jt_local_return;
		}

        Eigen::Vector<double,4> manipulator_ik_diff(Eigen::Vector<double,4> q, Eigen::Vector<double,3> twist){
            Eigen::Vector<double,4> vel_output_joint;
            Eigen::Matrix<double, 3, 4> LocalJacobian = local_jacobian_manipulator_xyz(q);
            Eigen::Matrix<double, 4, 3> invLocalJacobian = LocalJacobian.inverse();
            vel_output_joint = invLocalJacobian*twist;
            return vel_output_joint;
        }

		Eigen::Vector<double,8> weighted_jointLimits_robust_mobile_axis_ik_diff_xyz(Eigen::Vector<double,8> q, Eigen::Vector<double,3> twist, Eigen::Vector<double,8> sedondaryVel, double penaltyBaseX, double penaltyBaseY, double penaltyBaseZ, double penaltyBaseYaw){
			Eigen::Vector<double,8> vel_output_joint;
			DiagonalMatrix<double, 8, 8> WeightMatrix;
			Matrix<double,8,8> Wmatrix;
			//& weights for joint limits. Joint4 and Joint6 are continuous son no penalization
			double w1 = weight_joint_limit_smooth(q(4), q_joint1_min, q_joint1_iInf, q_joint1_iSup,  q_joint1_max);
            double w2 = weight_joint_limit_smooth(q(5), q_joint2_min, q_joint2_iInf, q_joint2_iSup,  q_joint2_max);
            double w3 = weight_joint_limit_smooth(q(6), q_joint3_min, q_joint3_iInf, q_joint3_iSup,  q_joint3_max);
			WeightMatrix.diagonal() << penaltyBaseX, penaltyBaseY, penaltyBaseZ, penaltyBaseYaw,  w1, w2, w3, 1.0;
			Wmatrix = WeightMatrix;
			Matrix<double, 8, 8> invWeightMatrix;
			invWeightMatrix = Wmatrix.inverse();
			double  epsilon = 0.045;
			double  delta_max = 0.2;//0.06
			Matrix<double, 3,8> localJacobian;
			localJacobian = local_jacobian_whole_vehicle_xyz(q);    
			Matrix<double, 8,3> localJacobianT;
			localJacobianT = localJacobian.transpose();    
			JacobiSVD<MatrixXd> svd(localJacobian);
			double lowest_sing_value = svd.singularValues()(2); 
			double delta;
			if (lowest_sing_value >= epsilon){
				delta = 0;
			}
			else{
				delta = (1 - pow((lowest_sing_value/epsilon), 2))*pow(delta_max, 2);
			}
			Matrix<double, 8, 3> component0;
			Matrix<double, 3, 3> component022; //probably fail here
			MatrixXd component02, component021;
			Matrix<double, 8, 3> component01;
			component01 = (invWeightMatrix*localJacobianT);			
			component021 = (localJacobian*invWeightMatrix)*localJacobianT + pow(delta, 2)*MatrixXd::Identity(3, 3);
			component02 = PseudoInverse(component021); 
			component022 = component02; 
			component0 = component01*component022;
			Matrix<double, 8, 1> component1;
			component1 = component0*twist;			
			Matrix<double, 8, 1> component2;
			Matrix<double, 8, 8> component21, I8;
			component21 = (component0 * localJacobian);
			I8 = MatrixXd::Identity(8, 8);
			component2 = (I8 - component21) * sedondaryVel;
			vel_output_joint = component1 + component2;			
			return vel_output_joint;
		}

        double weight_joint_limit_smooth(double q, double qmin, double qinf, double qsup, double qmax) {
            double ais = 2;
            double bis = -3 * (qmax + qsup);
            double cis = 6 * qmax * qsup;
            double dis = pow(qmax, 3) - 3 * pow(qmax, 2) * qsup;
            double aii = 2;
            double bii = -3 * (qmin + qinf);
            double cii = 6 * qmin * qinf;
            double dii = pow(qmin, 3) - 3 * pow(qmin, 2) * qinf;
            double w;
            if ((q >= qsup) && (q <= qmax)) {
                w = (1 / pow((qmax - qsup), 3)) * (ais * pow(q, 3) + bis * pow(q, 2) + cis * q + dis);
            } 
            else if ((q >= qmin) && (q <= qinf)) {
                w = (1 / pow((qmin - qinf), 3)) * (aii * pow(q, 3) + bii * pow(q, 2) + cii * q + dii);
            } 
            else if ((q > qinf) && (q < qsup)) {
                w = 1;
            } 
            else {
                w = 100000000000000000000000.0;
            }
            return w;
        }

        void reset_integration(Eigen::Vector<double, 4> q_current_arm){
            Eigen::Vector<double, 4> out_pinocchio;
            out_pinocchio = bravo.stonefish2pinocchio(q_current_arm);
            joint_whole_integration << 0.0, 0.0, 0.0, 0.0, out_pinocchio(0), out_pinocchio(1), out_pinocchio(2), out_pinocchio(3);
            joint_pos_bravo_integration << out_pinocchio(0), out_pinocchio(1), out_pinocchio(2), out_pinocchio(3);
            start_integration_time = std::chrono::high_resolution_clock::now();
            last_integration_time = std::chrono::high_resolution_clock::now();
            start_bravo_integration_time = std::chrono::high_resolution_clock::now();
            last_bravo_integration_time = std::chrono::high_resolution_clock::now();
            last_filter = std::chrono::high_resolution_clock::now();
            vel_pre_x = 0.0; vel_pre_y = 0.0; vel_pre_z = 0.0; vel_pre_yaw = 0.0;
        }

        void motion_integration(double sampling_time){
            std::chrono::duration<double> elapsed_integration= std::chrono::high_resolution_clock::now()-last_integration_time;
            if (elapsed_integration.count() > sampling_time) {
                joint_vel_integration = weighted_jointLimits_robust_mobile_axis_ik_diff_xyz(joint_whole_integration, primary_vel, secondary_vel, penaltyBaseX, penaltyBaseY, penaltyBaseZ, penaltyBaseYaw);
                finish_integration_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = finish_integration_time - start_integration_time;
                joint_whole_integration = joint_whole_integration + joint_vel_integration*elapsed.count();
                start_integration_time = std::chrono::high_resolution_clock::now();
                last_integration_time = std::chrono::high_resolution_clock::now();
            }
        }

        void bravo_motion_integration(double sampling_time){
            std::chrono::duration<double> elapsed_integration= std::chrono::high_resolution_clock::now()-last_bravo_integration_time;
            if (elapsed_integration.count() > sampling_time) {
                joint_vel_bravo_integration = manipulator_ik_diff(joint_pos_bravo_integration, primary_vel);
                finish_bravo_integration_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = finish_bravo_integration_time - start_bravo_integration_time;
                joint_pos_bravo_integration = joint_pos_bravo_integration + joint_vel_bravo_integration*elapsed.count();
                start_bravo_integration_time = std::chrono::high_resolution_clock::now();
                last_bravo_integration_time = std::chrono::high_resolution_clock::now();
            }
        }
};

enum CP_states{MANUAL_CONTROL, INTERACTION_INITIAL, INTERACTION, INTERACTION_NO_FEEDBACK, INTERACTION_REACTION, MANIPULATOR_CRITICAL, NULL_SPACE};

class cathode_protection_demo : public rclcpp::Node{
    public:
        CP_states state, prev_state;
        uvm_sim_stonefish uvm;
        interaction_control force_control;
        Eigen::Vector<double, 6> current_wrench_ee;
        Eigen::Vector<double, 6> desired_effort, desired_twist, base_twist_interaction;
        Eigen::Vector<double, 4> arm_interaction_initial, current_joint_position; 
        double  teleop_Vel_eeX =0.0;   
        double  teleop_Vel_eeY =0.0;
        double  teleop_Vel_eeZ =0.0;
        bool SET_INITIAL = true;
        
        //& PUBLISHERS AND SUBSCRIBERS
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr              subJoystick;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr          subFalconTwist;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       subJointFdb;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr  subFT;
        
        rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr                             subDVL;
        rclcpp::Subscription<stonefish_ros2::msg::INS>::SharedPtr                             subINS;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                              subOdometryTool;
        rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr       subGyro;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr                        pubFalconThruster;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                                  pubForceZ;
    
    public:

        cathode_protection_demo(const std::string urdf_filename_uvm, const std::string urdf_filename_manipulator, const std::string toolLink) : rclcpp::Node("CP_node"), uvm(urdf_filename_uvm, urdf_filename_manipulator, toolLink){
            state      = CP_states::MANUAL_CONTROL;
            prev_state = CP_states::MANUAL_CONTROL;
            desired_effort          << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //x, y, z, ang_x, ang_y, ang_z
            desired_twist           << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            arm_interaction_initial << 3.201, 2.474, 1.903, 0.057; 

            subJoystick    = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&cathode_protection_demo::airbus_joy_callback, this, std::placeholders::_1));
            subFalconTwist = this->create_subscription<geometry_msgs::msg::Twist>("/airbus/falcon/twist", 10, std::bind(&cathode_protection_demo::twist_base_callback, this, std::placeholders::_1));
            
            // BRAVO
            subJointFdb     = this->create_subscription<sensor_msgs::msg::JointState>("/bluevolta/servos/joint_states", 10, std::bind(&cathode_protection_demo::jointFdb_callback, this, std::placeholders::_1));
            subFT           = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/bravo/FT", 10, std::bind(&cathode_protection_demo::FT_callback, this, std::placeholders::_1));
            subOdometryTool = this->create_subscription<nav_msgs::msg::Odometry>("/bluevolta/navigator/odometry", 10, std::bind(&cathode_protection_demo::Odometry_Tool_callback, this, std::placeholders::_1));

            // FALCON
            subDVL         = this->create_subscription<stonefish_ros2::msg::DVL>("/bluevolta/dvl", 10, std::bind(&cathode_protection_demo::DVL_callback, this, std::placeholders::_1) );
            subGyro        = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/bluevolta/gyro", 10, std::bind(&cathode_protection_demo::gyro_callback, this, std::placeholders::_1) );
            subINS         = this->create_subscription<stonefish_ros2::msg::INS>("/bluevolta/INS", 10, std::bind(&cathode_protection_demo::INS_callback, this, std::placeholders::_1) );
        }
        
        //& FINITE MACHINE STATE AND MAIN LOOP FOR CP DEMO
        void program_loop(){
            while (rclcpp::ok()) {
                switch (state){
                case CP_states::MANUAL_CONTROL:
                    RCLCPP_INFO_ONCE(this->get_logger(), "Manual Control Mode");
                    manual_control();
                    break;
                case CP_states::INTERACTION_INITIAL:
                    RCLCPP_INFO_ONCE(this->get_logger(), "Interaction Initial State");
                    set_arm_ready_for_interaction();
                    break;
                case CP_states::INTERACTION:
                    RCLCPP_INFO_ONCE(this->get_logger(), "Interaction State");
                    contact_maintenance_base_control_current();
                    break; 
                case CP_states::INTERACTION_REACTION:
                    RCLCPP_INFO_ONCE(this->get_logger(), "Interaction with reaction");
                    contact_maintenance_base_control();
                    break; 
                case CP_states::INTERACTION_NO_FEEDBACK:
                    RCLCPP_INFO_ONCE(this->get_logger(), "Interaction NO Feedback State");
                    contact_maintenance();
                    break;
                case CP_states::NULL_SPACE:
                    RCLCPP_INFO_ONCE(this->get_logger(), "Null-Space State");
                    null_space_testing();
                    break;  
                case CP_states::MANIPULATOR_CRITICAL:
                    RCLCPP_WARN_ONCE(this->get_logger(), "Manipulator Critical");
                    state = CP_states::INTERACTION_INITIAL;
                    SET_INITIAL = true;
                    break;         
                default:
                    RCLCPP_ERROR_ONCE(this->get_logger(), "Error in CP main loop");
                    //cout << "Error in CP main loop";
                    break;
                }
            }
        }

        void manual_control(){
            prev_state = CP_states::MANUAL_CONTROL; 
            uvm.bluevolta.P_gain = 2; 
            uvm.bluevolta.P_gain_yaw =1;
            uvm.bluevolta.twist_control(desired_twist(0), desired_twist(1), desired_twist(2), desired_twist(5));
        }

        void set_arm_ready_for_interaction(){  
            prev_state = CP_states::INTERACTION_INITIAL;       
            std::chrono::high_resolution_clock::time_point last_interpolation_time = std::chrono::high_resolution_clock::now();
            uvm.bravo.joint_cmd = uvm.bravo.current_joint_position;  

            //& GOING TO DESIRED CONFIGURATION 
            while(not uvm.bravo.is_in_desired_configuration(0.01, arm_interaction_initial) && (SET_INITIAL==true)){                
                std::chrono::duration<double> elapsed= std::chrono::high_resolution_clock::now()-last_interpolation_time;
                uvm.bravo.going2joint_pos(arm_interaction_initial, 0.1, 0.0002);
                last_interpolation_time = std::chrono::high_resolution_clock::now();
                Eigen::Vector<double, 4> error; 
                error = arm_interaction_initial - uvm.bravo.current_joint_position;
                RCLCPP_INFO_ONCE(this->get_logger(), "Not yet in Desired Configuration");
                uvm.bluevolta.P_gain = 10; 
                uvm.bluevolta.P_gain_yaw =3;
                base_twist_interaction = 0.1 * desired_twist;
                uvm.bluevolta.twist_control(base_twist_interaction(0), base_twist_interaction(1), base_twist_interaction(2), base_twist_interaction(5));
                uvm.reset_integration(uvm.bravo.current_joint_position); 
            }

            //& MOTION IN X AND Y EE
            SET_INITIAL = false;
            uvm.bluevolta.P_gain = 10; 
            uvm.bluevolta.P_gain_yaw =3;
            base_twist_interaction = 0.1 * desired_twist;            
            uvm.primary_vel << teleop_Vel_eeX, teleop_Vel_eeY, 0.0, 0.0, 0.0, 0.0;
            uvm.bravo_motion_integration(0.001);
            uvm.bluevolta.twist_control(base_twist_interaction(0), base_twist_interaction(1), base_twist_interaction(2), base_twist_interaction(5));
            Eigen::Vector<double, 4> joint_pos_cmd_stonefish;
            joint_pos_cmd_stonefish = uvm.bravo.pinocchio2stonefish(uvm.joint_pos_bravo_integration);
            uvm.bravo.set_joint_pos(joint_pos_cmd_stonefish);
        }

        void contact_maintenance(){
            prev_state = CP_states::INTERACTION_NO_FEEDBACK;
            uvm.bluevolta.P_gain = 10; 
            uvm.bluevolta.P_gain_yaw =3;
            base_twist_interaction = 0.1 * desired_twist;
            double velZ; 
           
            //& CONTACT LOSS
            // The arm moves in z to gain contact
            if (abs(uvm.bravo.current_wrench_ee(2)) < 3.0){
                velZ = 0.02;
                force_control.velPreZ =0.0;
            }
            //& CONTACT MAINTENANCE
            else{
                force_control.Z_md_admittance_force_control_loop(0.01, 25.0, uvm.bravo.current_wrench_ee(2)); //0.02
                //velZ = VAL_SAT<double>(force_control.admittance_vel, 0.1, -0.1);
                velZ = force_control.admittance_vel;
            }
            //& WHOLE-BODY MOTION CONTROL GIVEN PRIMARY AND SECONDARY TASKS
            uvm.primary_vel << 0.0, 0.0, velZ, 0.0, 0.0, 0.0;
            uvm.secondary_vel << base_twist_interaction(0), base_twist_interaction(1), base_twist_interaction(2), base_twist_interaction(5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            uvm.motion_integration(0.001);
            uvm.bluevolta.twist_control(uvm.joint_vel_integration(0), uvm.joint_vel_integration(1), uvm.joint_vel_integration(2), uvm.joint_vel_integration(3));
            Eigen::Vector<double, 4> joint_pos_cmd_stonefish, joint_pos_cmd_pinocchio;
            joint_pos_cmd_pinocchio <<  uvm.joint_whole_integration(4), uvm.joint_whole_integration(5), uvm.joint_whole_integration(6), uvm.joint_whole_integration(7);
            joint_pos_cmd_stonefish = uvm.bravo.pinocchio2stonefish(joint_pos_cmd_pinocchio);
            uvm.bravo.set_joint_pos(joint_pos_cmd_stonefish);
            
            //& CHECK MANIPULABILITY
            Matrix<double, 3,4> localJacobian;
            Eigen::Vector<double,4> q56;
            q56 << joint_pos_cmd_pinocchio(0), joint_pos_cmd_pinocchio(1), joint_pos_cmd_pinocchio(2), joint_pos_cmd_pinocchio(3);
            localJacobian = uvm.local_jacobian_manipulator_xyz(q56);
            double cond = uvm.cond_arm(localJacobian);
            if (abs(cond) > 25){
                RCLCPP_WARN(this->get_logger(), "Condition number to high, switching to manipulator critical");
                state = CP_states::MANIPULATOR_CRITICAL; 
            }          
        }

        void contact_maintenance_base_control(){
            prev_state = CP_states::INTERACTION_REACTION;
            uvm.bluevolta.P_gain = 10; 
            uvm.bluevolta.P_gain_yaw =3;
            base_twist_interaction = 0.1 * desired_twist;
            double velZ; 

            //& USING ARM TO MEASURED DRIFT
            Eigen::Vector<double, 4> joint_pos_fdb_pinocchio, joint_pos_initial_pinocchio;
            joint_pos_initial_pinocchio = uvm.bravo.stonefish2pinocchio(arm_interaction_initial);
            joint_pos_fdb_pinocchio = uvm.bravo.stonefish2pinocchio(uvm.bravo.current_joint_position);
            Eigen::Vector<double,5> q;
            Eigen::Vector<double,5> q_init;
            q <<  0.0, joint_pos_fdb_pinocchio(0), joint_pos_fdb_pinocchio(1), joint_pos_fdb_pinocchio(2), joint_pos_fdb_pinocchio(3);
            q_init <<  0.0, joint_pos_initial_pinocchio(0), joint_pos_initial_pinocchio(1), joint_pos_initial_pinocchio(2), joint_pos_initial_pinocchio(3);
            forwardKinematics(uvm.model_uvm, uvm.data_uvm, q);
            Eigen::Vector3d current_pos_ee = uvm.data_uvm.oMi[4].translation();
            forwardKinematics(uvm.model_uvm, uvm.data_uvm, q_init);
            Eigen::Vector3d init_pos_ee = uvm.data_uvm.oMi[4].translation();
            Eigen::Vector<double, 3> pos_error = init_pos_ee - current_pos_ee;
            //RCLCPP_INFO(this->get_logger(), "Position Error 0: %f; 1: %f; 2: %f; 3", pos_error(0), pos_error(1), pos_error(2));
            Eigen::Vector<double, 3> vel_base_placement; 
            vel_base_placement << 0.0, 0.0, 0.0; 
            float Kp_base = 1; 

            //& CONTACT LOSS
            if (abs(uvm.bravo.current_wrench_ee(2)) < 3.0){
                velZ = 0.02;
                force_control.velPreZ =0.0;
            }
            //& CONTACT MAINTENANCE
            else{
                force_control.Z_md_admittance_force_control_loop(0.02, 25.0, uvm.bravo.current_wrench_ee(2));
                //velZ = VAL_SAT<double>(force_control.admittance_vel, 0.1, -0.1);
                velZ = force_control.admittance_vel;
                vel_base_placement = Kp_base*pos_error;
            }

            //& WHOLE-BODY MOTION CONTROL GIVEN PRIMARY AND SECONDARY TASKS
            uvm.primary_vel << 0.0, 0.0, velZ, 0.0, 0.0, 0.0;
            Eigen::Vector<double, 4> filtered_out_vel_base;
            filtered_out_vel_base = uvm.filter_base(base_twist_interaction(0)-vel_base_placement(0), base_twist_interaction(1), base_twist_interaction(2), base_twist_interaction(5));
            uvm.secondary_vel << filtered_out_vel_base(0), filtered_out_vel_base(1), filtered_out_vel_base(2), filtered_out_vel_base(3), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            uvm.motion_integration(0.001);
            uvm.bluevolta.twist_control(uvm.joint_vel_integration(0), uvm.joint_vel_integration(1), uvm.joint_vel_integration(2), uvm.joint_vel_integration(3));
            Eigen::Vector<double, 4> joint_pos_cmd_stonefish, joint_pos_cmd_pinocchio;
            joint_pos_cmd_pinocchio <<  uvm.joint_whole_integration(4), uvm.joint_whole_integration(5), uvm.joint_whole_integration(6), uvm.joint_whole_integration(7);
            joint_pos_cmd_stonefish = uvm.bravo.pinocchio2stonefish(joint_pos_cmd_pinocchio);
            uvm.bravo.set_joint_pos(joint_pos_cmd_stonefish);
        
            //& CHECK MANIPULABILITY
            Matrix<double, 3,4> localJacobian;
            Eigen::Vector<double,4> q56;
            q56 << joint_pos_cmd_pinocchio(0), joint_pos_cmd_pinocchio(1), joint_pos_cmd_pinocchio(2), joint_pos_cmd_pinocchio(3);
            localJacobian = uvm.local_jacobian_manipulator_xyz(q56);
            double cond = uvm.cond_arm(localJacobian);
            //RCLCPP_INFO(this->get_logger(), "CONDITION NUMBER: %f", cond);
            if (abs(cond) > 25){
                RCLCPP_WARN(this->get_logger(), "Condition number to high, switching to manipulator critical");
                state = CP_states::MANIPULATOR_CRITICAL; 
            }         
        }

        void contact_maintenance_base_control_current(){
            prev_state = CP_states::INTERACTION;
            uvm.bluevolta.P_gain = 10; 
            uvm.bluevolta.P_gain_yaw =3;
            base_twist_interaction = 0.1 * desired_twist;
            double velZ; 

            //& USING ARM TO MEASURED DRIFT
            Eigen::Vector<double, 4> joint_pos_fdb_pinocchio, joint_pos_initial_pinocchio;
            joint_pos_initial_pinocchio = uvm.bravo.stonefish2pinocchio(arm_interaction_initial);
            joint_pos_fdb_pinocchio = uvm.bravo.stonefish2pinocchio(uvm.bravo.current_joint_position);
            Eigen::Vector<double,7> q;
            Eigen::Vector<double,7> q_init;
            q <<  0.0, joint_pos_fdb_pinocchio(0), joint_pos_fdb_pinocchio(1), joint_pos_fdb_pinocchio(2), joint_pos_fdb_pinocchio(3);
            q_init <<  0.0, joint_pos_initial_pinocchio(0), joint_pos_initial_pinocchio(1), joint_pos_initial_pinocchio(2), joint_pos_initial_pinocchio(3);
            forwardKinematics(uvm.model_uvm, uvm.data_uvm, q);
            Eigen::Vector3d current_pos_ee = uvm.data_uvm.oMi[4].translation();
            forwardKinematics(uvm.model_uvm, uvm.data_uvm, q_init);
            Eigen::Vector3d init_pos_ee = uvm.data_uvm.oMi[4].translation();
            Eigen::Vector<double, 3> pos_error = init_pos_ee - current_pos_ee;
            //RCLCPP_INFO(this->get_logger(), "Position Error 0: %f; 1: %f; 2: %f; 3", pos_error(0), pos_error(1), pos_error(2));
            Eigen::Vector<double, 3> vel_base_placement; 
            vel_base_placement << 0.0, 0.0, 0.0; 
            float Kp_base = 1.5; 

            //& CONTACT LOSS
            if (abs(uvm.bravo.current_wrench_ee(2)) < 3.0){
                velZ = 0.02;
                force_control.velPreZ =0.0;
            }
            //& CONTACT MAINTENANCE
            else{
                force_control.Z_md_admittance_force_control_loop(0.02, 25.0, uvm.bravo.current_wrench_ee(2));
                //velZ = VAL_SAT<double>(force_control.admittance_vel, 0.1, -0.1);
                velZ = force_control.admittance_vel;
                vel_base_placement = Kp_base*pos_error;
            }

            //& WHOLE-BODY MOTION CONTROL GIVEN PRIMARY AND SECONDARY TASKS
            Eigen::Vector<double, 3> vel_base_placement_sat;
            vel_base_placement_sat(0) = VAL_SAT<double>(vel_base_placement(0), 0.04, -0.04);
            vel_base_placement_sat(1) = VAL_SAT<double>(vel_base_placement(1), 0.04, -0.04);
            vel_base_placement_sat(2) = VAL_SAT<double>(vel_base_placement(2), 0.04, -0.04);
            double vel_y_compensation = uvm.secondary_vel(0)-uvm.bluevolta.dvl_velocity(1);
            //vel_x_compensation = 0.0;
            uvm.primary_vel << 0.0, vel_y_compensation, velZ, 0.0, 0.0, 0.0;
            Eigen::Vector<double, 4> filtered_out_vel_base;
            filtered_out_vel_base = uvm.filter_base(base_twist_interaction(0)-vel_base_placement(0), base_twist_interaction(1)-vel_base_placement(1), base_twist_interaction(2), base_twist_interaction(5));
            uvm.secondary_vel << filtered_out_vel_base(0), filtered_out_vel_base(1), filtered_out_vel_base(2), filtered_out_vel_base(3), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            uvm.motion_integration(0.001);
            uvm.bluevolta.twist_control(uvm.joint_vel_integration(0), uvm.joint_vel_integration(1), uvm.joint_vel_integration(2), uvm.joint_vel_integration(3));
            Eigen::Vector<double, 4> joint_pos_cmd_stonefish, joint_pos_cmd_pinocchio;
            joint_pos_cmd_pinocchio <<  uvm.joint_whole_integration(4), uvm.joint_whole_integration(5), uvm.joint_whole_integration(6), uvm.joint_whole_integration(7);
            joint_pos_cmd_stonefish = uvm.bravo.pinocchio2stonefish(joint_pos_cmd_pinocchio);
            uvm.bravo.set_joint_pos(joint_pos_cmd_stonefish);
        
            //& CHECK MANIPULABILITY
            Matrix<double, 3,4> localJacobian;
            Eigen::Vector<double,4> q56;
            q56 << joint_pos_cmd_pinocchio(0), joint_pos_cmd_pinocchio(1), joint_pos_cmd_pinocchio(2), joint_pos_cmd_pinocchio(3);
            localJacobian = uvm.local_jacobian_manipulator_xyz(q56);
            double cond = uvm.cond_arm(localJacobian);
            //RCLCPP_INFO(this->get_logger(), "CONDITION NUMBER: %f", cond);
            if (abs(cond) > 25){
                RCLCPP_WARN(this->get_logger(), "Condition number to high, switching to manipulator critical");
                state = CP_states::MANIPULATOR_CRITICAL; 
            }         
        }


        void null_space_testing(){
            prev_state = CP_states::NULL_SPACE;
            uvm.bluevolta.P_gain = 10; 
            uvm.bluevolta.P_gain_yaw =3;
            base_twist_interaction = 0.1 * desired_twist;

            //& WHOLE-BODY MOTION CONTROL GIVEN PRIMARY AND SECONDARY TASKS
            uvm.primary_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            uvm.secondary_vel << base_twist_interaction(0), base_twist_interaction(1), base_twist_interaction(2), base_twist_interaction(5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            uvm.motion_integration(0.001);
            uvm.bluevolta.twist_control(uvm.joint_vel_integration(0), uvm.joint_vel_integration(1), uvm.joint_vel_integration(2), 3*uvm.joint_vel_integration(3));
            RCLCPP_INFO(this->get_logger(), "Cmd: %f", base_twist_interaction(5));
            RCLCPP_INFO(this->get_logger(), "Fdb: %f", -uvm.bluevolta.gyro_angular(2));
            Eigen::Vector<double, 4> joint_pos_cmd_stonefish, joint_pos_cmd_pinocchio;
            joint_pos_cmd_pinocchio <<  uvm.joint_whole_integration(4), uvm.joint_whole_integration(5), uvm.joint_whole_integration(6), uvm.joint_whole_integration(7);
            joint_pos_cmd_stonefish = uvm.bravo.pinocchio2stonefish(joint_pos_cmd_pinocchio);            
            uvm.bravo.set_joint_pos(joint_pos_cmd_stonefish);

            //& CHECK MANIPULABILITY
            Matrix<double, 4,8> localJacobian;
            Eigen::Vector<double,10> q56;
            q56 << 0.0, 0.0, 0.0, 0.0, joint_pos_cmd_pinocchio(0), joint_pos_cmd_pinocchio(1), joint_pos_cmd_pinocchio(2), joint_pos_cmd_pinocchio(3);
            localJacobian = uvm.local_jacobian_whole_vehicle(q56);
            double cond = uvm.cond_arm(localJacobian);
            if (abs(cond) > 3.4){
                RCLCPP_WARN(this->get_logger(), "Condition number to high, switching to manipulator critical");
                state = CP_states::MANIPULATOR_CRITICAL; 
            }          
        }

        void FT_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg){
            uvm.bravo.current_wrench_ee << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;            
        }

        void jointFdb_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
            uvm.bravo.current_joint_position << msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5];            
        }
        
        void twist_base_callback(const geometry_msgs::msg::Twist::SharedPtr msg){ 
            desired_twist << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z;
        }

        void airbus_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
            //&  CHANGE STATE
            if (msg->buttons[1] && (prev_state ==CP_states::INTERACTION_INITIAL)){                
                uvm.reset_integration(uvm.bravo.current_joint_position); 
                state = CP_states::INTERACTION;
                RCLCPP_INFO(this->get_logger(), "Set to Interaction");
            }    

            if ((msg->buttons[2]) && (abs(desired_twist(0))< 0.1) && (abs(desired_twist(1))< 0.1) && (abs(desired_twist(2))< 0.1) && (abs(desired_twist(5))< 0.1)){
                SET_INITIAL = true;
                uvm.reset_integration(uvm.bravo.current_joint_position); 
                state = CP_states::INTERACTION_INITIAL;
                RCLCPP_INFO(this->get_logger(), "Set to Initial Interaction");
            }    

            if (msg->buttons[3]){
                state = CP_states::MANUAL_CONTROL;
                RCLCPP_INFO(this->get_logger(), "Set to Manual Control");
            }       

            if (msg->buttons[7] && (prev_state ==CP_states::INTERACTION_INITIAL)){
                uvm.reset_integration(uvm.bravo.current_joint_position); 
                state = CP_states::NULL_SPACE;
                RCLCPP_INFO(this->get_logger(), "NULL_SPACE STATE");
            }      

            if (msg->buttons[8] && (prev_state ==CP_states::INTERACTION_INITIAL)){
                uvm.reset_integration(uvm.bravo.current_joint_position); 
                state = CP_states::INTERACTION_NO_FEEDBACK;
                RCLCPP_INFO(this->get_logger(), "Set to Interaction without feedback");
            }  

            if (msg->buttons[9] && (prev_state ==CP_states::INTERACTION_INITIAL)){
                uvm.reset_integration(uvm.bravo.current_joint_position); 
                state = CP_states::INTERACTION_REACTION;
                RCLCPP_INFO(this->get_logger(), "Set to Interaction with reaction");
            }  
            
            //&  MOTION CMD IN EE X AND Y
            double enableBaseMotion = (bool)(msg->buttons[0]);
            double vel =0.05;
            if (enableBaseMotion){
                teleop_Vel_eeX = msg->axes[5]*vel;  
                teleop_Vel_eeY = msg->axes[4]*vel; 
            }     
            else{ 
                teleop_Vel_eeX =0.0;   
                teleop_Vel_eeY =0.0;   
            }  
        }

        void DVL_callback(const stonefish_ros2::msg::DVL::SharedPtr msg){
            uvm.bluevolta.dvl_velocity << -msg->velocity.z, msg->velocity.x, msg->velocity.y;             
        }
        void Odometry_Tool_callback(const nav_msgs::msg::Odometry::SharedPtr msg){

        }

        void INS_callback(const stonefish_ros2::msg::INS::SharedPtr msg){
            uvm.bluevolta.orientation << msg->pose.roll, msg->pose.pitch, msg->pose.yaw;            
        }

        void gyro_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg){
            uvm.bluevolta.gyro_angular << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;            
        }

};

 
#endif 