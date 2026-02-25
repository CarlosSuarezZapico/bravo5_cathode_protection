/**
 *    @file  joystick_bravo_sim_diff.h
 *    @brief Executable program to text in simulation, the differential kinematics of bravo arm
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  16-May-2023
 *    Modification 16-May-2023
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp" 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "bravo_compliance/bravo_cpp/bravo_handler/bravo_handler_v2.h"
#include "general_libs_unite/joysticks/airbus_joystick.h"
#include "general_libs_unite/general_utils/general_utils.h"

#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

Eigen::Vector<double, 4> HOME = (Eigen::Vector<double, 4>() << 3.14, 1.70, 1.20, 0.0).finished();

void teleop_bravo_sim(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo_handler<double>> bravo){
    std::chrono::high_resolution_clock::time_point sim_last_integration_time   = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point sim_finish_integration_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point sim_start_integration_time  = std::chrono::high_resolution_clock::now();
    Eigen::Vector<double, 4> sim_joint_whole_integration = HOME;
    const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr & js_pub = bravo->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = rclcpp::Clock().now();   // use the node clock
    joint_state_msg.name.resize(5);
    joint_state_msg.name[0] = "joint1";
    joint_state_msg.name[1] = "joint2";
    joint_state_msg.name[2] = "joint3";
    joint_state_msg.name[3] = "joint4";
    joint_state_msg.name[4] = "gripper";
    joint_state_msg.position.resize(5);

    while (rclcpp::ok()) {
        // Your code for other tasks here
        std::this_thread::sleep_for(10ms);
        Eigen::Vector<double, 3> twist_joy;
        float reduced_vel_factor = 0.5;
        twist_joy << airbus_joy->teleop_VelX, airbus_joy->teleop_VelY, airbus_joy->teleop_VelZ;//, airbus_joy->teleop_Wx, airbus_joy->teleop_Wy, airbus_joy->teleop_Wz;
        
        twist_joy = reduced_vel_factor*twist_joy;

        std::chrono::duration<double> elapsed_integration= std::chrono::high_resolution_clock::now() - sim_last_integration_time;
        if (elapsed_integration.count() > 0.01) {
            Eigen::MatrixXd jacobian = bravo->kinodynamics.fixedJacobian(sim_joint_whole_integration);
            Eigen::MatrixXd jacobian3x4 = jacobian.topRows(3);
            Eigen::Vector<double, 4> sim_joint_vel_integration = jacobian3x4.colPivHouseholderQr().solve(twist_joy);
            //Eigen::Vector<double, 4> sim_joint_vel_integration = jacobian3x4.inverse()*twist_joy;
            sim_finish_integration_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = sim_finish_integration_time - sim_start_integration_time;
            sim_joint_whole_integration = sim_joint_whole_integration + sim_joint_vel_integration*elapsed.count();
            sim_start_integration_time = std::chrono::high_resolution_clock::now();
            sim_last_integration_time = std::chrono::high_resolution_clock::now();
            //std::cout << "Joint Vel: " << sim_joint_vel_integration.transpose() << std::endl;
            // ---- Publish JointState ----
            joint_state_msg.header.stamp = rclcpp::Clock().now();
            joint_state_msg.position[0] = sim_joint_whole_integration[0];
            joint_state_msg.position[1] = sim_joint_whole_integration[1];
            joint_state_msg.position[2] = sim_joint_whole_integration[2];
            joint_state_msg.position[3] = sim_joint_whole_integration[3];
            joint_state_msg.position[4] = 0.0; //gripper
            js_pub->publish(joint_state_msg);
        } 
    //bravo->publish_RVIZ_sim_bravo_joint_states(sim_joint_whole_integration, 0.0);
    }
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        const std::string urdf_filename = ament_index_cpp::get_package_share_directory("bpl_bravo_description") + "/urdf/bravo_5_dynamics_no_ee_pinocchio.urdf";
        const std::string tool_link = std::string("contact_point");  
        auto joystick        = std::make_shared<airbus_joystick_bravo_twist_ee>();
        auto bravo           = std::make_shared<bravo_handler<double>>(urdf_filename, tool_link, false);
        auto executor        = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor->add_node(joystick);
        std::thread executor_thread([&executor]() {
                executor->spin();
        });
        teleop_bravo_sim(joystick, bravo);
        executor_thread.join();
        rclcpp::shutdown();

        return 0;
}



