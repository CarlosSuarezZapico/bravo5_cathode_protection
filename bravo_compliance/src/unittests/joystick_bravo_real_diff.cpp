/**
 *    @file  joystick_bravo_real_diff_velocity.cpp
 *    @brief Executable program to test diff kinematics in real bravo arm with airbus joystick
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  16-May-2023
 *    Modification 28-May-2023
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


#include "ament_index_cpp/get_package_share_directory.hpp"
#include "bravo_manipulator/bravo_cpp/bravo_handler/bravo7_handler.h"
#include "general_libs_unite/joysticks/airbus_joystick.h"
#include "general_libs_unite/general_utils/general_utils.h"

#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;

void teleop_bravo_sim(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo7_handler<double>> bravo){
    while (rclcpp::ok()) {
        // Your code for other tasks here
        //std::this_thread::sleep_for(1ms);
        Eigen::Vector<double, 6> twist_joy;
        float reduced_vel_factor = 1;
        twist_joy << 1*airbus_joy->teleop_VelX, 1*airbus_joy->teleop_VelY, 1*airbus_joy->teleop_VelZ, 0.1*airbus_joy->teleop_Wx, 0.1*airbus_joy->teleop_Wy, 0.1*airbus_joy->teleop_Wz;
        //RCLCPP_INFO(this->get_logger(), "Twist Cmd %f, %f, %f, %f, %f, %f ", twist_joy[0], twist_joy[1], twist_joy[2], twist_joy[3], twist_joy[4], twist_joy[5]);
        bravo->moveCmdTCPLocalTwist(reduced_vel_factor*twist_joy, 0.0005);
        bravo->publish_RVIZ_sim_bravo_joint_states(bravo->get_joint_whole_integration(), 0.0);
        bravo->publish_bravo_joint_states();
        bravo->publish_feedback_rates(); //Monitor the feedback frequency at which the bravo is responding
    }
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        //const std::string urdf_filename = ament_index_cpp::get_package_share_directory("robot_descriptions") + "/urdf/bravo7_sphere_endeffector.urdf";
        const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_pinocchio.urdf");
        //const std::string urdf_filename = "/home/carlos/cp_unite_ws/src/robot_descriptions/example-robot-data/robots/bravo7_description/urdf/bravo7_sphere_endeffector.urdf";
        const std::string tool_link = std::string("contact_point");
        auto joystick        = std::make_shared<airbus_joystick_bravo_twist_ee>();
        auto bravo           = std::make_shared<bravo7_handler<double>>(urdf_filename, tool_link);//(urdf_filename);
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



