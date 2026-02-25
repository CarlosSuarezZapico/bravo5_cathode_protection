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
        std::this_thread::sleep_for(10ms);
        Eigen::Vector<double, 6> twist_joy;
        float reduced_vel_factor = 0.5;
        twist_joy << airbus_joy->teleop_VelX, airbus_joy->teleop_VelY, airbus_joy->teleop_VelZ, airbus_joy->teleop_Wx, airbus_joy->teleop_Wy, airbus_joy->teleop_Wz;
        bravo->sim_motion_integration(0.001, reduced_vel_factor*twist_joy);
        bravo->publish_RVIZ_sim_bravo_joint_states(bravo->get_sim_joint_whole_integration(), 0.0);
    }
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/example-robot-data/robots/bravo7_description/urdf/bravo7_sphere_endeffector.urdf");
        const std::string tool_link = std::string("contact_point");
        auto joystick        = std::make_shared<airbus_joystick_bravo_twist_ee>();
        auto bravo_simulator = std::make_shared<bravo7_handler<double>>(urdf_filename, tool_link, false);//(urdf_filename);
        auto executor        = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor->add_node(joystick);
        std::thread executor_thread([&executor]() {
                executor->spin();
        });
        teleop_bravo_sim(joystick, bravo_simulator);
        executor_thread.join();
        rclcpp::shutdown();

        return 0;
}



