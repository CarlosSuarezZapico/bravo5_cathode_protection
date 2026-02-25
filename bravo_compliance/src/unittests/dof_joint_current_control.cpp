/**
 *    @file  joystick_bravo_real_force_control.h
 *    @brief Executable program to test remote control of bravo with joystic and force control when contact is 
 *    established
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

#include "bravo7_version_2/bravo_cpp/bravo_handler/bravo7_handler_v2.h"
#include "general_libs_unite/joysticks/airbus_joystick.h"
#include "general_libs_unite/interaction/interaction_control.h"
#include "general_libs_unite/general_utils/general_utils.h"

#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;



void teleop_bravo_sim(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo7_handler<double>> bravo){
    auto start_time = std::chrono::steady_clock::now();
    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests
        
        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
    }   
    int joint = 0; 
    bravo->set_control_mode(set_bravo_control_mode::current_control);
    double ref_joint_pos = bravo->get_bravo_joint_states()[joint];
    Eigen::VectorXd torque_cmd(6);
    torque_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    while (rclcpp::ok()) {
        //JOint 0 gain 4000
        //Joint 1 gain 4000
        //Joint 2 gain 4000
        //Joint 3 gain 2500
        //Joint 4 gain 2500
        
        torque_cmd[joint] = 4000*(ref_joint_pos - bravo->get_bravo_joint_states()[joint]);
        std::cout << "Current joint position: " << torque_cmd[joint] << std::endl;
        bravo->cmdJointCurrent_SAT(torque_cmd, 1500.0);
        bravo->publish_bravo_joint_states();
    }
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        //const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/example-robot-data/robots/bravo7_description/urdf/bravo7_sphere_endeffector.urdf");
        const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_pinocchio.urdf");
        const std::string tool_link = std::string("contact_point");
        auto joystick         = std::make_shared<airbus_joystick_bravo_twist_ee>();
        auto bravo            = std::make_shared<bravo7_handler<double>>(urdf_filename, tool_link);//(urdf_filename);
        auto executor         = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor->add_node(joystick);
        std::thread executor_thread([&executor]() {
                executor->spin();
        });
        teleop_bravo_sim(joystick, bravo);
        executor_thread.join();
        rclcpp::shutdown();

        return 0;
}



