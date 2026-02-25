/**
 *    @file  goToStore_joint_current.h
 *    @brief Goto store position in joint current control mode
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  2-Sep-2023
 *    Modification 2-Sep-2023
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
        bravo->set_control_mode(set_bravo_control_mode::current_control);

        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
    }        
    Eigen::VectorXd current_cmd(6), joint_position_cmd(6), joint_postion_cmd_aux(6);
    joint_position_cmd = bravo->get_bravo_joint_states();
    current_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd pos_gains(6), pos_error(6); 
    pos_gains << 8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0; // Gains for position control
    Eigen::Vector<double, 6> STORE;
    STORE << 0.323, 2.957, 0.05, 3.387, 0.066, 0.2304;
    bool goal_achieved = false;
    bool ready = false; 
    std::chrono::high_resolution_clock::time_point last_call = std::chrono::high_resolution_clock::now();
    while(!goal_achieved){
        std::tie(goal_achieved, ready, joint_postion_cmd_aux) = bravo->go_to_JointPos(STORE, joint_position_cmd, 2.0, 0.2, last_call);
        if (ready){
            joint_position_cmd = joint_postion_cmd_aux;
        }
        pos_error = bravo->signedAngleDistance(joint_position_cmd, bravo->get_bravo_joint_states());
        current_cmd = pos_gains.array() * pos_error.array();

        std::cout << "Pos Error [0]: " << pos_error[0] <<" [1]: "<< pos_error[1] <<" [2]: "<< pos_error[2] << " [3]: " <<
                                          pos_error[3] <<" [4]: "<< pos_error[4] <<" [5]: "<< pos_error[5] <<std::endl;
        std::cout << "Current_cmd [0]: " << current_cmd[0] <<" [1]: "<< current_cmd[1] <<" [2]: "<< current_cmd[2] << " [3]: " <<
                                            current_cmd[3] <<" [4]: "<< current_cmd[4] <<" [5]: "<< current_cmd[5] <<std::endl;
        //current_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        bravo->cmdJointCurrent_SAT(current_cmd, 1000.0);
    }
    current_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    bravo->cmdJointCurrent_SAT(current_cmd, 1000.0);
    std::cout << "Goal position reached!" << std::endl;
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_noFT_pinocchio.urdf");
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



