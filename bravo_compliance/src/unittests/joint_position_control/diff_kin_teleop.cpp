/**
 *    @file  diff_kin_teleop.cpp
 *    @brief Unittest for the Bravo7 arm. The program teleops the arm with in 
 *    differential ik, some dimensions are in local or global axis depending on . 
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  16-May-2025
 *    Modification 12-Jun-2025
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

enum state { GO_HOME, CONTACT};

void program_loop(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo7_handler<double>> bravo){     
    auto start_time = std::chrono::steady_clock::now();
    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests
        bravo->set_control_mode(set_bravo_control_mode::position_control);

        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
    }    
    Eigen::Vector<double, 6> HOME;
    HOME  << 4.904, 2.592, 0.549, 4.904, 1.570, 0.0; 
    bravo->go_to_JointPos(HOME, 3.0);
    while (rclcpp::ok()) {
        if (bravo->compute_manipulability()>25){
            bravo->go_to_JointPos(HOME, 3.0);
        }
        else{
            //& JACOBIAN
            Eigen::Matrix<double, 6, 6> Jacobian;
            Eigen::Vector<bool, 6>  mask_local;
            mask_local << true, true, false, false, false, false;
            //! Wrong Jacobian = bravo->manipulator_kin_dyn.mixed_jacobian(bravo->get_bravo_joint_states(), mask_local);
            Jacobian = bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states());
            //& TWIST 
            Eigen::Vector<double, 6> twist_joy_fixed;
            float reduced_vel_factor = 0.9;
            twist_joy_fixed << airbus_joy->teleop_VelZ, -airbus_joy->teleop_VelX, -airbus_joy->teleop_VelY, airbus_joy->teleop_Wx, airbus_joy->teleop_Wy, airbus_joy->teleop_Wz;
            bravo->move_ee_twist(reduced_vel_factor*twist_joy_fixed, Jacobian, 0.05);
        }        
        bravo->publish_bravo_joint_states();
    }
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
        program_loop(joystick, bravo);
        executor_thread.join();
        rclcpp::shutdown();
        return 0;
}



