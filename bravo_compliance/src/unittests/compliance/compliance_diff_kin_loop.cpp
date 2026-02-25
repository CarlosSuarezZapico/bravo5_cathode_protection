/**
 *    @file  diff_kin_loop.cpp
 *    @brief Unittest for the Bravo7 arm. The program run a loop back and forth. 
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

        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
    }    
    bravo->set_control_mode(set_bravo_control_mode::current_control);
    Eigen::VectorXd ref_joint_pos = bravo->get_bravo_joint_states();
    Eigen::VectorXd torque_cmd(6);
    Eigen::VectorXd joint_position_cmd(6);
    torque_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    bool high_cond = false;
    Eigen::VectorXd pos_gains(6), pos_gains_contact(6); 
    pos_gains_contact << 4000.0, 4000.0, 4000.0, 2500.0, 2500.0, 2500.0; //COMPLIANCE VALUES
    //pos_gains <<7000.0, 7000.0, 7000.0, 7000.0, 7000.0, 7000.0;
    pos_gains <<10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0;
    Eigen::Vector3d ref_ee_pos, current_ee_pos;
    Eigen::Matrix3d ref_ee_rot, current_ee_rot;
    std::tie(ref_ee_pos, ref_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(ref_joint_pos);
    Eigen::Vector3d position_error_fixed, force_estimation, position_error_global;
    force_estimation << 0.0, 0.0, 0.0;

    while (rclcpp::ok()) {
        if (bravo->compute_manipulability()>25){
            torque_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            bravo->cmdJointCurrent_SAT(torque_cmd, 2000.0);
            std::cout << "Manipulability is high, no command sent." << std::endl;
        }
        else{
            
            Eigen::Vector<double, 6> twist_joy, twist_fixed;
            double reduced_vel_factor = 0.02;
            twist_joy << reduced_vel_factor*airbus_joy->teleop_VelX, reduced_vel_factor*airbus_joy->teleop_VelY, reduced_vel_factor*airbus_joy->teleop_VelZ, 0.0, 0.0, 0.0;
            twist_fixed << 0.0, 0.0, 0.01, 0.0, 0.0, 0.0;
            std::cout << "Diff called" << std::endl;
            std::tie(high_cond, joint_position_cmd) = bravo->twist_diff_kin(twist_fixed, 0.02);
            std::cout << "Position [0]: " << twist_joy[0] <<" [1]: "<< twist_joy[1] <<" [2]: "<< twist_joy[2] << " [3]: " <<
                            twist_joy[3] <<" [4]: "<< twist_joy[4] <<" [5]: "<< twist_joy[5] <<std::endl;
            if (high_cond){
                torque_cmd = pos_gains_contact.array() * (bravo->signedAngleDistance(joint_position_cmd, bravo->get_bravo_joint_states())).array();
            }
            Eigen::VectorXd torque_fdb = bravo->get_bravo_joint_torques();
            Eigen::MatrixXd jacobian = bravo->manipulator_kin_dyn.localJacobian(bravo->get_bravo_joint_states());
            Eigen::VectorXd wrench = jacobian * torque_fdb;
            bravo->publish_wrench_estimation(wrench);
            //*EE position error
            std::tie(current_ee_pos, current_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(bravo->get_bravo_joint_states());
            std::tie(ref_ee_pos, ref_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(joint_position_cmd);
            position_error_fixed[0] = abs(ref_ee_pos[0] - current_ee_pos[0]);
            position_error_fixed[1] = abs(ref_ee_pos[1] - current_ee_pos[1]);
            position_error_fixed[2] = abs(ref_ee_pos[2] - current_ee_pos[2]);
            std::cout << "Position Error Fixed [0]: " << position_error_fixed[0] <<" [1]: "<< position_error_fixed[1] <<" [2]: "<< position_error_fixed[2] << std::endl;
            position_error_global = ref_ee_rot * position_error_fixed;
            bravo->publish_force_estimation(position_error_fixed);
            std::cout << "Position Error Global [0]: " << position_error_global[0] <<" [1]: "<< position_error_global[1] <<" [2]: "<< position_error_global[2] << std::endl;
            bravo->cmdJointCurrent_SAT(torque_cmd, 2000.0);
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



