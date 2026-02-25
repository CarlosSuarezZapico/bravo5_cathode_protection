/**
 *    @file  compliance_3D_stiffness_control.cpp
 *    @brief Unittest for the Bravo7 arm. The program run a loop back and forth. 
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      4-Aug-2025
 *    Modification 4-Aug-2025
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
#include <cmath>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

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
    Eigen::VectorXd current_cmd(6);
    Eigen::VectorXd joint_position_cmd(6);
    Eigen::VectorXd joint_torque_cmd(6);
    current_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    bool low_cond = false;
    Eigen::VectorXd pos_gains(6); 
    pos_gains << 3000.0, 4000.0, 4000.0, 2500.0, 2500.0, 2500.0; // Gains for position control
    Eigen::Vector3d ref_ee_pos, current_ee_pos;
    Eigen::Matrix3d ref_ee_rot, current_ee_rot;
    Eigen::Vector3d position_error_fixed, force_estimation, position_error_global;
    std::tie(ref_ee_pos, ref_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(ref_joint_pos);
    Eigen::Vector3d Kxyz(0.0, 0.0, 0.5); // Stiffness gains for 3D control

    while (rclcpp::ok()) {
        //& STIFFNESS CONTROL

        std::tie(low_cond, joint_torque_cmd) = bravo->pos_3D_stiffness_control(Kxyz, ref_joint_pos);
        if (low_cond){
            current_cmd =  1000 * (joint_torque_cmd.array() / torque_constants.array());
            //bravo->cmdJointCurrent_SAT(current_cmd, 1000.0);
        }
        else{
            std::cout << "Low manipulability condition, not sending torque command." << std::endl;
            joint_torque_cmd.setZero(); // Set to zero if low manipulability
            current_cmd.setZero(); // Set to zero if low manipulability
        }

        std::cout << "Current [0]: " << current_cmd[0] <<" [1]: "<< current_cmd[1] <<" [2]: "<< current_cmd[2] << " [3]: " <<
                        current_cmd[3] <<" [4]: "<< current_cmd[4] <<" [5]: "<< current_cmd[5] <<std::endl;
        bravo->cmdJointCurrent_SAT(current_cmd, 1500.0);

        //& USE JOINT TORQUE FEEDBACK TO ESTIMATE CONTACT FORCE & POSITION ERROR
        //*EE position error
        std::tie(current_ee_pos, current_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(bravo->get_bravo_joint_states());
        position_error_fixed = ref_ee_pos - current_ee_pos;
        std::cout << "Position Error Fixed [0]: " << position_error_fixed[0] <<" [1]: "<< position_error_fixed[1] <<" [2]: "<< position_error_fixed[2] << std::endl;
        position_error_global = ref_ee_rot * position_error_fixed;
        std::cout << "Position Error Global [0]: " << position_error_global[0] <<" [1]: "<< position_error_global[1] <<" [2]: "<< position_error_global[2] << std::endl;
        force_estimation = 1 * position_error_fixed;
        //* joint torque feedback
        Eigen::VectorXd torque_fdb = bravo->get_bravo_joint_torques();
        Eigen::MatrixXd jacobian = bravo->manipulator_kin_dyn.localJacobian(bravo->get_bravo_joint_states());
        Eigen::VectorXd wrench = jacobian * torque_fdb;
        bravo->publish_force_estimation(force_estimation);
        bravo->publish_wrench_estimation(wrench);
        bravo->publish_bravo_joint_torques();
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



