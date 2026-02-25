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
    bravo->set_control_mode(set_bravo_control_mode::position_control);

    Eigen::VectorXd ref_joint_pos = bravo->get_bravo_joint_states();
    Eigen::VectorXd torque_cmd(6);
    Eigen::VectorXd joint_position_cmd(6);
    torque_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    bool high_cond = false;
    Eigen::VectorXd pos_gains(6); 
    pos_gains << 3000.0, 4000.0, 4000.0, 2500.0, 2500.0, 2500.0;
    Eigen::Vector3d ref_ee_pos, current_ee_pos;
    Eigen::Matrix3d ref_ee_rot, current_ee_rot;
    Eigen::Vector3d position_error;
    std::tie(ref_ee_pos, ref_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(ref_joint_pos);
    Eigen::Vector3d force_estimation; 
    force_estimation << 0.0, 0.0, 0.0;

    //& GO TO HOME POSITION
    Eigen::Vector<double, 6> HOME;
    HOME << 0.323, 2.708, 0.5045, 1.9917, 1.6112, 0.2304;
    bravo->go_to_JointPos(HOME, 3.0);
    bravo->set_control_mode(set_bravo_control_mode::current_control);
   


    while (rclcpp::ok()) {
        if (bravo->compute_manipulability()>26){
            torque_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            std::cout << "Manipulability is high, no command sent." <<  bravo->compute_manipulability() << std::endl;
            bravo->cmdJointCurrent_SAT(torque_cmd, 2000.0);
            bravo->set_control_mode(set_bravo_control_mode::position_control);
            bravo->go_to_JointPos(HOME, 3.0);
            bravo->set_control_mode(set_bravo_control_mode::current_control);
            
        }
        else{
            
            Eigen::Vector<double, 6> twist_joy;
            float reduced_vel_factor = 1;
            twist_joy << 0.0, 0.0, 0.005, 0.0, 0.0, 0.0;
            std::cout << "Diff called" << std::endl;
            std::tie(high_cond, joint_position_cmd) = bravo->twist_diff_kin(reduced_vel_factor*twist_joy, 0.2);
            std::cout << "Position [0]: " << joint_position_cmd[0] <<" [1]: "<< joint_position_cmd[1] <<" [2]: "<< joint_position_cmd[2] << " [3]: " <<
                            joint_position_cmd[3] <<" [4]: "<< joint_position_cmd[4] <<" [5]: "<< joint_position_cmd[5] <<std::endl;
            if (high_cond){
                torque_cmd = pos_gains.array() * (bravo->signedAngleDistance(joint_position_cmd, bravo->get_bravo_joint_states())).array();
            }
            Eigen::VectorXd torque_fdb = bravo->get_bravo_joint_torques();
            Eigen::MatrixXd jacobian = bravo->manipulator_kin_dyn.localJacobian(bravo->get_bravo_joint_states());
            Eigen::VectorXd wrench = jacobian * torque_fdb;
            bravo->publish_wrench_estimation(wrench);
            std::cout << "Torque [0]: " << torque_cmd[0] <<" [1]: "<< torque_cmd[1] <<" [2]: "<< torque_cmd[2] << " [3]: " <<
                                       torque_cmd[3] <<" [4]: "<< torque_cmd[4] <<" [5]: "<< torque_cmd[5] <<std::endl;
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



