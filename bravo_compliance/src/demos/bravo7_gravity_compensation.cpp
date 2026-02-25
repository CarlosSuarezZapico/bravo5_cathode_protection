/**
 *    @file  bravo7_gravity_compensation.cpp
 *    @brief Unittest for the Bravo7 arm. The program run a loop back and forth. 
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      16-May-2025
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

#include "bravo_compliance/bravo_cpp/bravo_handler/bravo7_handler_v2.h"

#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

void program_loop(std::shared_ptr<bravo7_handler<double>> bravo){     
    auto start_time = std::chrono::steady_clock::now();
    Eigen::Vector3d gravity_vector;
    gravity_vector << 0.0, 0.0, -9.81;              //! GRAVITY VECTOR FOR ARM MOUNTING POINT
    bravo->manipulator_kin_dyn.change_gravity_vector(gravity_vector);  
    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests
        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
    }    

    bravo->set_control_mode(set_bravo_control_mode::current_control);
    Eigen::VectorXd mA_joint_current_cmd(6), joint_position_fdb(6);
    mA_joint_current_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    joint_position_fdb << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    double MAX_CURRENT_mA = 2500.0;
  
    while (rclcpp::ok()) {
        joint_position_fdb = bravo->get_bravo_joint_states();
        Eigen::VectorXd tau_gravity = bravo->manipulator_kin_dyn.invDynamics(joint_position_fdb, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));
        mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
        // mA_joint_current_cmd =  (tau_gravity/(0.222*120.0))*1000.0;
        std::cout << "Torque Nm[0]: " << joint_position_fdb[0] <<" [1]: "<< joint_position_fdb[1] <<" [2]: "<< joint_position_fdb[2] << " [3]: " <<
        joint_position_fdb[3] <<" [4]: "<< joint_position_fdb[4] <<" [5]: "<< joint_position_fdb[5] <<std::endl;
        std::cout << "Current mA[0]: " << mA_joint_current_cmd[0] <<" [1]: "<< mA_joint_current_cmd[1] <<" [2]: "<< mA_joint_current_cmd[2] << " [3]: " <<
        mA_joint_current_cmd[3] <<" [4]: "<< mA_joint_current_cmd[4] <<" [5]: "<< mA_joint_current_cmd[5] <<std::endl;            
        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);            
        bravo->publish_bravo_joint_states();
    }
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        //const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_amir_2.urdf");
    const std::string urdf_filename = ament_index_cpp::get_package_share_directory("bpl_bravo_description"); + "/urdf/bravo_7_amir_2.urdf";
        const std::string tool_link = std::string("EE");   
        auto bravo            = std::make_shared<bravo7_handler<double>>(urdf_filename, tool_link);//(urdf_filename);
        auto executor         = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();        
        std::thread executor_thread([&executor]() {
                executor->spin();
        });
        program_loop(bravo);
        executor_thread.join();
        rclcpp::shutdown();
        return 0;
}



