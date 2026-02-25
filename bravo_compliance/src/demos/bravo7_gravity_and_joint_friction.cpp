/**
 *    @file  bravo7_gravity_and_joint_friction.cpp
 *    @brief Unittest for the Bravo7 arm. The program implements gravity compensation
 *           plus stiction compensation using a smooth Stribeck model.
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      25-Nov-2025
 *    Modification 25-Nov-2025
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
#include <string>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

void program_loop(std::shared_ptr<bravo7_handler<double>> bravo7){     
    auto start_time = std::chrono::steady_clock::now();
    Eigen::Vector3d gravity_vector;
    gravity_vector << 0.0, 0.0, -9.81;              //! GRAVITY VECTOR FOR ARM MOUNTING POINT
    bravo7->manipulator_kin_dyn.change_gravity_vector(gravity_vector); 
    while (!bravo7->isConnected()){
        bravo7->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests
        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
    }    

    bravo7->set_control_mode(set_bravo_control_mode::current_control);
    Eigen::VectorXd mA_joint_current_cmd(6), coulomb_custom(6), stiction_custom(6);
    coulomb_custom << 300.0, 200.0, 200.0, 200.0, 200.0, 200.0;
    stiction_custom << 350.0, 350.0, 350.0, 350.0, 350.0, 350.0;
    mA_joint_current_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    double MAX_CURRENT_mA = 2500.0;
  
    while (rclcpp::ok()) {
        Eigen::VectorXd mA_stiction_stribeck_custom = bravo7->manipulator_kin_dyn.compute_I_ff_stribeck_smooth(bravo7->get_bravo_joint_velocities(), coulomb_custom, stiction_custom, 0.045, 100.0, 0.03);
        Eigen::VectorXd tau_gravity                 = bravo7->manipulator_kin_dyn.invDynamics(bravo7->get_bravo_joint_states(), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));
        mA_joint_current_cmd = bravo7->torqueNm_2_currentmA(tau_gravity);
        mA_joint_current_cmd = mA_joint_current_cmd.array() + mA_stiction_stribeck_custom.array(); // Add stiction compensation
        bravo7->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);
    }
}
//&  MAIN 
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);       
    //const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_amir_2.urdf");
    const std::string urdf_filename = ament_index_cpp::get_package_share_directory("bpl_bravo_description"); + "/urdf/bravo_7_amir_2.urdf";
    const std::string tool_link = std::string("EE");   
    auto bravo7            = std::make_shared<bravo7_handler<double>>(urdf_filename, tool_link);//(urdf_filename);
    auto executor         = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();        
    std::thread executor_thread([&executor]() {
            executor->spin();
    });
    program_loop(bravo7);
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}



