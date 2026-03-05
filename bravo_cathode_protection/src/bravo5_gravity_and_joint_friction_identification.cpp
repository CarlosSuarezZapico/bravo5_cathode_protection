/**
 *    @file  bravo_gravity_and_joint_friction.cpp
 *    @brief Unittest for the Bravo5 arm. The program implements gravity compensation
 *           plus stiction compensation using a smooth Stribeck model.
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      25-Nov-2025
 *    Modification 25-Nov-2025
 *    State:       //& WORKING
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
#include "bravo_cathode_protection/bravo_cpp/bravo_handler/bravo_handler_v2.h"
#include <string>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

void program_loop(std::shared_ptr<bravo_handler<double>> bravo){     
    Eigen::Matrix<double,4,6> FRICTION_MAT;
    FRICTION_MAT << 0.0, 0.0, 0.0, 14.66651, 10.09561,  1.25110,  // Joint 1
                    0.0, 0.0, 0.0, 14.50959, 10.09275,  1.14381,  // Joint 2
                    0.0, 0.0, 0.0, 13.98409, 12.97659, 21.69165,  // Joint 3
                    0.0, 0.0, 0.0, 15.82785, 14.35639, 24.04595;  // Joint 4
    const double FRICTION_SATURATION = 60; 
    auto start_time = std::chrono::steady_clock::now();
    Eigen::Vector3d gravity_vector;
    gravity_vector << 0.0, 0.0, 9.81;              //! GRAVITY VECTOR FOR ARM MOUNTING POINT
    bravo->kinodynamics.change_gravity_vector(gravity_vector); 
    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests
        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; 
        }
    }    
    bravo->set_control_mode(set_bravo_control_mode::current_control);
    //& DEFINE PARAMETERS
    Eigen::VectorXd mA_joint_current_cmd(4), Nm_friction_compensation(4), mA_friction_compensation(4);
    mA_joint_current_cmd = Eigen::VectorXd::Zero(4);
    double MAX_CURRENT_mA = 1500.0;
    
    while (rclcpp::ok()) {
        //& PLOT MANIPULABILITY
        std::cout << "Manipulablity: "<< bravo->compute_manipulability_position() << std::endl;
        Eigen::VectorXd joint_velocity_friction = bravo->get_bravo_joint_velocities();
        //& FRICTION COMPENSATION
        Nm_friction_compensation[0] = bravo->kinodynamics.computeFriction(joint_velocity_friction[0], FRICTION_MAT(0,0), FRICTION_MAT(0,1), FRICTION_MAT(0,2), FRICTION_MAT(0,3), FRICTION_MAT(0,4), FRICTION_MAT(0,5), FRICTION_SATURATION);
        Nm_friction_compensation[1] = bravo->kinodynamics.computeFriction(joint_velocity_friction[1], FRICTION_MAT(1,0), FRICTION_MAT(1,1), FRICTION_MAT(1,2), FRICTION_MAT(1,3), FRICTION_MAT(1,4), FRICTION_MAT(1,5), FRICTION_SATURATION);
        Nm_friction_compensation[2] = bravo->kinodynamics.computeFriction(joint_velocity_friction[2], FRICTION_MAT(2,0), FRICTION_MAT(2,1), FRICTION_MAT(2,2), FRICTION_MAT(2,3), FRICTION_MAT(2,4), FRICTION_MAT(2,5), FRICTION_SATURATION);
        Nm_friction_compensation[3] = bravo->kinodynamics.computeFriction(joint_velocity_friction[3], FRICTION_MAT(3,0), FRICTION_MAT(3,1), FRICTION_MAT(3,2), FRICTION_MAT(3,3), FRICTION_MAT(3,4), FRICTION_MAT(3,5), FRICTION_SATURATION);
        mA_friction_compensation = bravo->torqueNm_2_currentmA(Nm_friction_compensation);
        //& GRAVITY COMPENSATION
        Eigen::VectorXd tau_gravity = bravo->kinodynamics.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(4), Eigen::VectorXd::Zero(4));
        mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
        mA_joint_current_cmd = mA_joint_current_cmd.array() + mA_friction_compensation.array(); // Add stiction compensation
        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);
    }
}
//&  MAIN 
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);       
    const std::string urdf_filename = ament_index_cpp::get_package_share_directory("bravo_cathode_protection") + "/urdf/bravo_5_dynamics_pinocchio_cp.urdf";
    const std::string tool_link = std::string("contact_point");
    const std::string ip_address = std::string("10.43.0.146");
    auto bravo            = std::make_shared<bravo_handler<double>>(urdf_filename, tool_link, bravo_control::ArmModel::bravo5, ip_address);
    auto executor         = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();        
    std::thread executor_thread([&executor]() {
            executor->spin();
    });
    program_loop(bravo);
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}


