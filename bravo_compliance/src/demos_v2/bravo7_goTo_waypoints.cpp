/**
 *    @file  bravo7_goTo_waypoints.cpp
 *    @brief Unittest for the Bravo7 arm. The program goes through a series of 
 *    joint position waypoints in position control instruction (bravo instruction)
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      8-Jan-2025
 *    Modification 8-Jan-2025
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

Eigen::Vector<double, 6> WAYPOINT_1 = (Eigen::Vector<double, 6>() << 3.10, 2.50, 0.4, 3.1, 1.4, 0.0).finished();
Eigen::Vector<double, 6> WAYPOINT_2 = (Eigen::Vector<double, 6>() << 3.10, 2.50, 0.4, 3.1, 1.4, 0.0).finished();
Eigen::Vector<double, 6> WAYPOINT_3 = (Eigen::Vector<double, 6>() << 3.10, 2.50, 0.4, 3.1, 1.4, 0.0).finished();
Eigen::Vector<double, 6> WAYPOINT_4 = (Eigen::Vector<double, 6>() << 3.10, 2.50, 0.4, 3.1, 1.4, 0.0).finished();

void program_loop(std::shared_ptr<bravo_handler<double>> bravo){     
    auto start_time = std::chrono::steady_clock::now();
    Eigen::Vector3d gravity_vector;
    gravity_vector << 0.0, 0.0, -9.81;              //! GRAVITY VECTOR FOR ARM MOUNTING POINT
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
    Eigen::VectorXd mA_joint_current_cmd(6), coulomb_custom(6), stiction_custom(6), p_gain_current_cmd(6);
    coulomb_custom  << 300.0, 200.0, 200.0, 200.0, 200.0, 200.0;
    stiction_custom << 350.0, 350.0, 350.0, 350.0, 350.0, 350.0;
    mA_joint_current_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    p_gain_current_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    double MAX_CURRENT_mA = 1000.0;
    double P_GAIN = 10.0; 
    bool arrived = false;
    int waypoint_counter = 1; 

    bool ready = false; 
    Eigen::Vector<double, 6> desired_waypoint;
    desired_waypoint = WAYPOINT_1;
    Eigen::Vector<double, 6> cmdIntermediateJointPos; 
    cmdIntermediateJointPos = bravo->get_bravo_joint_states(); 
    std::chrono::high_resolution_clock::time_point& last_call
  
    while (rclcpp::ok()) {
        //& GRAVITY AND FRICTION COMPENSATION 
        Eigen::VectorXd mA_stiction_stribeck_custom = bravo->kinodynamics.compute_I_ff_stribeck_smooth(bravo->get_bravo_joint_velocities(), coulomb_custom, stiction_custom, 0.045, 100.0, 0.03);
        Eigen::VectorXd tau_gravity                 = bravo->kinodynamics.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));

        std::tie(arrived, ready, cmdIntermediateJointPos) = bravo->go_to_joint_position(desired_waypoint, 0.34, 0.05, std::chrono::high_resolution_clock::time_point& last_call)
        if (arrived){
            waypoint_counter++;
        }
        if (waypoint_counter == 2)
        {
            desired_waypoint = WAYPOINT_2;
        }
        if (waypoint_counter == 3){
            desired_waypoint = WAYPOINT_3;
        }
        if (waypoint_counter == 4){
            desired_waypoint = WAYPOINT_4;
        }
        p_gain_current_cmd = P_GAIN * bravo->signedAngleDistance(cmdIntermediateJointPos, bravo->get_bravo_joint_states());

        mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity).array() + mA_stiction_stribeck_custom.array() + p_gain_current_cmd.array(); 

        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);
    }
}
//&  MAIN 
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);       
    //const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_amir_2.urdf");
    const std::string urdf_filename = ament_index_cpp::get_package_share_directory("bpl_bravo_description"); + "/urdf/bravo_7_dynamics_no_ee.urdf";
    const std::string tool_link = std::string("EE");   
    auto bravo            = std::make_shared<bravo_handler<double>>(urdf_filename, tool_link);//(urdf_filename); //! CONSTRUCTOR
    auto executor         = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();        
    std::thread executor_thread([&executor]() {
            executor->spin();
    });
    program_loop(bravo);
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}



