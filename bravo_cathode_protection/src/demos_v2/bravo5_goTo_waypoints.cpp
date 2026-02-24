/**
 *    @file  bravo5_goTo_waypoints.cpp
 *    @brief Unittest for the Bravo5 arm. The program goes through a series of 
 *    joint position waypoints in position control instruction (bravo instruction)
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      8-Jan-2025
 *    Modification 19-Jan-2025
 *    Project: UNITE
 *    State:   //& WORKING
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
#include "general_libs_unite/general_utils/general_utils.h"
#include <string>
#include <iostream>
#include <atomic>
#include <csignal>
#include <rclcpp/rclcpp.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;
using namespace general_utils;

Eigen::Vector<double, 4> WAYPOINT_1 = (Eigen::Vector<double, 4>() << 3.14, 2.706, 0.946, 0.0).finished();
Eigen::Vector<double, 4> WAYPOINT_2 = (Eigen::Vector<double, 4>() << 3.14, 2.9, 1.476, 0.0).finished();
Eigen::Vector<double, 4> WAYPOINT_3 = (Eigen::Vector<double, 4>() << 3.14, 2.9, 0.946, 0.0).finished();
Eigen::Vector<double, 4> WAYPOINT_4 = (Eigen::Vector<double, 4>() << 3.14, 2.9, 0.946, 0.0).finished();

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
    Eigen::VectorXd mA_joint_current_cmd(4), coulomb_custom(4), stiction_custom(4), p_gain_current_cmd(4), Nm_friction_compensation(4), mA_friction_compensation(4);
    coulomb_custom  << 200.0, 200.0, 200.0, 200.0;
    stiction_custom << 350.0, 350.0, 350.0, 350.0;
    mA_joint_current_cmd << 0.0, 0.0, 0.0, 0.0;
    p_gain_current_cmd   << 0.0, 0.0, 0.0, 0.0;
    double MAX_CURRENT_mA = 1500.0;
    double j1_g0 = 0.0;
    double j1_g1 = 0.0;
    double j1_g2 = 0.0;
    double j1_g3 = 14.66651;
    double j1_g4 = 10.09561;
    double j1_g5 = 1.25110;

    double j2_g0 = 0.0;
    double j2_g1 = 0.0;
    double j2_g2 = 0.0;
    double j2_g3 = 14.50959;
    double j2_g4 = 10.09275;
    double j2_g5 = 1.14381;

    double j3_g0 = 0.0;
    double j3_g1 = 0.0;
    double j3_g2 = 0.0;
    double j3_g3 = 13.98409;
    double j3_g4 = 12.97659;
    double j3_g5 = 21.69165;

    double j4_g0 = 0.0;
    double j4_g1 = 0.0;
    double j4_g2 = 0.0;
    double j4_g3 = 15.82785;
    double j4_g4 = 14.35639;
    double j4_g5 = 24.04595;
    double saturation = 50; 

    Eigen::Vector4d prev_err = Eigen::Vector4d::Zero();
    Eigen::Vector<double, 4> P_GAIN, D_GAIN;
    P_GAIN << 950.0, 950.0, 1100.0, 950.0;
    D_GAIN << 10.0, 10.0, 10.0, 10.0;
    //double P_GAIN = 150.0; 
    bool arrived = false;
    int waypoint_counter = 1; 

    bool ready = false; 
    Eigen::Vector<double, 4> desired_waypoint;
    desired_waypoint = WAYPOINT_1;
    Eigen::Vector<double, 4> cmdIntermediateJointPos; 
    cmdIntermediateJointPos = bravo->get_bravo_joint_states(); 
    std::chrono::high_resolution_clock::time_point last_call = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point last_call2 = std::chrono::high_resolution_clock::now();
    while (rclcpp::ok()) {
        //& GRAVITY AND FRICTION COMPENSATION 
        std::cout << "Manipulablity: "<< bravo->compute_manipulability_position() << std::endl;
        Eigen::VectorXd joint_velocity_friction = bravo->get_bravo_joint_velocities();
        Nm_friction_compensation[0] = bravo->kinodynamics.computeFriction(joint_velocity_friction[0], j1_g0, j1_g1, j1_g2, j1_g3, j1_g4, j1_g5, saturation);
        Nm_friction_compensation[1] = bravo->kinodynamics.computeFriction(joint_velocity_friction[1], j2_g0, j2_g1, j2_g2, j2_g3, j2_g4, j2_g5, saturation);
        Nm_friction_compensation[2] = bravo->kinodynamics.computeFriction(joint_velocity_friction[2], j3_g0, j3_g1, j3_g2, j3_g3, j3_g4, j3_g5, saturation);
        Nm_friction_compensation[3] = bravo->kinodynamics.computeFriction(joint_velocity_friction[3], j4_g0, j4_g1, j4_g2, j4_g3, j4_g4, j4_g5, saturation);
        mA_friction_compensation = bravo->torqueNm_2_currentmA(Nm_friction_compensation);
        Eigen::VectorXd mA_stiction_stribeck_custom = bravo->kinodynamics.compute_I_ff_stribeck_smooth(bravo->get_bravo_joint_velocities(), coulomb_custom, stiction_custom, 0.045, 100.0, 0.03);
        Eigen::VectorXd tau_gravity                 = bravo->kinodynamics.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(4), Eigen::VectorXd::Zero(4));

        std::tie(arrived, ready, cmdIntermediateJointPos) = bravo->go_to_joint_position(desired_waypoint, 0.34, 0.05, last_call);
        bool onwaypoint = bravo->is_in_desired_configuration(0.4, desired_waypoint, bravo->get_bravo_joint_states());
        if (onwaypoint){
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
        if (waypoint_counter > 4){
            desired_waypoint = WAYPOINT_1;
            waypoint_counter = 1;
        }
        double dt = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - last_call2).count();
        last_call2 = std::chrono::high_resolution_clock::now();
        if (dt <= 1e-6) dt = 1e-3;
        Eigen::Vector4d err = bravo->signedAngleDistance(desired_waypoint, bravo->get_bravo_joint_states());
        Eigen::Vector4d derr = (err - prev_err) / dt;
        prev_err = err;
        //p_gain_current_cmd = P_GAIN * bravo->signedAngleDistance(cmdIntermediateJointPos, bravo->get_bravo_joint_states());
        p_gain_current_cmd = P_GAIN.cwiseProduct(err) + D_GAIN.cwiseProduct(derr);
        p_gain_current_cmd[0] = general_utils::VAL_SAT<double>(p_gain_current_cmd[0], 900.0, -900.0);
        p_gain_current_cmd[1] = general_utils::VAL_SAT<double>(p_gain_current_cmd[1], 900.0, -900.0);
        p_gain_current_cmd[2] = general_utils::VAL_SAT<double>(p_gain_current_cmd[2], 900.0, -900.0);
        p_gain_current_cmd[3] = general_utils::VAL_SAT<double>(p_gain_current_cmd[3], 900.0, -900.0);
        //p_gain_current_cmd[0] = 0.0;
       
        //std::cout << "p_gain_current_cmd: " << p_gain_current_cmd.transpose() << std::endl;

        mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity).array() + mA_friction_compensation.array() + p_gain_current_cmd.array(); 

        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);

        std::cout << "Difference to Goal: " << bravo->signedAngleDistance(desired_waypoint, bravo->get_bravo_joint_states()).transpose()  << "   p_gain: " << p_gain_current_cmd.transpose() <<"      joint_cmd: " << mA_joint_current_cmd.transpose() << std::endl;
        //std::cout << "Current Joint Feedback: " << bravo->get_bravo_joint_states() << std::endl;

    }
    Eigen::VectorXd mA_stiction_stribeck_custom = bravo->kinodynamics.compute_I_ff_stribeck_smooth(bravo->get_bravo_joint_velocities(), coulomb_custom, stiction_custom, 0.045, 100.0, 0.03);
    Eigen::VectorXd tau_gravity                 = bravo->kinodynamics.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(4), Eigen::VectorXd::Zero(4));
    mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity).array() + mA_stiction_stribeck_custom.array();
    bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);

}
//&  MAIN 
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);       
    //const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_amir_2.urdf");
    const std::string urdf_filename = ament_index_cpp::get_package_share_directory("bpl_bravo_description") + "/urdf/bravo_5_dynamics_no_ee_pinocchio_rov_mount.urdf";
    const std::string tool_link = std::string("contact_point");  
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



