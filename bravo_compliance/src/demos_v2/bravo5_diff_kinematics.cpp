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
#include "general_libs_unite/joysticks/airbus_joystick.h"
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

Eigen::Vector<double, 4> HOME = (Eigen::Vector<double, 4>() << 3.14, 1.70, 1.20, 0.0).finished();

void program_loop(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo_handler<double>> bravo){     
    std::chrono::high_resolution_clock::time_point sim_last_integration_time   = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point sim_finish_integration_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point sim_start_integration_time  = std::chrono::high_resolution_clock::now();
    Eigen::Vector<double, 4> sim_joint_whole_integration = HOME;
    auto start_time = std::chrono::steady_clock::now();
    Eigen::Vector3d gravity_vector;
    gravity_vector << 0.0, 0.0, -9.81;              //! GRAVITY VECTOR FOR ARM MOUNTING POINT
    bravo->kinodynamics.change_gravity_vector(gravity_vector); 
    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests, NOT RECOMMENDED BEYOND 200Hz
        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; 
        }
    }    
    bravo->set_control_mode(set_bravo_control_mode::current_control);
    Eigen::VectorXd mA_joint_current_cmd(4), coulomb_custom(4), stiction_custom(4), pd_current_cmd(4);
    coulomb_custom  << 350.0, 350.0, 350.0, 450.0;
    stiction_custom << 500.0, 500.0, 500.0, 500.0;
    mA_joint_current_cmd << 0.0, 0.0, 0.0, 0.0;
    pd_current_cmd  << 0.0, 0.0, 0.0, 0.0;
    double MAX_CURRENT_mA = 2000.0;
    double MAX_CURRENT_mA_GO_HOME = 1500.0;
    double MAX_MANIPULABILITY = 13.0;
    double MAX_SPEED_JOY = 0.15; // m/s
    Eigen::Vector4d prev_err = Eigen::Vector4d::Zero();
    Eigen::Vector<double, 4> P_GAIN, D_GAIN;
    P_GAIN << 4000.0, 4000.0, 4000.0, 4000.0;
    D_GAIN << 200.0, 200.0, 200.0, 200.0;
    bool arrived2HOME = false;
    bool switch_state = true; 
    Eigen::Vector<double, 4> desired_waypoint = HOME;

    std::chrono::high_resolution_clock::time_point last_call = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point last_call2 = std::chrono::high_resolution_clock::now();
    while (rclcpp::ok()) {
        //& GRAVITY AND FRICTION COMPENSATION 
        Eigen::VectorXd mA_stiction_stribeck_custom = bravo->kinodynamics.compute_I_ff_stribeck_smooth(bravo->get_bravo_joint_velocities(), coulomb_custom, stiction_custom, 0.045, 100.0, 0.03);
        Eigen::VectorXd tau_gravity                 = bravo->kinodynamics.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(4), Eigen::VectorXd::Zero(4));
         
        //& GO HOME STATE
        if ((bravo->compute_manipulability() > MAX_MANIPULABILITY) || (!arrived2HOME)){
            switch_state = true;
            desired_waypoint = HOME;
            arrived2HOME = bravo->is_in_desired_configuration(0.4, desired_waypoint, bravo->get_bravo_joint_states());   
            std::cout << "ArriveHome ?: " << arrived2HOME << " MANIPULABILITY : "<< bravo->compute_manipulability() << std::endl;     
            //std::cout << "pd_current_cmd: " << pd_current_cmd.transpose() << std::endl;
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity).array() + mA_stiction_stribeck_custom.array() + pd_current_cmd.array();
        }
        else{
            //& JOYSTICK TELEOP
            if (airbus_joy->enableBaseMotion){ 
                std::cout << "Switching 2 to JOYSTICK MOTION" << std::endl;
                if (switch_state){
                    
                    switch_state = false;
                    sim_joint_whole_integration = bravo->get_bravo_joint_states();
                    sim_start_integration_time = std::chrono::high_resolution_clock::now();
                    sim_last_integration_time = std::chrono::high_resolution_clock::now();
                    sim_finish_integration_time = std::chrono::high_resolution_clock::now();
                    std::cout << "Switching to JOYSTICK MOTION" << std::endl;
                }
                Eigen::Vector<double, 3> twist_joy_fixed;
                float reduced_vel_factor = 0.5;
                float reduce_w_factor = 0.2; 
                twist_joy_fixed << airbus_joy->teleop_VelZ, airbus_joy->teleop_VelX, airbus_joy->teleop_VelY;
                twist_joy_fixed.head<3>() = twist_joy_fixed.head<3>() * reduced_vel_factor; // Reduce linear velocity       
                twist_joy_fixed[0] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[0], MAX_SPEED_JOY, -MAX_SPEED_JOY);
                twist_joy_fixed[1] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[1], MAX_SPEED_JOY, -MAX_SPEED_JOY);
                twist_joy_fixed[2] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[2], MAX_SPEED_JOY, -MAX_SPEED_JOY);
                Eigen::MatrixXd jacobian = bravo->kinodynamics.fixedJacobian(sim_joint_whole_integration);
                Eigen::MatrixXd jacobian3x4 = jacobian.topRows(3);
                Eigen::Vector<double, 4> sim_joint_vel_integration = jacobian3x4.colPivHouseholderQr().solve(twist_joy_fixed);
                sim_finish_integration_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = sim_finish_integration_time - sim_start_integration_time;
                sim_joint_whole_integration = sim_joint_whole_integration + sim_joint_vel_integration*elapsed.count();
                sim_start_integration_time = std::chrono::high_resolution_clock::now();
                sim_last_integration_time = std::chrono::high_resolution_clock::now();
                desired_waypoint = sim_joint_whole_integration;
            }
        }        

        double dt = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - last_call2).count();
        last_call2 = std::chrono::high_resolution_clock::now();
        if (dt <= 1e-6) dt = 1e-3;
        Eigen::Vector4d err = bravo->signedAngleDistance(desired_waypoint, bravo->get_bravo_joint_states());
        Eigen::Vector4d derr = (err - prev_err) / dt;
        prev_err = err;
        //pd_current_cmd = P_GAIN * bravo->signedAngleDistance(cmdIntermediateJointPos, bravo->get_bravo_joint_states());
        pd_current_cmd = P_GAIN.cwiseProduct(err) + D_GAIN.cwiseProduct(derr);
        pd_current_cmd[0] = general_utils::VAL_SAT<double>(pd_current_cmd[0], MAX_CURRENT_mA_GO_HOME, -MAX_CURRENT_mA_GO_HOME);
        pd_current_cmd[1] = general_utils::VAL_SAT<double>(pd_current_cmd[1], MAX_CURRENT_mA_GO_HOME, -MAX_CURRENT_mA_GO_HOME);
        pd_current_cmd[2] = general_utils::VAL_SAT<double>(pd_current_cmd[2], MAX_CURRENT_mA_GO_HOME, -MAX_CURRENT_mA_GO_HOME);
        pd_current_cmd[3] = general_utils::VAL_SAT<double>(pd_current_cmd[3], 2000, -2000);
            //pd_current_cmd[0] = 0.0;
        mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity).array() + mA_stiction_stribeck_custom.array() + pd_current_cmd.array();
        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);

        std::cout << "Difference to Goal: " << bravo->signedAngleDistance(desired_waypoint, bravo->get_bravo_joint_states()).transpose()  << "   p_gain: " << pd_current_cmd.transpose() <<"      joint_cmd: " << mA_joint_current_cmd.transpose() << std::endl;

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
    const std::string urdf_filename = ament_index_cpp::get_package_share_directory("bpl_bravo_description") + "/urdf/bravo_5_dynamics_no_ee_pinocchio.urdf";
    const std::string tool_link = std::string("contact_point");  
    auto joystick         = std::make_shared<airbus_joystick_bravo_twist_ee>();
    auto bravo            = std::make_shared<bravo_handler<double>>(urdf_filename, tool_link);//(urdf_filename); //! CONSTRUCTOR
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



