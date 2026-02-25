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
#include "bravo_compliance/bravo_cpp/bravo_handler/bravo_handler_v2.h"
#include <string>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

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
    //& DEFINE PARAMETERS
    Eigen::VectorXd mA_joint_current_cmd(4), coulomb_custom(4), stiction_custom(4), Nm_friction_compensation(4), mA_friction_compensation(4);
    coulomb_custom  << 300.0, 200.0, 200.0, 200.0;
    stiction_custom << 350.0, 350.0, 350.0, 350.0;
    coulomb_custom  << 350.0, 350.0, 450.0, 550.0;
    stiction_custom << 450.0, 450.0, 550.0, 650.0;
    mA_joint_current_cmd << 0.0, 0.0, 0.0, 0.0;
    double MAX_CURRENT_mA = 1500.0;

    double j1_g0 = 0.0;
    double j1_g1 = 0.0;
    double j1_g2 = 0.0;
    double j1_g3 = 14.22636;
    double j1_g4 = 10.07377;
    double j1_g5 = 1.0;

    double j2_g0 = 0.0;
    double j2_g1 = 0.0;
    double j2_g2 = 0.0;
    double j2_g3 = 14.22636;
    double j2_g4 = 10.07377;
    double j2_g5 = 1.0;

    double j3_g0 = -24;
    double j3_g1 = -69;
    double j3_g2 = 9.1;
    double j3_g3 = -44;
    double j3_g4 = 13;
    double j3_g5 = 3;

    double j4_g0 = 0.0;
    double j4_g1 = 0.0;
    double j4_g2 = 0.0;
    double j4_g3 = 14.22636;
    double j4_g4 = 10.07377;
    double j4_g5 = 1.23;
    double saturation = 50; 

    // g0 = 0.0;
    // g1 = 0.0;
    // g2 = 0.0;
    // g3= 17.66;
    // g4 = 10.71;
    // g5 = 0.3; 
    
    while (rclcpp::ok()) {
        //& PLOT MANIPULABILITY
        std::cout << "Manipulablity: "<< bravo->compute_manipulability_position() << std::endl;
        Eigen::VectorXd joint_velocity_friction = bravo->get_bravo_joint_velocities();
        Nm_friction_compensation[0] = bravo->kinodynamics.computeFriction(joint_velocity_friction[0], j1_g0, j1_g1, j1_g2, j1_g3, j1_g4, j1_g5, saturation);
        Nm_friction_compensation[1] = bravo->kinodynamics.computeFriction(joint_velocity_friction[1], j2_g0, j2_g1, j2_g2, j2_g3, j2_g4, j2_g5, saturation);
        Nm_friction_compensation[2] = bravo->kinodynamics.computeFriction(joint_velocity_friction[2], j3_g0, j3_g1, j3_g2, j3_g3, j3_g4, j3_g5, saturation);
        Nm_friction_compensation[3] = bravo->kinodynamics.computeFriction(joint_velocity_friction[3], j4_g0, j4_g1, j4_g2, j4_g3, j4_g4, j4_g5, saturation);
        mA_friction_compensation = bravo->torqueNm_2_currentmA(Nm_friction_compensation);
        Eigen::VectorXd mA_stiction_stribeck_custom = bravo->kinodynamics.compute_I_ff_stribeck_smooth(bravo->get_bravo_joint_velocities(), coulomb_custom, stiction_custom, 0.045, 100.0, 0.03);
        Eigen::VectorXd tau_gravity                 = bravo->kinodynamics.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(4), Eigen::VectorXd::Zero(4));
        mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
        mA_joint_current_cmd = mA_joint_current_cmd.array() + mA_friction_compensation.array(); // Add stiction compensation
        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);
        bravo->publish_bravo_joint_states();
    }
}
//&  MAIN 
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);       
    //const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_amir_2.urdf");
    const std::string urdf_filename = ament_index_cpp::get_package_share_directory("bpl_bravo_description") + "/urdf/bravo_5_dynamics_no_ee_pinocchio.urdf";
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



