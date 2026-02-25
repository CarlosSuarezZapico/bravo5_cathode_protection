/**
 *    @file  bravo5_cp_compliance.cpp
 *    @brief Program to make interaction with Bravo arms. Design for cathode-protection tasks
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created       5-Dec-2025
 *    Modification 19-Jan-2026
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
#include "general_libs_unite/joysticks/airbus_joystick.h"
#include "general_libs_unite/interaction/stiffness_control_position.h"

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

enum class StateMachine{
    GO_HOME,
    JOYSTICK_TELEOP,
    STIFFNESS_CONTROL
};

void program_loop(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo_handler<double>> bravo){     
    //& PARAMETERS
    StateMachine prev_state = StateMachine::GO_HOME;
    double MAX_CURRENT_mA = 2500.0;
    double MAX_CURRENT_mA_GO_HOME = 1500.0;
    double VELOCITY_X_NOMINAL = 0.02; // m/s
    double GAIN_FORCE_X = 0.01; // N to m/s
    double MAX_MANIPULABILITY = 7.0;
    double MAX_RATIO_FORCE_ELLIPSOID = 5.0;
    double MAX_SPEED_JOY = 0.25; // m/s
    double SPEED_GO_HOME = 0.15; // adimensional? I don't know
    Eigen::Vector<double, 4> HOME = (Eigen::Vector<double, 4>() << 3.14, 1.70, 1.20, 0.0).finished(); //! define home for bravo5
    Eigen::Vector3d gravity_vector = (Eigen::Vector3d() << 0.0, 0.0, -9.81).finished();
    Eigen::Vector3d force_cmd = Eigen::Vector3d::Zero();
    //PD CONTROLLER
    std::chrono::high_resolution_clock::time_point sim_last_integration_time   = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point sim_finish_integration_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point sim_start_integration_time  = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point last_call_pd = std::chrono::high_resolution_clock::now();
    Eigen::Vector4d prev_err = Eigen::Vector4d::Zero();
    Eigen::Vector<double, 4> sim_joint_whole_integration = HOME;
    Eigen::Vector4d tau_gravity = Eigen::Vector4d::Zero();
    Eigen::Vector<double, 4> P_GAIN, D_GAIN;
    P_GAIN << 6500.0, 6500.0, 6500.0, 6500.0;
    D_GAIN << 200.0, 200.0, 200.0, 200.0;
    bool arrived2HOME = false;
    bool switch_state = true;
    Eigen::Vector<double, 4> desired_waypoint = HOME; 

    //$ FRICTION JOINT PARAMETERS (VALID FOR BRAVO7 OR BRAVO5)
    Eigen::VectorXd mA_joint_current_cmd(4), mA_stiffness_current_cmd(4), mA_pd_current_cmd(4), joint_velocity_friction(4), Nm_friction_compensation(4), mA_friction_compensation(4), coulomb_custom(4), stiction_custom(4), qd_stiction(4);
    coulomb_custom  << 450.0, 450.0, 450.0, 450.0;
    stiction_custom << 650.0, 650.0, 650.0, 650.0;
    qd_stiction     << 0.006, 0.006, 0.006, 0.006;
    mA_joint_current_cmd     = Eigen::VectorXd::Zero(4);
    mA_stiffness_current_cmd = Eigen::VectorXd::Zero(4);
    mA_pd_current_cmd        = Eigen::VectorXd::Zero(4);

    auto start_time = std::chrono::steady_clock::now();
    stiffness_control_position <double> stiffness_controller;
    
    //& AUX VARIABLES, probably should be in stiffness controller
    Eigen::Vector<double, 4> ref_joint_pos;  
    Eigen::VectorXd joint_torque_cmd, joint_velocity_cmd, joint_velocity_fdb;
    joint_torque_cmd.resize(4);
    joint_velocity_cmd.resize(4);
    joint_velocity_fdb.resize(4);
    
    //& ARM CONNECTION
    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests  
             //! GRAVITY VECTOR FOR ARM MOUNTING POINT
        bravo->kinodynamics.change_gravity_vector(gravity_vector); //!changing gravity compensation
        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
    }    
    std::cerr << "Manipulability condition number: " << bravo->compute_manipulability_position() << std::endl;
    ref_joint_pos = bravo->get_bravo_joint_states();
    stiffness_controller.set_pos_stiffness(Eigen::Vector<double, 3>(20.0, 1000.0, 200.0));
    stiffness_controller.set_pos_damping(Eigen::Vector<double, 3>(7.0, 100.0, 20.0));
    stiffness_controller.set_desired_force(Eigen::Vector3d(400.0, 0.0, 0.0)); // Desired force in Z
    stiffness_controller.set_ref_ee_position(bravo->kinodynamics.FK_ee_pos(ref_joint_pos));
    //& MAIN LOOPcoulomb_custom
    while (rclcpp::ok()) {
        //& GO TO CONFIGURATION HOME
        if ( (bravo->compute_manipulability_position() > MAX_MANIPULABILITY) || (!arrived2HOME) || (airbus_joy->goHome) ){
            if (prev_state != StateMachine::GO_HOME){      
                //* STATE INITIALIZATION 
                std::cout << "Switching to GO HOME" << std::endl;             
            }
            desired_waypoint = HOME;
            arrived2HOME = bravo->is_in_desired_configuration(0.2, desired_waypoint, bravo->get_bravo_joint_states());   
            std::cout << "ArriveHome ?: " << arrived2HOME << " MANIPULABILITY : "<< bravo->compute_manipulability() << std::endl;     
            prev_state = StateMachine::GO_HOME;
            joint_velocity_friction = bravo->get_bravo_joint_velocities();
        }
        //& JOYSTICK TELEOP
        else if (airbus_joy->enableBaseMotion){ 
            if (prev_state != StateMachine::JOYSTICK_TELEOP){      
                //* STATE INITIALIZATION 
                std::cout << "Switching to JOYSTICK TELEOP" << std::endl;             
                switch_state = false;
                sim_joint_whole_integration = bravo->get_bravo_joint_states();
                sim_start_integration_time = std::chrono::high_resolution_clock::now();
                sim_last_integration_time = std::chrono::high_resolution_clock::now();
                sim_finish_integration_time = std::chrono::high_resolution_clock::now();
            }
            Eigen::Vector<double, 3> twist_joy_fixed;
            float reduced_vel_factor = 1.2;
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
            joint_velocity_friction = bravo->get_bravo_joint_velocities();
            prev_state = StateMachine::JOYSTICK_TELEOP;
        }
        //& STIFFNESS CONTROL 
        else{
            if (prev_state != StateMachine::STIFFNESS_CONTROL){      
                //* STATE INITIALIZATION 
                std::cout << "Switching to STIFFNESS CONTROL" << std::endl;             
                stiffness_controller.set_ref_ee_position(bravo->kinodynamics.FK_ee_pos(bravo->get_bravo_joint_states()));
            }
            Eigen::MatrixXd Jacobian = bravo->kinodynamics.fixedJacobian(bravo->get_bravo_joint_states());
            Eigen::MatrixXd jacobian3x4 = Jacobian.topRows(3);
            force_cmd = stiffness_controller.compute_force_action(
                        stiffness_controller.get_ref_ee_position() - bravo->kinodynamics.FK_ee_pos(bravo->get_bravo_joint_states()), 
                        stiffness_controller.get_vel_ee() - jacobian3x4*bravo->get_bravo_joint_velocities());
            //$  Joint commands               
            joint_velocity_cmd = jacobian3x4.colPivHouseholderQr().solve(stiffness_controller.get_vel_ee());
            //& HANDLE FRICTION TO MAKE BRAVO AS COMPLIANT AS POSSIBLE
            joint_velocity_fdb = bravo->get_bravo_joint_velocities();
            if ((joint_velocity_fdb.array().abs() < 0.04).all()){
                joint_velocity_friction = joint_velocity_cmd;
            }
            else{
                joint_velocity_friction = joint_velocity_fdb;
            }
            //joint_velocity_friction = joint_velocity_fdb;  //! delete this
            //mA_friction_compensation =  bravo->kinodynamics.compute_I_ff_stribeck(joint_velocity_friction, coulomb_custom, stiction_custom, qd_stiction);
            joint_torque_cmd = jacobian3x4.colPivHouseholderQr().solve(force_cmd);
            mA_stiffness_current_cmd = bravo->torqueNm_2_currentmA(joint_torque_cmd);
            
            //! CHECKING IN SATURATION IS HAPPENING
            bool exceeds_limit = false;
            for (int i = 0; i < 4; i++) {
                if (std::abs(mA_stiffness_current_cmd[i]) > MAX_CURRENT_mA) {
                    exceeds_limit = true;
                    break;
                }
            }
            if (exceeds_limit) {
                std::cout << "Current limit exceeded, SATURATING" << std::endl;
                std::cout << "Current [0]: " << mA_stiffness_current_cmd[0]
                                    << " [1]: " << mA_stiffness_current_cmd[1]
                                    << " [2]: " << mA_stiffness_current_cmd[2]
                                    << " [3]: " << mA_stiffness_current_cmd[3]
                                    << std::endl;
            }
            prev_state = StateMachine::STIFFNESS_CONTROL;
            last_call_pd = std::chrono::high_resolution_clock::now();
            std::cout << " MANIPULABILITY : "<< bravo->compute_manipulability() << std::endl; 
            
        }           
        double dt = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - last_call_pd).count();
        last_call_pd = std::chrono::high_resolution_clock::now();
        if (dt <= 1e-6) dt = 1e-3;
        Eigen::Vector4d err = bravo->signedAngleDistance(desired_waypoint, bravo->get_bravo_joint_states());
        Eigen::Vector4d derr = (err - prev_err) / dt;
        prev_err = err;
        //mA_pd_current_cmd = P_GAIN * bravo->signedAngleDistance(cmdIntermediateJointPos, bravo->get_bravo_joint_states());
        mA_pd_current_cmd = P_GAIN.cwiseProduct(err) + D_GAIN.cwiseProduct(derr);
        mA_pd_current_cmd[0] = general_utils::VAL_SAT<double>(mA_pd_current_cmd[0], MAX_CURRENT_mA_GO_HOME, -MAX_CURRENT_mA_GO_HOME);
        mA_pd_current_cmd[1] = general_utils::VAL_SAT<double>(mA_pd_current_cmd[1], MAX_CURRENT_mA_GO_HOME, -MAX_CURRENT_mA_GO_HOME);
        mA_pd_current_cmd[2] = general_utils::VAL_SAT<double>(mA_pd_current_cmd[2], MAX_CURRENT_mA_GO_HOME, -MAX_CURRENT_mA_GO_HOME);
        mA_pd_current_cmd[3] = general_utils::VAL_SAT<double>(mA_pd_current_cmd[3], 2000, -2000);
        
        //mA_friction_compensation = bravo->kinodynamics.compute_I_ff_stribeck(joint_velocity_friction, coulomb_custom, stiction_custom, qd_stiction);
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

        // g0 = 0.0;
        // g1 = 0.0;
        // g2 = 0.0;
        // g3= 17.66;
        // g4 = 10.71;
        // g5 = 0.3; 
  
        Nm_friction_compensation[0] = bravo->kinodynamics.computeFriction(joint_velocity_friction[0], j1_g0, j1_g1, j1_g2, j1_g3, j1_g4, j1_g5, saturation);
        Nm_friction_compensation[1] = bravo->kinodynamics.computeFriction(joint_velocity_friction[1], j2_g0, j2_g1, j2_g2, j2_g3, j2_g4, j2_g5, saturation);
        Nm_friction_compensation[2] = bravo->kinodynamics.computeFriction(joint_velocity_friction[2], j3_g0, j3_g1, j3_g2, j3_g3, j3_g4, j3_g5, saturation);
        Nm_friction_compensation[3] = bravo->kinodynamics.computeFriction(joint_velocity_friction[3], j4_g0, j4_g1, j4_g2, j4_g3, j4_g4, j4_g5, saturation);
        mA_friction_compensation = bravo->torqueNm_2_currentmA(Nm_friction_compensation);

        if (prev_state == StateMachine::STIFFNESS_CONTROL){
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity).array() + mA_friction_compensation.array() + mA_stiffness_current_cmd.array();
        }
        else{
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity).array() + mA_friction_compensation.array() + mA_pd_current_cmd.array();
        }
        //& COMPOSE JOINT COMMAND
        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA); 
        bravo->publish_bravo_joint_states();
    }

    mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity).array();
    bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);
    bravo->publish_bravo_joint_states();

}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        const std::string urdf_filename = ament_index_cpp::get_package_share_directory("bpl_bravo_description") + "/urdf/bravo_5_dynamics_no_ee_pinocchio.urdf";
        const std::string tool_link = std::string("contact_point");  
        auto joystick         = std::make_shared<airbus_joystick_bravo_twist_ee>();
        auto bravo            = std::make_shared<bravo_handler<double>>(urdf_filename, tool_link);//(urdf_filename);
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



