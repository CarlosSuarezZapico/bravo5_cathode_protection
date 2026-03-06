/**
 *    @file  bravo5_cp_mobile_axis.cpp
 *    @brief Program to make interaction with Bravo 5 arm. Design for cathode-protection tasks
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created       5-Dec-2025
 *    Modification  24-Feb-2026
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */

#include "bravo_cathode_protection/bravo_cpp/bravo_handler/bravo_handler_v2.h"
#include "bravo_cathode_protection/bravo_cpp/utils/bravo_dashboard.h"
#include "bravo_cathode_protection/bravo_cpp/utils/bravo_logger.h"
#include "bravo_cathode_protection/bravo_cpp/joysticks/airbus_joystick.h"
#include "bravo_cathode_protection/bravo_cpp/interaction/stiffness_control_position.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include "pinocchio/fwd.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stdexcept>
#include <filesystem>
#include <thread>
#include <fstream>
#include <cmath>
#include <yaml-cpp/yaml.h>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

enum class StateMachine{
    GO_HOME,
    HOME_WAITING,
    TELEOP,
    STIFFNESS_CONTROL
};

static const char* state_machine_to_string(StateMachine state)
{
    switch (state) {
        case StateMachine::GO_HOME: return "GO_HOME";
        case StateMachine::HOME_WAITING: return "HOME_WAITING";
        case StateMachine::TELEOP: return "TELEOP";
        case StateMachine::STIFFNESS_CONTROL: return "STIFFNESS_CONTROL";
        default: return "UNKNOWN";
    }
}

bool program_loop(std::shared_ptr<airbus_joystick_bravo5_CP> airbus_joy,
                  std::shared_ptr<bravo_handler<double>> bravo,
                  const stiffness_control_config::StiffnessParams& stiff_params,
                  const bravo_utils::RuntimeConfig& runtime_config,
                  const bravo_utils::FrictionConfig& friction_config,
                  const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& joint_state_pub,
                  const std::shared_ptr<rclcpp::Node>& joint_state_node,
                  const std::shared_ptr<bravo_utils::TerminalDashboard>& dashboard,
                  const std::shared_ptr<bravo_utils::Logger>& logger,
                  const std::string& ip_address,
                  int udp_port){     
    const auto active_logger = logger ? logger : std::make_shared<bravo_utils::Logger>(bravo_utils::write_log_stderr);

    //& PARAMETERS
    const double MAX_CURRENT_mA            = runtime_config.max_current_mA; // mA
    const double MAX_CURRENT_mA_GO_HOME    = runtime_config.max_current_mA_go_home; // mA
    const double MAX_MANIPULABILITY        = runtime_config.max_manipulability;    // Unitless
    const double HOME_STABLE_DWELL_SEC     = 0.35;   // seconds
    const double MAX_RATIO_FORCE_ELLIPSOID = runtime_config.max_ratio_force_ellipsoid;    // Unitless
    const double MAX_SPEED_TELEOP          = runtime_config.max_speed_teleop;   // m/s
    const double LOOP_FREQUENCY            = runtime_config.loop_frequency;  // Hz (Similar to the arm frequency)
    const Eigen::Vector<double, 4> HOME    = runtime_config.home; //! define home for bravo5
    const Eigen::Vector3d GRAVITY_VECTOR   = runtime_config.gravity_vector;
    const Eigen::Matrix<double,4,6>& FRICTION_MAT = friction_config.friction_mat;
    const double FRICTION_SATURATION = friction_config.friction_saturation; 

    //& VARIABLES DIFF KINEMATICS
    std::chrono::steady_clock::time_point sim_finish_integration_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point sim_start_integration_time  = std::chrono::steady_clock::now();
    Eigen::Vector<double, 4> sim_joint_whole_integration = HOME;

    //& PID CONTROLLER
    PID<double> j1_PID(3500.0, 0.0, 200.0, -MAX_CURRENT_mA_GO_HOME, MAX_CURRENT_mA_GO_HOME); // P, I, D gains and output limits
    PID<double> j2_PID(3500.0, 0.0, 200.0, -MAX_CURRENT_mA_GO_HOME, MAX_CURRENT_mA_GO_HOME); // P, I, D gains and output limits
    PID<double> j3_PID(3500.0, 0.0, 200.0, -MAX_CURRENT_mA_GO_HOME, MAX_CURRENT_mA_GO_HOME); // P, I, D gains and output limits
    PID<double> j4_PID(3500.0, 0.0, 200.0, -MAX_CURRENT_mA_GO_HOME, MAX_CURRENT_mA_GO_HOME); // P, I, D gains and output limits
    std::chrono::steady_clock::time_point last_call_pd = std::chrono::steady_clock::now();

    //& STIFFNESS CONTROLLER
    stiffness_control_position <double> stiffness_controller;
    
    //& STATE MACHINE VARIABLES
    StateMachine prev_state    = StateMachine::GO_HOME;
    StateMachine current_state = StateMachine::GO_HOME;
    bool arrived2HOME = false;
    std::chrono::steady_clock::time_point home_stable_since = std::chrono::steady_clock::now();

    //& AUX VARIABLES
    Eigen::Vector4d desired_joint_pos = HOME; 
    Eigen::Vector4d joint_velocity_friction, Nm_friction_compensation, mA_friction_compensation, joint_torque_cmd;
    Eigen::VectorXd joint_velocity_fdb(4), joint_postion_fdb(4);
    Eigen::Vector4d joint_velocity_cmd       = Eigen::Vector4d::Zero();
    Eigen::Vector4d mA_joint_current_cmd     = Eigen::Vector4d::Zero();
    Eigen::Vector4d mA_stiffness_current_cmd = Eigen::Vector4d::Zero();
    Eigen::Vector4d mA_pd_current_cmd        = Eigen::Vector4d::Zero();
    Eigen::Vector4d Nm_gravity               = Eigen::Vector4d::Zero();
    Eigen::Vector4d Nm_gravity_compensation  = Eigen::Vector4d::Zero();
    Eigen::Vector3d force_cmd                = Eigen::Vector3d::Zero();
    double dashboard_vel_ee_x                = 0.0;
    double dashboard_exerted_force_x         = 0.0;
    const double dashboard_desired_force_x   = stiff_params.desired_force[0];
    Eigen::MatrixXd jacobian3x4_mobile       = Eigen::MatrixXd::Zero(3,4);
    Eigen::Vector3d twist_joy_world          = Eigen::Vector3d::Zero();
    Eigen::Vector3d twist_joy_mobile         = Eigen::Vector3d::Zero();
    double manipulabilityXy = 0.0;
    double manipulabilityXz = 0.0;
    double manipulability   = 0.0;
    bool   CMD_ENABLE_TELEOP  = false;
    bool   CMD_GO_HOME        = false;
    bool   CMD_MAKE_READING   = false;
    bool   ALLOW_LEAVING_HOME = false;
    bool   LEAVE_HOME_FLAG    = false;
    Eigen::Matrix3d ref_ee_rot, current_ee_rot;
    Eigen::Vector3d current_ee_pos           = Eigen::Vector3d::Zero();

    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.name = {"joint1", "joint2", "joint3", "joint4"};
    joint_state_msg.position.resize(4, 0.0);
    joint_state_msg.velocity.resize(4, 0.0);
    
    //! GRAVITY VECTOR FOR ARM MOUNTING POINT
    bravo->kinodynamics.change_gravity_vector(GRAVITY_VECTOR); //!changing gravity compensation
    bravo->set_bravo_frequency_packet_exchange(300); //! Set frequency of requests  
    auto start_time = std::chrono::steady_clock::now();

    if (dashboard) {
        dashboard->setArmStatus("CONNECTING", 0.0, true, 0.0);
    }
    
    //& ARM CONNECTION
    while (!bravo->isConnected()){        
        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            BRAVO_LOG_ERROR(
                *active_logger,
                "Timeout: no UDP feedback from Bravo after 5 seconds (target ",
                ip_address,
                ":",
                udp_port,
                "). Check network routing.");
            if (dashboard) {
                dashboard->setArmStatus("DISCONNECTED", manipulability, false, 0.0);
            }
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }    

    //& STIFFNESS CONTROLLER PARAMETERS
    stiffness_controller.set_pos_stiffness(stiff_params.pos_stiffness);
    stiffness_controller.set_pos_damping(stiff_params.pos_damping);
    stiffness_controller.set_gain_force(stiff_params.gain_force);
    stiffness_controller.set_nominal_vel(stiff_params.nominal_vel);
    stiffness_controller.set_desired_force(stiff_params.desired_force);
    stiffness_controller.set_max_vel(stiff_params.maximum_vel);

    stiffness_controller.set_reference_integration_local(true);
    Eigen::Vector4d ref_joint_pos = bravo->get_bravo_joint_states();
    Eigen::Vector3d ref_ee_pos_init;
    std::tie(ref_ee_pos_init, ref_ee_rot) = bravo->kinodynamics.FK_ee(ref_joint_pos);
    stiffness_controller.set_ref_ee_position(ref_ee_pos_init);
    stiffness_controller.set_reference_integration_rotation(ref_ee_rot);

    rclcpp::Rate loop_rate(LOOP_FREQUENCY);  

    //& ------MAIN LOOP--------
    while (rclcpp::ok()) {
        //& SHUTDOWN IF ARM DISCONNECTED
        if (!bravo->isConnected()) {
            BRAVO_LOG_ERROR(*active_logger, "[safety] Bravo arm communication lost. Controller is shutting down for safety.");
            if (dashboard) {
                dashboard->setArmStatus("DISCONNECTED", manipulability, false, bravo->get_udp_rx_frequency_hz());
            }
            rclcpp::shutdown();
            return false;
        }
        //& RECURRENT CALCULATIONS
        joint_postion_fdb  = bravo->get_bravo_joint_states();
        joint_velocity_fdb = bravo->get_bravo_joint_velocities();
        jacobian3x4_mobile = bravo->kinodynamics.localJacobian(joint_postion_fdb).topRows(3);
        std::tie(current_ee_pos, current_ee_rot) = bravo->kinodynamics.FK_ee(joint_postion_fdb);
        manipulability     = bravo->kinodynamics.compute_manipulability_position(bravo->get_bravo_joint_states());
        std::tie(manipulabilityXy, manipulabilityXz) = stiffness_controller.compute_X_compliance_ratios(jacobian3x4_mobile);
        const auto now_steady = std::chrono::steady_clock::now();

        //& MAPPING JOYSTICK
        twist_joy_world << airbus_joy->teleop_VelZ, airbus_joy->teleop_VelX, airbus_joy->teleop_VelY; 
        twist_joy_world[0] = bravo_utils::VAL_SAT<double>(twist_joy_world[0], MAX_SPEED_TELEOP, -MAX_SPEED_TELEOP);
        twist_joy_world[1] = bravo_utils::VAL_SAT<double>(twist_joy_world[1], MAX_SPEED_TELEOP, -MAX_SPEED_TELEOP);
        twist_joy_world[2] = bravo_utils::VAL_SAT<double>(twist_joy_world[2], MAX_SPEED_TELEOP, -MAX_SPEED_TELEOP);
        twist_joy_mobile = current_ee_rot.transpose() * twist_joy_world;
        CMD_ENABLE_TELEOP = airbus_joy->enableBaseMotion;
        CMD_GO_HOME       = airbus_joy->goHome;
        CMD_MAKE_READING  = airbus_joy->makeReading;
        
        //&MACHINE STATES
        switch(current_state) {
            case StateMachine::GO_HOME:
                if (prev_state != StateMachine::GO_HOME){     
                    prev_state = StateMachine::GO_HOME; 
                    arrived2HOME = false;
                    ALLOW_LEAVING_HOME = false;
                    LEAVE_HOME_FLAG = false;
                }
                desired_joint_pos = HOME;
                arrived2HOME = bravo->is_in_desired_configuration(0.2, desired_joint_pos, joint_postion_fdb);
                joint_velocity_cmd = Eigen::Vector4d::Zero();
                joint_velocity_friction = joint_velocity_fdb;
                dashboard_vel_ee_x = 0.0;
                dashboard_exerted_force_x = 0.0;
                break;

            case StateMachine::HOME_WAITING:
                if (prev_state != StateMachine::HOME_WAITING){    
                    prev_state = StateMachine::HOME_WAITING;                      
                }
                joint_velocity_cmd = Eigen::Vector4d::Zero();
                joint_velocity_friction = joint_velocity_fdb;
                dashboard_vel_ee_x = 0.0;
                dashboard_exerted_force_x = 0.0;
                break;

            case StateMachine::TELEOP: {
                if (prev_state != StateMachine::TELEOP){      
                    prev_state = StateMachine::TELEOP;
                    sim_joint_whole_integration = joint_postion_fdb;
                    sim_start_integration_time  = std::chrono::steady_clock::now();
                    sim_finish_integration_time = std::chrono::steady_clock::now();
                }
                const Eigen::Vector<double, 4> sim_joint_vel_integration = jacobian3x4_mobile.colPivHouseholderQr().solve(twist_joy_mobile); //! MAYBE BETTER USE DAMPING 
                sim_finish_integration_time = std::chrono::steady_clock::now();
                const std::chrono::duration<double> elapsed = sim_finish_integration_time - sim_start_integration_time;
                sim_joint_whole_integration = sim_joint_whole_integration + sim_joint_vel_integration * elapsed.count();
                sim_start_integration_time = std::chrono::steady_clock::now();
                desired_joint_pos = sim_joint_whole_integration;

                joint_velocity_cmd = sim_joint_vel_integration;
                joint_velocity_friction = sim_joint_vel_integration;
                dashboard_vel_ee_x = 0.0;
                dashboard_exerted_force_x = 0.0;
                break;
            }

            case StateMachine::STIFFNESS_CONTROL:
                if (prev_state != StateMachine::STIFFNESS_CONTROL){  
                    prev_state = StateMachine::STIFFNESS_CONTROL;
                    stiffness_controller.set_ref_ee_position(current_ee_pos); 
                }
                stiffness_controller.set_reference_integration_rotation(current_ee_rot);
                const Eigen::Vector3d output_vel_ee = stiffness_controller.get_vel_ee();
                const Eigen::Vector3d pos_error_mobile = current_ee_rot.transpose() * (stiffness_controller.get_ref_ee_position() - current_ee_pos);
                force_cmd = stiffness_controller.compute_force_action(
                            pos_error_mobile, 
                            output_vel_ee - jacobian3x4_mobile * joint_velocity_fdb, CMD_MAKE_READING);
                joint_torque_cmd = jacobian3x4_mobile.colPivHouseholderQr().solve(force_cmd); // ! MAYBE BETTER USE DAMPING
                mA_stiffness_current_cmd = bravo->torqueNm_2_currentmA(joint_torque_cmd);   

                joint_velocity_cmd = jacobian3x4_mobile.colPivHouseholderQr().solve(output_vel_ee);
                joint_velocity_friction = joint_velocity_fdb;    
                dashboard_vel_ee_x = CMD_MAKE_READING ? output_vel_ee[0] : 0.0;
                dashboard_exerted_force_x = force_cmd[0];
                break;
        }

        //& MACHINE STATES TRANSITIONS 
        if ((manipulability > MAX_MANIPULABILITY) || CMD_GO_HOME || !arrived2HOME) {
            current_state = StateMachine::GO_HOME;
        }
        else if ((manipulabilityXy < MAX_RATIO_FORCE_ELLIPSOID) || (manipulabilityXz < MAX_RATIO_FORCE_ELLIPSOID)) {
            current_state = StateMachine::GO_HOME;
        }
        else if (arrived2HOME && !ALLOW_LEAVING_HOME) {
            current_state = StateMachine::HOME_WAITING;
            if (!LEAVE_HOME_FLAG){
                    LEAVE_HOME_FLAG = true;
                    home_stable_since = now_steady;
            }
            const double home_stable_time =  std::chrono::duration<double>(now_steady - home_stable_since).count();
            if (home_stable_time >= HOME_STABLE_DWELL_SEC){
                ALLOW_LEAVING_HOME = true;
            }
        }
        else if (CMD_ENABLE_TELEOP){
            current_state = StateMachine::TELEOP;
        }
        else{
            current_state = StateMachine::STIFFNESS_CONTROL;
        }

        // & PID CONTROLLER
        double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_call_pd).count();
        Eigen::Vector4d error = bravo->signedAngleDistance(desired_joint_pos, joint_postion_fdb);
        last_call_pd = std::chrono::steady_clock::now();
        mA_pd_current_cmd[0] = j1_PID.compute(error[0], dt);
        mA_pd_current_cmd[1] = j2_PID.compute(error[1], dt);
        mA_pd_current_cmd[2] = j3_PID.compute(error[2], dt);
        mA_pd_current_cmd[3] = j4_PID.compute(error[3], dt);
        
        //& FRICTION COMPENSATION
        Nm_friction_compensation[0] = bravo->kinodynamics.computeFriction(joint_velocity_friction[0], FRICTION_MAT(0,0), FRICTION_MAT(0,1), FRICTION_MAT(0,2), FRICTION_MAT(0,3), FRICTION_MAT(0,4), FRICTION_MAT(0,5), FRICTION_SATURATION);
        Nm_friction_compensation[1] = bravo->kinodynamics.computeFriction(joint_velocity_friction[1], FRICTION_MAT(1,0), FRICTION_MAT(1,1), FRICTION_MAT(1,2), FRICTION_MAT(1,3), FRICTION_MAT(1,4), FRICTION_MAT(1,5), FRICTION_SATURATION);
        Nm_friction_compensation[2] = bravo->kinodynamics.computeFriction(joint_velocity_friction[2], FRICTION_MAT(2,0), FRICTION_MAT(2,1), FRICTION_MAT(2,2), FRICTION_MAT(2,3), FRICTION_MAT(2,4), FRICTION_MAT(2,5), FRICTION_SATURATION);
        Nm_friction_compensation[3] = bravo->kinodynamics.computeFriction(joint_velocity_friction[3], FRICTION_MAT(3,0), FRICTION_MAT(3,1), FRICTION_MAT(3,2), FRICTION_MAT(3,3), FRICTION_MAT(3,4), FRICTION_MAT(3,5), FRICTION_SATURATION);
        mA_friction_compensation = bravo->torqueNm_2_currentmA(Nm_friction_compensation);
 
        //& RIGID BODY DYNAMICS
        Nm_gravity = bravo->kinodynamics.invDynamics(joint_postion_fdb, joint_velocity_cmd, Eigen::VectorXd::Zero(4));
        Nm_gravity_compensation = bravo->kinodynamics.invDynamics(joint_postion_fdb, Eigen::VectorXd::Zero(4), Eigen::VectorXd::Zero(4));
        
        //& COMPOSE JOINT COMMAND
        if (prev_state == StateMachine::STIFFNESS_CONTROL){
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(Nm_gravity).array() + mA_friction_compensation.array() + mA_stiffness_current_cmd.array();
        }
        else{
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(Nm_gravity).array() + mA_friction_compensation.array() + mA_pd_current_cmd.array();
        }
        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA); 
        
        //& DASHBOARD UPDATE
        if (dashboard) {
            dashboard->setInteractionStatusX(
                dashboard_vel_ee_x,
                dashboard_desired_force_x,
                dashboard_exerted_force_x);
            dashboard->setArmStatus(
                state_machine_to_string(current_state),
                manipulability,
                bravo->isConnected(),
                bravo->get_udp_rx_frequency_hz());
        }
        //& JOINT STATE PUBLISHER
        joint_state_msg.header.stamp = joint_state_node->now();
        for (int i = 0; i < 4; ++i) {
            joint_state_msg.position[i] = joint_postion_fdb[i];
            joint_state_msg.velocity[i] = joint_velocity_fdb[i];
        }
        joint_state_pub->publish(joint_state_msg);
        loop_rate.sleep();
    }
    // & SAFETY: STOP THE ARM WITH ONLY GRAVITY COMPENSATION
    mA_joint_current_cmd = bravo->torqueNm_2_currentmA(Nm_gravity_compensation).array();
    bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);
    return true;
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        const std::string package_path = ament_index_cpp::get_package_share_directory("bravo_cathode_protection");
        const std::string config_stiff_control_file = (package_path + "/config/stiff_control_params/bravo5_cp_compliance_mobile_axis.yaml");
        const std::string config_runtime_file       = (package_path + "/config/program_params/bravo5_cp_mobile_axis_runtime.yaml");
        const std::string config_friction_file      = (package_path + "/config/joint_friction_params/joint_friction_bravo5.yaml");
        const std::string urdf_filename             = (package_path + "/urdf/bravo_5_dynamics_pinocchio_cp.urdf");
        auto shared_logger = std::make_shared<bravo_utils::Logger>(bravo_utils::write_log_stderr);
        bravo_utils::RuntimeConfig runtime_config;
        bravo_utils::FrictionConfig friction_config;
        //& PARAMETERS LOADED FROM YAML CONFIG FILE
        stiffness_control_config::StiffnessParams stiff_params;
        try {
            stiff_params = stiffness_control_config::load_stiffness_params_yaml(config_stiff_control_file);
            runtime_config = bravo_utils::load_runtime_config_yaml(config_runtime_file);
            friction_config = bravo_utils::load_friction_config_yaml(config_friction_file);
        }
        catch (const std::exception& e) {
            BRAVO_LOG_ERROR(*shared_logger, "[yaml config] ", e.what());
            rclcpp::shutdown();
            return 1;
        }
        //& EXTERNAL LOGGER + DASHBOARD SINK (owned by main)
        auto dashboard = std::make_shared<bravo_utils::TerminalDashboard>();
        shared_logger->set_callback(bravo_utils::make_dashboard_or_stderr_callback(
            [dashboard]() {
                return dashboard->is_enabled();
            },
            [dashboard](bravo_utils::LogLevel level, std::string_view message) {
                dashboard->push(level, std::string(message));
            }));
        BRAVO_LOG_INFO(*shared_logger, "[main] Bravo UDP target: ", runtime_config.ip_address, ":", runtime_config.udp_port);
        BRAVO_LOG_INFO(*shared_logger, "[main] Using ee frame: ", runtime_config.tool_link);

        auto joystick         = std::make_shared<airbus_joystick_bravo5_CP>();
        auto joint_state_node = std::make_shared<rclcpp::Node>("bravo5_cp_mobile_axis_joint_state_pub");
        auto joint_state_pub  = joint_state_node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        auto bravo            = std::make_shared<bravo_handler<double>>(urdf_filename, runtime_config.tool_link, bravo_control::ArmModel::bravo5, runtime_config.ip_address, runtime_config.udp_port, shared_logger);//(urdf_filename);
        auto executor         = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();        
        executor->add_node(joystick);
        executor->add_node(joint_state_node);
        std::thread executor_thread([&executor]() {
                executor->spin();
        });
        const bool program_ok = program_loop(joystick, bravo, stiff_params, runtime_config, friction_config, joint_state_pub, joint_state_node, dashboard, shared_logger, runtime_config.ip_address, runtime_config.udp_port);
        executor->cancel();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        executor_thread.join();
        if (!program_ok) {
            return 1;
        }
        return 0;
}
