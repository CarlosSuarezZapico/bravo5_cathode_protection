/**
 *    @file  bravo5_cp_fixed_axis.cpp
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

#include <iostream>
#include "pinocchio/fwd.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stdexcept>
#include <filesystem>
#include <thread>
#include <fstream>

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

struct RuntimeConfig
{
    std::string ip_address = "10.43.0.146";
    int udp_port = 6789;
    Eigen::Vector4d home = (Eigen::Vector4d() << 3.14, 2.857, 1.362, 0.0).finished();
    Eigen::Vector3d gravity_vector = (Eigen::Vector3d() << 0.0, 0.0, 9.81).finished();
    double max_current_mA = 2000.0;
    double max_current_mA_go_home = 1000.0;
    double max_manipulability = 6.0;
    double max_ratio_force_ellipsoid = 0.3;
    double max_speed_teleop = 0.25;
    double loop_frequency = 250.0;
    std::string tool_link = "contact_point";
};

static Eigen::Vector4d json_vec4(const nlohmann::json& j, const std::string& key)
{
    if (!j.contains(key) || !j.at(key).is_array() || j.at(key).size() != 4) {
        throw std::runtime_error("JSON key '" + key + "' must be an array of 4 numbers");
    }
    return Eigen::Vector4d(
        j.at(key).at(0).get<double>(),
        j.at(key).at(1).get<double>(),
        j.at(key).at(2).get<double>(),
        j.at(key).at(3).get<double>());
}

static Eigen::Vector3d json_vec3(const nlohmann::json& j, const std::string& key)
{
    if (!j.contains(key) || !j.at(key).is_array() || j.at(key).size() != 3) {
        throw std::runtime_error("JSON key '" + key + "' must be an array of 3 numbers");
    }
    return Eigen::Vector3d(
        j.at(key).at(0).get<double>(),
        j.at(key).at(1).get<double>(),
        j.at(key).at(2).get<double>());
}

static RuntimeConfig load_runtime_config_json(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open runtime config file: " + path);
    }

    nlohmann::json j;
    file >> j;

    RuntimeConfig cfg;
    cfg.ip_address = j.at("IP").get<std::string>();
    cfg.udp_port = j.at("port").get<int>();
    cfg.home = json_vec4(j, "HOME");
    cfg.gravity_vector = json_vec3(j, "GRAVITY_VECTOR");
    cfg.max_current_mA = j.at("MAX_CURRENT_mA").get<double>();
    cfg.max_current_mA_go_home = j.at("MAX_CURRENT_mA_GO_HOME").get<double>();
    cfg.max_manipulability = j.at("MAX_MANIPULABILITY").get<double>();
    cfg.max_ratio_force_ellipsoid = j.at("MAX_RATIO_FORCE_ELLIPSOID").get<double>();
    cfg.max_speed_teleop = j.at("MAX_SPEED_TELEOP").get<double>();
    cfg.loop_frequency = j.at("LOOP_FREQUENCY").get<double>();
    cfg.tool_link = j.at("tool_link").get<std::string>();

    return cfg;
}

bool program_loop(std::shared_ptr<airbus_joystick_bravo5_CP> airbus_joy,
                  std::shared_ptr<bravo_handler<double>> bravo,
                  const stiffness_control_config::StiffnessJsonParams& stiff_params,
                  const RuntimeConfig& runtime_config,
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
    //Joint friction compensation parameters
    Eigen::Matrix<double,4,6> FRICTION_MAT;
    FRICTION_MAT << 0.0, 0.0, 0.0, 14.66651, 10.09561,  1.25110,  // Joint 1
                    0.0, 0.0, 0.0, 14.50959, 10.09275,  1.14381,  // Joint 2
                    0.0, 0.0, 0.0, 13.98409, 12.97659, 21.69165,  // Joint 3
                    0.0, 0.0, 0.0, 15.82785, 14.35639, 24.04595;  // Joint 4
    const double FRICTION_SATURATION = 60; 

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
    Eigen::MatrixXd jacobian3x4              = Eigen::MatrixXd::Zero(3,4);
    Eigen::Vector3d twist_joy_fixed          = Eigen::Vector3d::Zero();
    double manipulabilityXy = 0.0;
    double manipulabilityXz = 0.0;
    double manipulability   = 0.0;
    bool   CMD_ENABLE_TELEOP  = false;
    bool   CMD_GO_HOME        = false;
    bool   CMD_MAKE_READING   = false;
    bool   ALLOW_LEAVING_HOME = false;
    bool   LEAVE_HOME_FLAG    = false;

    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.name = {"joint1", "joint2", "joint3", "joint4"};
    joint_state_msg.position.resize(4, 0.0);
    joint_state_msg.velocity.resize(4, 0.0);
    
    //! GRAVITY VECTOR FOR ARM MOUNTING POINT
    bravo->kinodynamics.change_gravity_vector(GRAVITY_VECTOR); //!changing gravity compensation
    bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests  
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
    Eigen::Vector4d ref_joint_pos = bravo->get_bravo_joint_states();
    stiffness_controller.set_ref_ee_position(bravo->kinodynamics.FK_ee_pos(ref_joint_pos));

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
        jacobian3x4        = bravo->kinodynamics.fixedJacobian(joint_postion_fdb).topRows(3);
        manipulability     = bravo->compute_manipulability_position();
        std::tie(manipulabilityXy, manipulabilityXz) = stiffness_controller.compute_X_compliance_ratios(jacobian3x4);
        const auto now_steady = std::chrono::steady_clock::now();

        //& MAPPING JOYSTICK
        twist_joy_fixed << airbus_joy->teleop_VelZ, airbus_joy->teleop_VelX, airbus_joy->teleop_VelY; 
        twist_joy_fixed[0] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[0], MAX_SPEED_TELEOP, -MAX_SPEED_TELEOP);
        twist_joy_fixed[1] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[1], MAX_SPEED_TELEOP, -MAX_SPEED_TELEOP);
        twist_joy_fixed[2] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[2], MAX_SPEED_TELEOP, -MAX_SPEED_TELEOP);
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
                const Eigen::Vector<double, 4> sim_joint_vel_integration = jacobian3x4.colPivHouseholderQr().solve(twist_joy_fixed); //! MAYBE BETTER USE DAMPING 
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
                    stiffness_controller.set_ref_ee_position(bravo->kinodynamics.FK_ee_pos(joint_postion_fdb));  
                }
                const Eigen::Vector3d output_vel_ee = stiffness_controller.get_vel_ee();
                force_cmd = stiffness_controller.compute_force_action(
                            stiffness_controller.get_ref_ee_position() - bravo->kinodynamics.FK_ee_pos(joint_postion_fdb), 
                            output_vel_ee - jacobian3x4 * joint_velocity_fdb, CMD_MAKE_READING);
                joint_torque_cmd = jacobian3x4.colPivHouseholderQr().solve(force_cmd); // ! MAYBE BETTER USE DAMPING
                mA_stiffness_current_cmd = bravo->torqueNm_2_currentmA(joint_torque_cmd);   

                joint_velocity_cmd = jacobian3x4.colPivHouseholderQr().solve(output_vel_ee);
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
        const std::string package_path = std::filesystem::path(__FILE__).parent_path().parent_path().string();
        const std::string config_stiff_control_file = (package_path + "/config/stiff_control_params/bravo5_cp_compliance_fixed_axis.json");
        const std::string config_runtime_file       = (package_path + "/config/program_params/bravo5_cp_fixed_axis_runtime.json");
        const std::string urdf_filename             = (package_path + "/urdf/bravo_5_dynamics_pinocchio_cp.urdf");
        auto shared_logger = std::make_shared<bravo_utils::Logger>(bravo_utils::write_log_stderr);
        RuntimeConfig runtime_config;
        //& PARAMETERS LOADED FROM JSON CONFIG FILE
        stiffness_control_config::StiffnessJsonParams stiff_params;
        try {
            stiff_params = stiffness_control_config::load_stiffness_params_json(config_stiff_control_file);
            runtime_config = load_runtime_config_json(config_runtime_file);
        }
        catch (const std::exception& e) {
            BRAVO_LOG_ERROR(*shared_logger, "[json config] ", e.what());
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
        auto joint_state_node = std::make_shared<rclcpp::Node>("bravo5_cp_fixed_axis_joint_state_pub");
        auto joint_state_pub  = joint_state_node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        auto bravo            = std::make_shared<bravo_handler<double>>(urdf_filename, runtime_config.tool_link, bravo_control::ArmModel::bravo5, runtime_config.ip_address, runtime_config.udp_port, shared_logger);//(urdf_filename);
        auto executor         = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();        
        executor->add_node(joystick);
        executor->add_node(joint_state_node);
        std::thread executor_thread([&executor]() {
                executor->spin();
        });
        const bool program_ok = program_loop(joystick, bravo, stiff_params, runtime_config, joint_state_pub, joint_state_node, dashboard, shared_logger, runtime_config.ip_address, runtime_config.udp_port);
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
