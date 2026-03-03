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
#include "pinocchio/fwd.hpp"

#include "bravo_cathode_protection/bravo_cpp/bravo_handler/bravo_handler_v2.h"
#include "bravo_cathode_protection/bravo_cpp/utils/bravo_dashboard.h"
#include "bravo_cathode_protection/bravo_cpp/utils/bravo_logger.h"
#include "general_libs_unite/joysticks/airbus_joystick.h"
#include "general_libs_unite/interaction/stiffness_control_position.h"

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stdexcept>
#include <filesystem>
#include <thread>

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

void program_loop(std::shared_ptr<airbus_joystick_bravo5_CP> airbus_joy,
                  std::shared_ptr<bravo_handler<double>> bravo,
                  const stiffness_control_config::StiffnessJsonParams& stiff_params,
                  const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& joint_state_pub,
                  const std::shared_ptr<rclcpp::Node>& joint_state_node,
                  const std::shared_ptr<bravo_utils::TerminalDashboard>& dashboard,
                  const std::shared_ptr<bravo_utils::Logger>& logger,
                  const std::string& ip_address,
                  int udp_port){     
    const auto active_logger = logger ? logger : std::make_shared<bravo_utils::Logger>(bravo_utils::write_log_stderr);

    //& PARAMETERS
    const double MAX_CURRENT_mA = 2000.0;
    const double MAX_CURRENT_mA_GO_HOME = 1000.0;
    const double MAX_MANIPULABILITY = 6.0;
    const double HOME_STABLE_DWELL_SEC = 0.35;
    const double MAX_RATIO_FORCE_ELLIPSOID = 0.3;
    const double MAX_SPEED_JOY = 0.25; // m/s
    const Eigen::Vector<double, 4> HOME = (Eigen::Vector<double, 4>() << 3.14, 2.706, 0.946, 0.0).finished(); //! define home for bravo5
    const Eigen::Vector3d GRAVITY_VECTOR = (Eigen::Vector3d() << 0.0, 0.0, -9.81).finished();
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
    double manipulability = 0.0;
    bool   CMD_ENABLE_TELEOP = false;
    bool   CMD_GO_HOME       = false;
    bool   CMD_MAKE_READING  = false;
    bool   MAKE_CP_READING   = false;
    bool   ALLOW_LEAVING_HOME = false;
    bool   LEAVE_HOME_FLAG    = false;
    double LOOP_FREQUENCY = 250.0;  // Hz (Matching the arm frequency)
    Eigen::Matrix3d ref_ee_rot, current_ee_rot;

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
            return;
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
    stiffness_controller.set_ref_ee_position(ref_ee_pos_init);
    stiffness_controller.set_reference_integration_local(true);
    Eigen::Vector4d ref_joint_pos = bravo->get_bravo_joint_states();
    Eigen::Vector3d ref_ee_pos_init;
    std::tie(ref_ee_pos_init, ref_ee_rot) = bravo->kinodynamics.FK_ee(ref_joint_pos);
    stiffness_controller.set_reference_integration_rotation(ref_ee_rot);

    rclcpp::Rate loop_rate(LOOP_FREQUENCY);  

    //& ------MAIN LOOP--------
    while (rclcpp::ok()) {
        //& RECURRENT CALCULATIONS
        joint_postion_fdb  = bravo->get_bravo_joint_states();
        joint_velocity_fdb = bravo->get_bravo_joint_velocities();
        jacobian3x4        = bravo->kinodynamics.localJacobian(joint_postion_fdb).topRows(3);
        manipulability     = bravo->compute_manipulability_position();
        std::tie(manipulabilityXy, manipulabilityXz) = stiffness_controller.compute_X_compliance_ratios(jacobian3x4);
        const auto now_steady = std::chrono::steady_clock::now();

        //& MAPPING JOYSTICK
        twist_joy_fixed << airbus_joy->teleop_VelZ, airbus_joy->teleop_VelX, airbus_joy->teleop_VelY; 
        twist_joy_fixed[0] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[0], MAX_SPEED_JOY, -MAX_SPEED_JOY);
        twist_joy_fixed[1] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[1], MAX_SPEED_JOY, -MAX_SPEED_JOY);
        twist_joy_fixed[2] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[2], MAX_SPEED_JOY, -MAX_SPEED_JOY);
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
                Eigen::Vector3d current_ee_pos;
                std::tie(current_ee_pos, current_ee_rot) = bravo->kinodynamics.FK_ee(joint_postion_fdb);
                if (prev_state != StateMachine::STIFFNESS_CONTROL){  
                    prev_state = StateMachine::STIFFNESS_CONTROL;
                    stiffness_controller.set_ref_ee_position(current_ee_pos); 
                }
                stiffness_controller.set_reference_integration_rotation(current_ee_rot);
                const Eigen::Vector3d output_vel_ee = stiffness_controller.get_vel_ee();
                const Eigen::Vector3d pos_error_mobile = current_ee_rot.transpose() * (stiffness_controller.get_ref_ee_position() - current_ee_pos);
                force_cmd = stiffness_controller.compute_force_action(
                            pos_error_mobile, 
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
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        std::string tool_link = "cp_probe_1";
        const std::string package_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().string();
        const std::string config_filename = (package_path + "/config/bravo5_cp_compliance_mobile_axis.json");
        const std::string urdf_filename   = (package_path + "/urdf/bravo_5_dynamics_pinocchio_cp.urdf");
        const std::string tool_link  = std::string("contact_point");
        const std::string ip_address = std::string("10.43.0.146");
        const int udp_port = 6789;
        auto shared_logger = std::make_shared<bravo_utils::Logger>(bravo_utils::write_log_stderr);
        //& PARAMETERS LOADED FROM JSON CONFIG FILE
        stiffness_control_config::StiffnessJsonParams stiff_params;
        try {
            stiff_params = stiffness_control_config::load_stiffness_params_json(config_filename);
        }
        catch (const std::exception& e) {
            BRAVO_LOG_ERROR(*shared_logger, "[stiffness json] ", e.what());
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
        BRAVO_LOG_INFO(*shared_logger, "[main] Bravo UDP target: ", ip_address, ":", udp_port);
        BRAVO_LOG_INFO(*shared_logger, "[main] Using ee frame: ", tool_link);

        auto joystick         = std::make_shared<airbus_joystick_bravo5_CP>();
        auto joint_state_node = std::make_shared<rclcpp::Node>("bravo5_cp_mobile_axis_switch_joint_state_pub");
        auto joint_state_pub  = joint_state_node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        auto bravo            = std::make_shared<bravo_handler<double>>(urdf_filename, tool_link, bravo_control::ArmModel::bravo5, ip_address, udp_port, shared_logger);//(urdf_filename);
        auto executor         = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();        
        executor->add_node(joystick);
        executor->add_node(joint_state_node);
        std::thread executor_thread([&executor]() {
                executor->spin();
        });
        program_loop(joystick, bravo, stiff_params, joint_state_pub, joint_state_node, dashboard, shared_logger, ip_address, udp_port);
        executor->cancel();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        executor_thread.join();
        rclcpp::shutdown();
        return 0;
}
