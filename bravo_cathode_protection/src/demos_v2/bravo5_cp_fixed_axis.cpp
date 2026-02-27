/**
 *    @file  bravo5_cp_fixed_axis.cpp
 *    @brief Program to make interaction with Bravo arms. Design for cathode-protection tasks
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
#include "pinocchio/parsers/urdf.hpp" 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "bravo_cathode_protection/bravo_cpp/bravo_handler/bravo_handler_v2.h"
#include "bravo_cathode_protection/bravo_cpp/utils/bravo_dashboard.h"
#include "bravo_cathode_protection/bravo_cpp/utils/bravo_logger.h"
#include "general_libs_unite/joysticks/airbus_joystick.h"
#include "general_libs_unite/interaction/stiffness_control_position.h"

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
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
                  const std::shared_ptr<bravo_utils::TerminalDashboard>& dashboard,
                  const std::shared_ptr<bravo_utils::Logger>& logger,
                  const std::string& ip_address,
                  int udp_port){     
    const auto active_logger = logger ? logger : std::make_shared<bravo_utils::Logger>();

    //& PARAMETERS
    const double MAX_CURRENT_mA = 2000.0;
    const double MAX_CURRENT_mA_GO_HOME = 1000.0;
    const double MAX_MANIPULABILITY = 6.0;
    const double MAX_MANIPULABILITY_RELEASE_HOME = 5.7;
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
    bool reading_latched = false;

    //& VARIABLES DIFF KINEMATICS
    std::chrono::high_resolution_clock::time_point sim_finish_integration_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point sim_start_integration_time  = std::chrono::high_resolution_clock::now();
    Eigen::Vector<double, 4> sim_joint_whole_integration = HOME;

    //& PID CONTROLLER
    PID<double> j1_PID(3500.0, 0.0, 200.0, -MAX_CURRENT_mA_GO_HOME, MAX_CURRENT_mA_GO_HOME); // P, I, D gains and output limits
    PID<double> j2_PID(3500.0, 0.0, 200.0, -MAX_CURRENT_mA_GO_HOME, MAX_CURRENT_mA_GO_HOME); // P, I, D gains and output limits
    PID<double> j3_PID(3500.0, 0.0, 200.0, -MAX_CURRENT_mA_GO_HOME, MAX_CURRENT_mA_GO_HOME); // P, I, D gains and output limits
    PID<double> j4_PID(3500.0, 0.0, 200.0, -MAX_CURRENT_mA_GO_HOME, MAX_CURRENT_mA_GO_HOME); // P, I, D gains and output limits
    std::chrono::high_resolution_clock::time_point last_call_pd = std::chrono::high_resolution_clock::now();

    //& STIFFNESS CONTROLLER
    stiffness_control_position <double> stiffness_controller;
    
    //& STATE MACHINE VARIABLES
    StateMachine prev_state    = StateMachine::GO_HOME;
    StateMachine current_state = StateMachine::GO_HOME;
    bool arrived2HOME = false;
    bool force_go_home = true;
    bool home_stability_timer_running = false;
    std::chrono::steady_clock::time_point home_stable_since = std::chrono::steady_clock::now();
    Eigen::Vector4d desired_joint_pos = HOME; 

    //& AUX VARIABLES
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
    bool   motion_teleop   = false;
    bool   perform_reading = false;
    
    //! GRAVITY VECTOR FOR ARM MOUNTING POINT
    bravo->kinodynamics.change_gravity_vector(GRAVITY_VECTOR); //!changing gravity compensation
    bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests  
    auto start_time = std::chrono::steady_clock::now();
    if (dashboard) {
        if (dashboard) {
            dashboard->setArmStatus("CONNECTING", 0.0, true, 0.0);
        }
    }
    //& ARM CONNECTION
    while (!bravo->isConnected()){        
        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: no UDP feedback from Bravo after 5 seconds (target "
                      << ip_address << ":" << udp_port << ").\n"
                      << "Check network routing.\n";
            if (dashboard) {
                dashboard->setArmStatus("DISCONNECTED", manipulability, false, 0.0);
            }
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }    

    //$ STIFFNESS CONTROLLER PARAMETERS
    Eigen::Vector4d ref_joint_pos = bravo->get_bravo_joint_states();
    stiffness_controller.set_pos_stiffness(stiff_params.pos_stiffness);
    stiffness_controller.set_pos_damping(stiff_params.pos_damping);
    stiffness_controller.set_gain_force(stiff_params.gain_force);
    stiffness_controller.set_nominal_vel(stiff_params.nominal_vel);
    stiffness_controller.set_desired_force(stiff_params.desired_force);
    stiffness_controller.set_max_vel(stiff_params.maximum_vel);
    stiffness_controller.set_ref_ee_position(bravo->kinodynamics.FK_ee_pos(ref_joint_pos));

    double LOOP_FREQUENCY = 250.0;  // Hz (Matching the arm frequency)
    rclcpp::Rate loop_rate(LOOP_FREQUENCY);  

    //& ------MAIN LOOP--------
    while (rclcpp::ok()) {
        joint_postion_fdb  = bravo->get_bravo_joint_states();
        joint_velocity_fdb = bravo->get_bravo_joint_velocities();
        jacobian3x4 = bravo->kinodynamics.fixedJacobian(joint_postion_fdb).topRows(3);
        manipulability = bravo->compute_manipulability_position();
        std::tie(manipulabilityXy, manipulabilityXz) = stiffness_controller.compute_X_compliance_ratios(jacobian3x4);

        //& MAPPING JOYSTICK
        twist_joy_fixed << airbus_joy->teleop_VelZ, airbus_joy->teleop_VelX, airbus_joy->teleop_VelY; 
        twist_joy_fixed[0] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[0], MAX_SPEED_JOY, -MAX_SPEED_JOY);
        twist_joy_fixed[1] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[1], MAX_SPEED_JOY, -MAX_SPEED_JOY);
        twist_joy_fixed[2] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[2], MAX_SPEED_JOY, -MAX_SPEED_JOY);
        motion_teleop = airbus_joy->enableBaseMotion;
        perform_reading = airbus_joy->makeReading;
        const auto now_steady = std::chrono::steady_clock::now();

        //& MACHINE STATES (legacy one-pass state branching)
        if ((manipulability > MAX_MANIPULABILITY) || airbus_joy->goHome || !arrived2HOME) {
            force_go_home = true;
        }

        if (force_go_home){
            if (prev_state != StateMachine::GO_HOME){      
            }
            desired_joint_pos = HOME;
            const bool arm_at_home = bravo->is_in_desired_configuration(0.2, desired_joint_pos, joint_postion_fdb);
            const bool can_exit_go_home = arm_at_home && !airbus_joy->goHome &&
                                          (manipulability < MAX_MANIPULABILITY_RELEASE_HOME);
            if (can_exit_go_home) {
                if (!home_stability_timer_running) {
                    home_stability_timer_running = true;
                    home_stable_since = now_steady;
                }
                const double home_stable_time =
                    std::chrono::duration<double>(now_steady - home_stable_since).count();
                if (home_stable_time >= HOME_STABLE_DWELL_SEC) {
                    arrived2HOME = true;
                    force_go_home = false;
                    home_stability_timer_running = false;
                } else {
                    arrived2HOME = false;
                }
            } else {
                arrived2HOME = false;
                home_stability_timer_running = false;
            }
            joint_velocity_friction = joint_velocity_fdb;
            joint_velocity_cmd = Eigen::Vector4d::Zero();
            reading_latched = false;
            dashboard_vel_ee_x = 0.0;
            dashboard_exerted_force_x = 0.0;
            prev_state = StateMachine::GO_HOME;
            current_state = StateMachine::GO_HOME;
        }
        else if ((!reading_latched) && (!motion_teleop)){
            if (prev_state != StateMachine::HOME_WAITING){      
            }
            if (perform_reading){
                reading_latched = true;
            }
            dashboard_vel_ee_x = 0.0;
            dashboard_exerted_force_x = 0.0;
            prev_state = StateMachine::HOME_WAITING;
            current_state = StateMachine::HOME_WAITING;
        }
        else if (motion_teleop){
            if (prev_state != StateMachine::TELEOP){      
                sim_joint_whole_integration = bravo->get_bravo_joint_states();
                sim_start_integration_time  = std::chrono::high_resolution_clock::now();
                sim_finish_integration_time = std::chrono::high_resolution_clock::now();
            }
            jacobian3x4 = bravo->kinodynamics.fixedJacobian(sim_joint_whole_integration).topRows(3);
            const Eigen::Vector<double, 4> sim_joint_vel_integration = jacobian3x4.colPivHouseholderQr().solve(twist_joy_fixed); //! MAYBE BETTER USE DAMPING 
            joint_velocity_cmd = sim_joint_vel_integration;
            sim_finish_integration_time = std::chrono::high_resolution_clock::now();
            const std::chrono::duration<double> elapsed = sim_finish_integration_time - sim_start_integration_time;
            sim_joint_whole_integration = sim_joint_whole_integration + sim_joint_vel_integration * elapsed.count();
            sim_start_integration_time = std::chrono::high_resolution_clock::now();
            desired_joint_pos = sim_joint_whole_integration;
            joint_velocity_friction = sim_joint_vel_integration;
            dashboard_vel_ee_x = 0.0;
            dashboard_exerted_force_x = 0.0;
            prev_state = StateMachine::TELEOP;
            current_state = StateMachine::TELEOP;
        }
        else{
            if (prev_state != StateMachine::STIFFNESS_CONTROL){       
                stiffness_controller.set_ref_ee_position(bravo->kinodynamics.FK_ee_pos(bravo->get_bravo_joint_states()));
            }
            const Eigen::Vector3d output_vel_ee = stiffness_controller.get_vel_ee();
            force_cmd = stiffness_controller.compute_force_action(
                        stiffness_controller.get_ref_ee_position() - bravo->kinodynamics.FK_ee_pos(bravo->get_bravo_joint_states()), 
                        output_vel_ee - jacobian3x4 * joint_velocity_fdb, perform_reading);
            joint_velocity_cmd = jacobian3x4.colPivHouseholderQr().solve(output_vel_ee);
            joint_velocity_friction = bravo->get_bravo_joint_velocities();
            joint_torque_cmd = jacobian3x4.colPivHouseholderQr().solve(force_cmd); // ! MAYBE BETTER USE DAMPING
            mA_stiffness_current_cmd = bravo->torqueNm_2_currentmA(joint_torque_cmd);          
            dashboard_vel_ee_x = output_vel_ee[0];
            dashboard_exerted_force_x = force_cmd[0];
            if ((manipulabilityXy < MAX_RATIO_FORCE_ELLIPSOID) || (manipulabilityXz < MAX_RATIO_FORCE_ELLIPSOID)) {
                arrived2HOME = false;
                force_go_home = true;
                home_stability_timer_running = false;
            }
            last_call_pd = std::chrono::high_resolution_clock::now();
            prev_state = StateMachine::STIFFNESS_CONTROL;
            current_state = StateMachine::STIFFNESS_CONTROL;
        }

        // & PID CONTROLLER
        double dt = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - last_call_pd).count();
        Eigen::Vector4d error = bravo->signedAngleDistance(desired_joint_pos, bravo->get_bravo_joint_states());
        last_call_pd = std::chrono::high_resolution_clock::now();
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
        Nm_gravity = bravo->kinodynamics.invDynamics(bravo->get_bravo_joint_states(), joint_velocity_cmd, Eigen::VectorXd::Zero(4));
        Nm_gravity_compensation = bravo->kinodynamics.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(4), Eigen::VectorXd::Zero(4));
        
        //& COMPOSE JOINT COMMAND
        if (prev_state == StateMachine::STIFFNESS_CONTROL){
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(Nm_gravity).array() + mA_friction_compensation.array() + mA_stiffness_current_cmd.array();
        }
        else{
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(Nm_gravity).array() + mA_friction_compensation.array() + mA_pd_current_cmd.array();
        }
        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA); 

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
        bravo->publish_bravo_joint_states();
        loop_rate.sleep();
    }
    // & SAFETY: STOP THE ARM WITH ONLY GRAVITY COMPENSATION
    mA_joint_current_cmd = bravo->torqueNm_2_currentmA(Nm_gravity_compensation).array();
    bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);
    bravo->publish_bravo_joint_states();
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        const std::string package_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().string();
        const std::string config_filename = (package_path + "/config/bravo5_cp_compliance_fixed_axis.json");
        const std::string urdf_filename   = (package_path + "/urdf/bravo_5_dynamics_pinocchio.urdf");
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

        auto joystick         = std::make_shared<airbus_joystick_bravo5_CP>();
        auto bravo            = std::make_shared<bravo_handler<double>>(urdf_filename, tool_link, ip_address, udp_port, true, shared_logger);//(urdf_filename);
        auto executor         = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();        
        executor->add_node(joystick);
        std::thread executor_thread([&executor]() {
                executor->spin();
        });
        program_loop(joystick, bravo, stiff_params, dashboard, shared_logger, ip_address, udp_port);
        executor->cancel();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        executor_thread.join();
        rclcpp::shutdown();
        return 0;
}
