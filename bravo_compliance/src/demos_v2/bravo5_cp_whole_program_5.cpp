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
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

enum class StateMachine{
    GO_HOME,
    HOME_WAITING,
    JOYSTICK_TELEOP,
    STIFFNESS_CONTROL
};

static Eigen::Vector3d json_vec3(const nlohmann::json& j, const std::string& key)
{
    if (!j.contains(key) || !j.at(key).is_array() || j.at(key).size() != 3) {
        throw std::runtime_error("JSON key '" + key + "' must be an array of 3 numbers");
    }
    return Eigen::Vector3d(
        j.at(key).at(0).get<double>(),
        j.at(key).at(1).get<double>(),
        j.at(key).at(2).get<double>()
    );
}

struct StiffnessJsonParams
{
    Eigen::Vector3d pos_stiffness{50.0, 1000.0, 200.0};
    Eigen::Vector3d pos_damping{1.0, 100.0, 20.0};
    Eigen::Vector3d gain_force{0.03, 0.0, 0.0};
    Eigen::Vector3d nominal_vel{0.05, 0.0, 0.0};
    Eigen::Vector3d desired_force{1.5, 0.0, 0.0};
    Eigen::Vector3d maximum_vel{0.25, 0.25, 0.25};
};

static StiffnessJsonParams load_stiffness_params_json(const std::string& path)
{
    StiffnessJsonParams p; // defaults
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Could not open stiffness config file: " + path);
    }
    nlohmann::json j;
    f >> j;
    p.pos_stiffness = json_vec3(j, "pos_stiffness");
    p.pos_damping   = json_vec3(j, "pos_damping");
    p.gain_force    = json_vec3(j, "gain_force");
    p.nominal_vel   = json_vec3(j, "nominal_vel");
    p.desired_force = json_vec3(j, "desired_force");
    p.maximum_vel   = json_vec3(j, "maximum_vel");
    return p;
}

void program_loop(std::shared_ptr<airbus_joystick_bravo5_CP> airbus_joy, std::shared_ptr<bravo_handler<double>> bravo, const StiffnessJsonParams& stiff_params){     
    //& PARAMETERS
    double MAX_CURRENT_mA = 2000.0;
    double MAX_CURRENT_mA_GO_HOME = 1000.0;
    double MAX_MANIPULABILITY = 6.0;
    double MAX_RATIO_FORCE_ELLIPSOID = 0.3;
    double MAX_SPEED_JOY = 0.25; // m/s
    Eigen::Vector<double, 4> HOME = (Eigen::Vector<double, 4>() << 3.14, 2.706, 0.946, 0.0).finished(); //! define home for bravo5
    Eigen::Vector3d GRAVITY_VECTOR = (Eigen::Vector3d() << 0.0, 0.0, -9.81).finished();
    Eigen::Matrix<double,4,6> FRICTION_MAT;
    FRICTION_MAT << 0.0, 0.0, 0.0, 14.66651, 10.09561,  1.25110,  // Joint 1
                    0.0, 0.0, 0.0, 14.50959, 10.09275,  1.14381,  // Joint 2
                    0.0, 0.0, 0.0, 13.98409, 12.97659, 21.69165,  // Joint 3
                    0.0, 0.0, 0.0, 15.82785, 14.35639, 24.04595;  // Joint 4
    double FRICTION_SATURATION = 60; 
    bool DEBUG = true;
    bool MAKE_READING = false;
    std::chrono::high_resolution_clock::time_point debug_timer = std::chrono::high_resolution_clock::now();

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
    StateMachine prev_state = StateMachine::GO_HOME;
    bool arrived2HOME = false;
    bool switch_state = true;
    Eigen::Vector4d desired_joint_pos = HOME; 

    //& AUX VARIABLES
    Eigen::VectorXd joint_velocity_friction(4), Nm_friction_compensation(4), mA_friction_compensation(4);
    Eigen::VectorXd joint_torque_cmd(4), joint_velocity_fdb(4);
    Eigen::Vector4d joint_velocity_cmd       = Eigen::Vector4d::Zero();
    Eigen::Vector4d mA_joint_current_cmd     = Eigen::Vector4d::Zero();
    Eigen::Vector4d mA_stiffness_current_cmd = Eigen::Vector4d::Zero();
    Eigen::Vector4d mA_pd_current_cmd        = Eigen::Vector4d::Zero();
    Eigen::Vector4d Nm_gravity               = Eigen::Vector4d::Zero();
    Eigen::Vector3d force_cmd                = Eigen::Vector3d::Zero();
    Eigen::MatrixXd jacobian3x4              = Eigen::MatrixXd::Zero(3,4);
    double manipulabilityXy = 0.0;
    double manipulabilityXz = 0.0;

    //! GRAVITY VECTOR FOR ARM MOUNTING POINT
    bravo->kinodynamics.change_gravity_vector(GRAVITY_VECTOR); //!changing gravity compensation
    bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests  
    auto start_time = std::chrono::steady_clock::now();
    //& ARM CONNECTION
    while (!bravo->isConnected()){        
        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
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

    double LOOP_FREQUENCY = 1000.0;  // Hz (change as needed)
    rclcpp::Rate loop_rate(LOOP_FREQUENCY);  

    //& ------MAIN LOOP--------
    while (rclcpp::ok()) {
        auto loop_start = std::chrono::high_resolution_clock::now();

        //& GO TO CONFIGURATION HOME
        if ( (bravo->compute_manipulability_position() > MAX_MANIPULABILITY) || (!arrived2HOME) || (airbus_joy->goHome)){
            if (prev_state != StateMachine::GO_HOME){      
                //* STATE INITIALIZATION 
                std::cout << "Switching to GO HOME" << std::endl;             
            }
            desired_joint_pos = HOME;
            arrived2HOME = bravo->is_in_desired_configuration(0.2, desired_joint_pos, bravo->get_bravo_joint_states());        
            joint_velocity_friction = bravo->get_bravo_joint_velocities();
            joint_velocity_cmd = Eigen::Vector4d::Zero();
            MAKE_READING = false;
            prev_state = StateMachine::GO_HOME;
        }
        //& WAITING IN HOME
        else if ((!MAKE_READING) && (!airbus_joy->enableBaseMotion)){
            if (prev_state != StateMachine::HOME_WAITING){      
                //* STATE INITIALIZATION 
                std::cout << "Switching to HOME_WAITING" << std::endl;   
            }
            if (airbus_joy->makeReading){
                MAKE_READING = true;
            }
            prev_state = StateMachine::HOME_WAITING;
        }
        //& JOYSTICK TELEOP
        else if (airbus_joy->enableBaseMotion){ 
            if (prev_state != StateMachine::JOYSTICK_TELEOP){      
                //* STATE INITIALIZATION 
                std::cout << "Switching to JOYSTICK TELEOP" << std::endl;             
                switch_state = false;
                sim_joint_whole_integration = bravo->get_bravo_joint_states();
                sim_start_integration_time  = std::chrono::high_resolution_clock::now();
                sim_finish_integration_time = std::chrono::high_resolution_clock::now();
            }
            Eigen::Vector<double, 3> twist_joy_fixed;
            twist_joy_fixed << airbus_joy->teleop_VelZ, airbus_joy->teleop_VelX, airbus_joy->teleop_VelY; 
            twist_joy_fixed[0] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[0], MAX_SPEED_JOY, -MAX_SPEED_JOY);
            twist_joy_fixed[1] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[1], MAX_SPEED_JOY, -MAX_SPEED_JOY);
            twist_joy_fixed[2] = bravo_utils::VAL_SAT<double>(twist_joy_fixed[2], MAX_SPEED_JOY, -MAX_SPEED_JOY);
            jacobian3x4 = bravo->kinodynamics.fixedJacobian(sim_joint_whole_integration).topRows(3);
            Eigen::Vector<double, 4> sim_joint_vel_integration = jacobian3x4.colPivHouseholderQr().solve(twist_joy_fixed);
            joint_velocity_cmd = sim_joint_vel_integration;
            sim_finish_integration_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = sim_finish_integration_time - sim_start_integration_time;
            sim_joint_whole_integration = sim_joint_whole_integration + sim_joint_vel_integration*elapsed.count();
            sim_start_integration_time = std::chrono::high_resolution_clock::now();
            desired_joint_pos = sim_joint_whole_integration;
            joint_velocity_friction = sim_joint_vel_integration;
            prev_state = StateMachine::JOYSTICK_TELEOP;
        }
        //& STIFFNESS CONTROL 
        else{
            if (prev_state != StateMachine::STIFFNESS_CONTROL){      
                //* STATE INITIALIZATION 
                std::cout << "Switching to STIFFNESS CONTROL" << std::endl;             
                stiffness_controller.set_ref_ee_position(bravo->kinodynamics.FK_ee_pos(bravo->get_bravo_joint_states()));
            }
            jacobian3x4 = bravo->kinodynamics.fixedJacobian(bravo->get_bravo_joint_states()).topRows(3);
            force_cmd = stiffness_controller.compute_force_action(
                        stiffness_controller.get_ref_ee_position() - bravo->kinodynamics.FK_ee_pos(bravo->get_bravo_joint_states()), 
                        stiffness_controller.get_vel_ee() - jacobian3x4*bravo->get_bravo_joint_velocities());
            //$  Joint commands               
            joint_velocity_cmd = jacobian3x4.colPivHouseholderQr().solve(stiffness_controller.get_vel_ee()); //! not useful
            joint_velocity_fdb = bravo->get_bravo_joint_velocities();
            if ((joint_velocity_fdb.array().abs() < 0.04).all()){
                joint_velocity_friction = joint_velocity_fdb;
            }
            else{
                joint_velocity_friction = joint_velocity_fdb;
            }
            std::tie(manipulabilityXy, manipulabilityXz) = stiffness_controller.compute_X_compliance_ratios(jacobian3x4);
            if ((manipulabilityXy < MAX_RATIO_FORCE_ELLIPSOID) || (manipulabilityXz < MAX_RATIO_FORCE_ELLIPSOID)){
                arrived2HOME = false;
            }
            joint_torque_cmd = jacobian3x4.colPivHouseholderQr().solve(force_cmd);
            mA_stiffness_current_cmd = bravo->torqueNm_2_currentmA(joint_torque_cmd);          
            last_call_pd = std::chrono::high_resolution_clock::now();
            prev_state = StateMachine::STIFFNESS_CONTROL;
        }        
        if (DEBUG){
            if (std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - debug_timer).count() > 100){
                stiffness_controller.debug_controller();
                debug_timer = std::chrono::high_resolution_clock::now();            
                std::cout << "Manipulability: " << bravo->compute_manipulability_position() << std::endl;
                std::cout << "Compliance Ratio XY: " << manipulabilityXy << " Compliance Ratio XZ: " << manipulabilityXz << std::endl;
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
            }
            // double ratio1, ratio2;
            // std::tie(ratio1, ratio2) = stiffness_controller.compute_X_compliance_ratios(jacobian3x4);
            // std::cout << "Compliance Ratio 1: " << ratio1 << " Compliance Ratio 2: " << ratio2 << std::endl;

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
 
        //& GRAVITY COMPENSATION
        Nm_gravity = bravo->kinodynamics.invDynamics(bravo->get_bravo_joint_states(), joint_velocity_cmd, Eigen::VectorXd::Zero(4));
        
        //& COMPOSE JOINT COMMAND
        if (prev_state == StateMachine::STIFFNESS_CONTROL){
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(Nm_gravity).array() + mA_friction_compensation.array() + mA_stiffness_current_cmd.array();
        }
        else{
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(Nm_gravity).array() + mA_friction_compensation.array() + mA_pd_current_cmd.array();
        }

        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA); 
        bravo->publish_bravo_joint_states();
        loop_rate.sleep();
    }
    
    // & SAFETY: STOP THE ARM WITH ONLY GRAVITY COMPENSATION
    mA_joint_current_cmd = bravo->torqueNm_2_currentmA(Nm_gravity).array();
    bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);
    bravo->publish_bravo_joint_states();
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        // Choose a path (example: first CLI arg)
        const std::string config_filename = ament_index_cpp::get_package_share_directory("bravo_compliance") + "/config/bravo5_cp_compliance.json";
        StiffnessJsonParams stiff_params;
        try {
            stiff_params = load_stiffness_params_json(config_filename);
        }
        catch (const std::exception& e) {
            std::cerr << "[stiffness json] " << e.what() << std::endl;
            rclcpp::shutdown();
            return 1;
        }
        const std::string urdf_filename = ament_index_cpp::get_package_share_directory("bpl_bravo_description") + "/urdf/bravo_5_dynamics_no_ee_pinocchio_rov_mount.urdf";
        const std::string tool_link = std::string("contact_point");  
        auto joystick         = std::make_shared<airbus_joystick_bravo5_CP>();
        auto bravo            = std::make_shared<bravo_handler<double>>(urdf_filename, tool_link);//(urdf_filename);
        auto executor         = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();        
        executor->add_node(joystick);
        std::thread executor_thread([&executor]() {
                executor->spin();
        });
        program_loop(joystick, bravo, stiff_params);
        executor_thread.join();
        rclcpp::shutdown();
        return 0;
}



