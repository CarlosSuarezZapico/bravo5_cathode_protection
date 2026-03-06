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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "bravo_cathode_protection/bravo_cpp/bravo_handler/bravo_handler_v2.h"
#include <string>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>


struct FrictionConfig
{
    Eigen::Matrix<double,4,6> friction_mat = Eigen::Matrix<double,4,6>::Zero();
    double friction_saturation = 60.0;
};

static FrictionConfig load_friction_config_yaml(const std::string& path)
{
    if (!std::ifstream(path).good()) {
        throw std::runtime_error("Could not open friction config file: " + path);
    }

    const YAML::Node node = YAML::LoadFile(path);
    const YAML::Node friction = node["friction"];
    if (!friction || !friction.IsMap()) {
        throw std::runtime_error("YAML key 'friction' must be a map");
    }

    FrictionConfig cfg;
    for (int joint = 0; joint < 4; ++joint) {
        const std::string joint_key = "joint" + std::to_string(joint + 1);
        const YAML::Node joint_node = friction[joint_key];
        if (!joint_node || !joint_node.IsMap()) {
            throw std::runtime_error("YAML key 'friction." + joint_key + "' must be a map");
        }

        cfg.friction_mat(joint, 0) = joint_node["g0"].as<double>();
        cfg.friction_mat(joint, 1) = joint_node["g1"].as<double>();
        cfg.friction_mat(joint, 2) = joint_node["g2"].as<double>();
        cfg.friction_mat(joint, 3) = joint_node["g3"].as<double>();
        cfg.friction_mat(joint, 4) = joint_node["g4"].as<double>();
        cfg.friction_mat(joint, 5) = joint_node["g5"].as<double>();
        cfg.friction_saturation = joint_node["s"].as<double>();
    }
    return cfg;
}

void program_loop(std::shared_ptr<bravo_handler<double>> bravo,
                  const FrictionConfig& friction_config){     
    const Eigen::Matrix<double,4,6>& FRICTION_MAT = friction_config.friction_mat;
    const double FRICTION_SATURATION = friction_config.friction_saturation; 
    auto start_time = std::chrono::steady_clock::now();
    bravo->kinodynamics.change_gravity_vector((Eigen::Vector3d() << 0.0, 0.0, 9.81).finished()); //! GRAVITY VECTOR FOR ARM MOUNTING POINT
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
    Eigen::VectorXd mA_joint_current_cmd(4), Nm_friction_compensation(4), mA_friction_compensation(4);
    mA_joint_current_cmd = Eigen::VectorXd::Zero(4);
    double MAX_CURRENT_mA = 1500.0;
    
    while (rclcpp::ok()) {
        //& PLOT MANIPULABILITY
        std::cout << "Manipulablity: "<< bravo->kinodynamics.compute_manipulability_position(bravo->get_bravo_joint_states()) << std::endl;
        Eigen::VectorXd joint_velocity_friction = bravo->get_bravo_joint_velocities();
        //& FRICTION COMPENSATION
        Nm_friction_compensation[0] = bravo->kinodynamics.computeFriction(joint_velocity_friction[0], FRICTION_MAT(0,0), FRICTION_MAT(0,1), FRICTION_MAT(0,2), FRICTION_MAT(0,3), FRICTION_MAT(0,4), FRICTION_MAT(0,5), FRICTION_SATURATION);
        Nm_friction_compensation[1] = bravo->kinodynamics.computeFriction(joint_velocity_friction[1], FRICTION_MAT(1,0), FRICTION_MAT(1,1), FRICTION_MAT(1,2), FRICTION_MAT(1,3), FRICTION_MAT(1,4), FRICTION_MAT(1,5), FRICTION_SATURATION);
        Nm_friction_compensation[2] = bravo->kinodynamics.computeFriction(joint_velocity_friction[2], FRICTION_MAT(2,0), FRICTION_MAT(2,1), FRICTION_MAT(2,2), FRICTION_MAT(2,3), FRICTION_MAT(2,4), FRICTION_MAT(2,5), FRICTION_SATURATION);
        Nm_friction_compensation[3] = bravo->kinodynamics.computeFriction(joint_velocity_friction[3], FRICTION_MAT(3,0), FRICTION_MAT(3,1), FRICTION_MAT(3,2), FRICTION_MAT(3,3), FRICTION_MAT(3,4), FRICTION_MAT(3,5), FRICTION_SATURATION);
        mA_friction_compensation = bravo->torqueNm_2_currentmA(Nm_friction_compensation);
        //& GRAVITY COMPENSATION
        Eigen::VectorXd tau_gravity = bravo->kinodynamics.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(4), Eigen::VectorXd::Zero(4));
        mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
        mA_joint_current_cmd = mA_joint_current_cmd.array() + mA_friction_compensation.array(); // Add stiction compensation
        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);
    }
}
//&  MAIN 
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);       
    const std::string package_share = ament_index_cpp::get_package_share_directory("bravo_cathode_protection");
    const std::string urdf_filename = package_share + "/urdf/bravo_5_dynamics_pinocchio_cp.urdf";
    const std::string config_friction_file = package_share + "/config/joint_friction_params/joint_friction_bravo5.yaml";
    const std::string tool_link = std::string("contact_point");
    const std::string ip_address = std::string("10.43.0.146");
    FrictionConfig friction_config;
    try {
        friction_config = load_friction_config_yaml(config_friction_file);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    auto bravo            = std::make_shared<bravo_handler<double>>(urdf_filename, tool_link, bravo_control::ArmModel::bravo5, ip_address);
    auto executor         = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();        
    std::thread executor_thread([&executor]() {
            executor->spin();
    });
    program_loop(bravo, friction_config);
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}
