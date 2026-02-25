/**
 *    @file  joint_CurrentFdb_FT_joystick_dataCollection.cpp
 *    @brief Script to move the arm around making interaction with the end-effector and collecting
 *    data from the joint current feedback and force-torque sensor.The information is published in ROS and can 
 *    be collected via a ROS BAG. 
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  16-May-2023
 *    Modification 12-Jun-2025
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

#include "bravo_manipulator/bravo_cpp/bravo_handler/bravo7_handler.h"
#include "general_libs_unite/joysticks/airbus_joystick.h"
#include "general_libs_unite/interaction/interaction_control.h"
#include "general_libs_unite/general_utils/general_utils.h"

#include <iostream>
#include <ctime>
#include <rclcpp/rclcpp.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

enum state { GO_HOME, CONTACT};

struct DataBravo{
    double time_seconds;
    Eigen::Vector<double, 6> joint_currents;
    Eigen::Vector<double, 6> ft_sensor;
    Eigen::Vector<double, 6> joint_positions;
};

std::atomic<bool> g_shutdown_requested(false);

// Signal handler
void signal_handler(int signal) {
    if (signal == SIGINT) {
        g_shutdown_requested = true;
        std::cout << "\nSIGINT received. Shutting down...\n";
    }
}

void program_loop(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo7_handler<double>> bravo){     
    //& INITIAL -- START TIMER, ESTABLISH CONNECTION AND OPEN FILE TO STORE DATABASE
    auto start_time = std::chrono::steady_clock::now();
    std::chrono::high_resolution_clock::time_point last_data_pushback_time; 
    std::vector<DataBravo> data_collection; // Vector to store data
    //& CREATE FOLDER TO STORE DATA, THE NAME OF THE FOLDER CONTAINS THE DATE AND TIME
    std::string folder = "src/bravo_manipulator/datasets/";  // from src/unittests/ to datasets/
    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&time);
    // 4. Format filename with date-time
    std::ostringstream filename;
    filename << folder << "FEEDBACK_ENCODERS_CURRENT_FT_"<< std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".csv";
    std::ofstream out(filename.str());
    if (!out) {
        std::cerr << "Failed to open file: " << filename.str() << "\n";
        RCLCPP_INFO(bravo->get_logger(), "[BRAVO7_FORCE_CONTROL_SCRIPT]: Leaving program loop, file not opened");
        return;
    }
    std::string file_path = filename.str();  // Save path for later
    RCLCPP_INFO(bravo->get_logger(), "[BRAVO7_FORCE_CONTROL_SCRIPT]: Check bravo connection");
    //& ESTABLISH CONNECTION WITH BRAVO MANIPULATOR
    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(1000); //! Set frequency of requests
        bravo->set_feedback_current_joints(true); // Enable joint current feedback

        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
    }    
    //& PROGRAM LOOP 
    Eigen::Vector<double, 6> HOME;
    HOME << 0.323, 2.708, 0.5045, 1.9917, 1.6112, 0.2304;
    bravo->go_to_JointPos(HOME, 2.0);
    RCLCPP_INFO(bravo->get_logger(), "[BRAVO7_FORCE_CONTROL_SCRIPT]: Entering in loop");
    while (rclcpp::ok() && !g_shutdown_requested) {
        if (bravo->compute_manipulability()>25){
            bravo->go_to_JointPos(HOME, 2.0);
            RCLCPP_INFO(bravo->get_logger(), "[BRAVO7_FORCE_CONTROL_SCRIPT]: In home already");
        }
        else{
            Eigen::Vector<double, 6> twist_joy;
            float reduced_vel_factor = 1;
            twist_joy << 2.5*airbus_joy->teleop_VelX, 2.5*airbus_joy->teleop_VelY, 2.5*airbus_joy->teleop_VelZ, 0.0, 0.0, 0.0;
            bravo->moveCmdTCPLocalTwist(reduced_vel_factor*twist_joy, 0.0005);
        }

        std::chrono::duration<double> elapsed_integration= std::chrono::high_resolution_clock::now() - last_data_pushback_time;
        if (elapsed_integration.count() > 0.01) {
            bravo->publish_bravo_joint_states();
            bravo->publish_bravo_joint_currents();
            bravo->publish_bravo_wrench();
            DataBravo data;
            data.time_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            data.joint_currents = bravo->get_bravo_joint_currents();
            data.ft_sensor = bravo->get_ft_fdb_data();
            data.joint_positions = bravo->get_bravo_joint_states();
            data_collection.push_back(data); // Store data in vector
        }
    }

    // SAVE DATA ON EXIT
    if (!out) {
        std::cerr << "Failed to open file for writing: " << file_path << "\n";
        return;
    }

    // CSV Header
    out << "time,"
        << "jc1,jc2,jc3,jc4,jc5,jc6,"
        << "ft1,ft2,ft3,ft4,ft5,ft6,"
        << "jp1,jp2,jp3,jp4,jp5,jp6\n";

    for (const auto& d : data_collection) {
        out << d.time_seconds << ",";
        for (int i = 0; i < 6; ++i) out << d.joint_currents[i] << ",";
        for (int i = 0; i < 6; ++i) out << d.ft_sensor[i] << ",";
        for (int i = 0; i < 6; ++i) out << d.joint_positions[i] << ",";
        out << "\n";
    }

    out.close();
    std::cout << "✔ Data saved to: " << file_path << "\n";
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_pinocchio.urdf");
    const std::string tool_link = std::string("contact_point");
    auto joystick         = std::make_shared<airbus_joystick_bravo_twist_ee>();
    auto bravo            = std::make_shared<bravo7_handler<double>>(urdf_filename, tool_link);//(urdf_filename);
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



