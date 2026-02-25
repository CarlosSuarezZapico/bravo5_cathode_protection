/**
 *    @file  diff_kin_loop.cpp
 *    @brief Unittest for the Bravo7 arm. The program run a loop back and forth. 
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  16-May-2025
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

#include "bravo7_version_2/bravo_cpp/bravo_handler/bravo7_handler_v2.h"
#include "general_libs_unite/joysticks/airbus_joystick.h"
#include "general_libs_unite/interaction/interaction_control.h"
#include "general_libs_unite/general_utils/general_utils.h"

#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

enum state { GO_HOME, CONTACT};

void program_loop(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo7_handler<double>> bravo){     

    while (rclcpp::ok()) {
            
            Eigen::Vector<double, 6> twist_joy;
            double reduced_vel_factor = 1.0;
            twist_joy << reduced_vel_factor*airbus_joy->teleop_VelX, reduced_vel_factor*airbus_joy->teleop_VelY, reduced_vel_factor*airbus_joy->teleop_VelZ, 0.0, 0.0, 0.0;
            std::cout << "Airbus Cmd [0]: " << twist_joy[0] <<" [1]: "<< twist_joy[1] <<" [2]: "<< twist_joy[2] << " [3]: " <<
                                             twist_joy[3] <<" [4]: "<< twist_joy[4] <<" [5]: "<< twist_joy[5] <<std::endl;
    }
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_noFT_pinocchio.urdf");        
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



