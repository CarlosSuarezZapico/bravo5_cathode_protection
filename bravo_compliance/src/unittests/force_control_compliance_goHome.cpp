/**
 *    @file  force_control_compliance_goHome.cpp
 *    @brief Unittest for the force control/compliance/gotoJointPos of the Bravo7 arm
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  16-May-2023
 *    Modification 28-May-2023
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
#include <rclcpp/rclcpp.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

enum state { GO_HOME, CONTACT};

void program_loop(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo7_handler<double>> bravo){
    state program_state = GO_HOME;
    float CONTACT_LOSS = 5.0;
    auto force_controllerX = std::make_shared<force_control<double>>();
    auto force_controllerY = std::make_shared<force_control<double>>();
    auto force_controllerZ = std::make_shared<force_control<double>>();
    force_controllerX->set_admittance_parameters(1.5, 150); //set virtual mass and damping (2,300) (1.5, 250)
    force_controllerY->set_admittance_parameters(1.5, 150);
    force_controllerZ->set_admittance_parameters(2.0, 350);

    if (bravo->isConnected()){
        RCLCPP_INFO(bravo->get_logger(), "[BRAVO7_FORCE_CONTROL_SCRIPT]: Bravo7 is connected");
        bravo->set_bravo_frequency_packet_exchange(1000); //! Set frequency of requests
    }
    else{
        RCLCPP_ERROR(bravo->get_logger(), "[BRAVO7_FORCE_CONTROL_SCRIPT]: Bravo7 is NOT connected");
    }

    Eigen::Vector<double, 6> HOME;
    HOME << 0.323, 2.708, 0.5045, 1.9917, 1.6112, 0.2304;
    bravo->go_to_JointPos(HOME, 2.0);
    while (rclcpp::ok()) {
        if (bravo->compute_manipulability()>25){
            bravo->go_to_JointPos(HOME, 2.0);
            program_state = GO_HOME;
            bravo->tare_ft();
        }
        else{
            Eigen::Vector<double, 6> twist_joy;
            float reduced_vel_factor = 1;
            twist_joy << 1*airbus_joy->teleop_VelX, 1*airbus_joy->teleop_VelY, 1*airbus_joy->teleop_VelZ, 0.0, 0.0, 0.0;
            //FORCE CONTROLLER
            Eigen::Vector<double, 6> FT = bravo->get_ft_fdb_data();
            force_controllerX->md_admittance_force_control_loop(0.005, CONTACT_LOSS, -FT[0]);
            force_controllerY->md_admittance_force_control_loop(0.005, CONTACT_LOSS, -FT[1]);
            force_controllerZ->md_admittance_force_control_loop(0.005, 8.0, -FT[2]);
            double vel = 0.0;
            //& X-AXIS COMPLIANCE
            if (abs(FT[0])>CONTACT_LOSS){
                twist_joy(0) = force_controllerX->get_admittance_vel();
            }
            else{
                twist_joy(0) = 0.0;
            }
            //& Y-AXIS COMPLIANCE
            if (abs(FT[1]) > CONTACT_LOSS){
                twist_joy(1) = force_controllerY->get_admittance_vel();
            }
            else{
                twist_joy(1) = 0.0;
            }
            //& Z-AXIS CONTACT MAINTENANCE
            if (abs(FT[2]) < CONTACT_LOSS){
                twist_joy(2)  = 0.05;
            }
            else{
                //RCLCPP_INFO(bravo->get_logger(), "[BRAVO7_FORCE_CONTROL_SCRIPT]: Bravo7 is in contact with the environment");
                program_state = CONTACT;
                twist_joy(2)  = force_controllerZ->get_admittance_vel();
            }
            twist_joy(0) = bravo_utils::VAL_SAT<double>(twist_joy(0), 0.25, -0.25);
            twist_joy(1) = bravo_utils::VAL_SAT<double>(twist_joy(1), 0.25, -0.25);
            twist_joy(2) = bravo_utils::VAL_SAT<double>(twist_joy(2), 0.25, -0.25);
            
            //RCLCPP_WARN(bravo->get_logger(), "[BRAVO7_FORCE_CONTROL_SCRIPT]: EE_twist %f, %f, %f, %f, %f, %f ", twist_joy[0], twist_joy[1], twist_joy[2], twist_joy[3], twist_joy[4], twist_joy[5]);   

            bravo->moveCmdTCPLocalTwist(reduced_vel_factor*twist_joy, 0.005); //! I DONT LIKE THIS INSTRUCTION HERE
        }
        
        bravo->publish_bravo_joint_states();
        bravo->publish_bravo_wrench();
    }
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



