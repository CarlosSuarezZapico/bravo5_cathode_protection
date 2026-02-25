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


template<typename Derived>
Eigen::Array<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
smoothSignEigen(const Eigen::ArrayBase<Derived>& x, double epsilon) {
    return x / ((x.square() + epsilon * epsilon).sqrt());
}

void program_loop(std::shared_ptr<airbus_joystick_compliance> airbus_joy, std::shared_ptr<bravo7_handler<double>> bravo){     
    auto start_time = std::chrono::steady_clock::now();
    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests

        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
    }    
    Eigen::Vector<double, 6> STORE, HOME;
    //STORE << 0.323, 2.957, 0.05, 3.387, 0.066, 0.2304;
    STORE << 4.904, 2.857, 0.05, 3.387, 0.066, 0.2304;
    HOME  << 4.904, 2.592, 0.549, 4.904, 1.570, 0.0; 

    Eigen::VectorXd ref_joint_pos = bravo->get_bravo_joint_states();
    Eigen::VectorXd torque_cmd(6);
    Eigen::VectorXd joint_position_cmd(6), joint_velocity_cmd(6);
    torque_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    bool high_cond = false;
    Eigen::VectorXd stiction(6), viscous_damping(6);
    viscous_damping << 34.45, 27.30, 14.82, 11.39, 15.68, 18.30;
    //stiction << 350.0, 350.0, 350.0, 350.0, 350.0, 350.0; // Stiction values for each joint
    stiction << 400.0, 400.0, 400.0, 400.0, 400.0, 400.0; // Stiction values for each joint
    Eigen::VectorXd pos_gains(6), pos_gains_contact(6), vel_gains(6); 
    //pos_gains_contact << 4000.0, 4000.0, 4000.0, 2500.0, 2500.0, 2500.0; //COMPLIANCE VALUES
    pos_gains << 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0;
    //vel_gains << 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0; // Gains for velocity control
    vel_gains << 600.0, 600.0, 600.0, 600.0, 600.0, 600.0; // Gains for velocity control
    //pos_gains <<10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0;
    Eigen::Vector3d ref_ee_pos, current_ee_pos;
    Eigen::Matrix3d ref_ee_rot, current_ee_rot;
    std::tie(ref_ee_pos, ref_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(ref_joint_pos);
    Eigen::Vector3d position_error_fixed, force_estimation, position_error_global;
    force_estimation << 0.0, 0.0, 0.0;
    bravo->go_to_JointPos(HOME, 2.0);
    

    while (rclcpp::ok()) {
        if (bravo->compute_manipulability()>25){
            // torque_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            // bravo->cmdJointCurrent_SAT(torque_cmd, 2000.0);
            bravo->go_to_JointPos(HOME, 2.0);
            std::cout << "Manipulability is high, no command sent." << std::endl;
        }
        else{
            
            Eigen::Vector<double, 6> twist_joy, twist_fixed;
            double reduced_vel_factor = 1.0;
            twist_joy << reduced_vel_factor*airbus_joy->teleop_VelX, reduced_vel_factor*airbus_joy->teleop_VelY, reduced_vel_factor*airbus_joy->teleop_VelZ, 0.0, 0.0, 0.0;
            std::cout << "Diff called" << std::endl;
            std::tie(high_cond, joint_position_cmd, joint_velocity_cmd) = bravo->twist_diff_kin_2(twist_joy, 0.02);
            std::cout << "Position [0]: " << twist_joy[0] <<" [1]: "<< twist_joy[1] <<" [2]: "<< twist_joy[2] << " [3]: " <<
                            twist_joy[3] <<" [4]: "<< twist_joy[4] <<" [5]: "<< twist_joy[5] <<std::endl;
            if (high_cond){
                // torque_cmd = pos_gains.array() * (bravo->signedAngleDistance(joint_position_cmd, bravo->get_bravo_joint_states())).array() + 
                //              stiction.array() * (bravo->signedAngleDistance(joint_position_cmd, bravo->get_bravo_joint_states())).array().sign();

            Eigen::ArrayXd pos_err = bravo->signedAngleDistance(joint_position_cmd, bravo->get_bravo_joint_states()).array();
            Eigen::ArrayXd vel_err = bravo->signedAngleDistance(joint_velocity_cmd, bravo->get_bravo_joint_velocities()).array();
            Eigen::VectorXd tau_gravity = bravo->manipulator_kin_dyn.invDynamics(bravo->get_bravo_joint_states(), joint_velocity_cmd, Eigen::VectorXd::Zero(6));
            Eigen::VectorXd mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
            //Eigen::VectorXd mA_gravity =  (tau_gravity/(0.222*120.0))*1000.0;

            torque_cmd = pos_gains.array() * pos_err + vel_gains.array() * vel_err + stiction.array() * smoothSignEigen(pos_err, 0.0015) + mA_joint_current_cmd.array() + 
                         viscous_damping.array() * joint_velocity_cmd.array();          
            }
            else{
                std::cout<< "Low condition, no command sent." << std::endl;
            }

            bravo->cmdJointCurrent_SAT(torque_cmd, 2000.0);
        }        
        bravo->publish_bravo_joint_states();
    }
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        //const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_noFT_pinocchio.urdf");        
        const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_amir_2.urdf");
        const std::string tool_link = std::string("EE");   
        //const std::string tool_link = std::string("contact_point");
        auto joystick         = std::make_shared<airbus_joystick_compliance>();
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



