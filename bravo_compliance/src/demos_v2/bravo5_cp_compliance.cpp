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

#include "bravo_compliance/bravo_cpp/bravo_handler/bravo_handler_v2.h"
#include "general_libs_unite/joysticks/airbus_joystick.h"
#include "general_libs_unite/interaction/stiffness_control.h"

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;



void program_loop(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo_handler<double>> bravo){     
    //& PARAMETERS
    double MAX_CURRENT_mA = 1800.0;
    double VELOCITY_X_NOMINAL = 0.02; // m/s
    double GAIN_FORCE_X = 0.01; // N to m/s
    double MAX_MANIPULABILITY = 13.0;
    double MAX_RATIO_FORCE_ELLIPSOID = 5.0;
    double MAX_SAT_SPEED = 0.15; // m/s
    double SPEED_GO_HOME = 0.15; // adimensional? I don't know
    Eigen::Vector<double, 4> HOME = (Eigen::Vector<double, 6>() << 3.14, 1.70, 1.20, 0.0).finished(); //! define home for bravo5
    Eigen::Vector3d gravity_vector = (Eigen::Vector3d() << 0.0, 0.0, -9.81).finished();

    //$ FRICTION JOINT PARAMETERS (VALID FOR BRAVO7 OR BRAVO5)
    Eigen::VectorXd mA_joint_current_cmd(4), wrench_command(4), joint_velocity_friction(4), mA_stiction_stribeck_custom(4), coulomb_custom(4), stiction_custom(4), qd_stiction(4);
    coulomb_custom  << 350.0, 440.0, 440.0, 440.0;
    stiction_custom << 400.0, 450.0, 450.0, 450.0
    qd_stiction     << 0.006, 0.006, 0.006, 0.006;

    auto start_time = std::chrono::steady_clock::now();
    stiffness_control<double> stiffness_controller(bravo->get_number_joints());
    stiffness_controller.set_desired_force(Eigen::Vector3d(0.0, 0.0, 45.0)); // Desired force in Z

    //& AUX VARIABLES, probably should be in stiffness controller
    Eigen::VectorXd ref_joint_pos;
    Eigen::Matrix<double, 6, Eigen::Dynamic> Jacobian;
    Jacobian.resize(Eigen::NoChange, bravo->get_number_joints());   
    Eigen::VectorXd joint_torque_cmd, joint_velocity_cmd, joint_velocity_fdb;
    joint_torque_cmd.resize(bravo->get_number_joints());
    joint_velocity_cmd.resize(bravo->get_number_joints());
    joint_velocity_fdb.resize(bravo->get_number_joints());
    
    //& ARM CONNECTION
    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests  
             //! GRAVITY VECTOR FOR ARM MOUNTING POINT
        bravo->kinodynamics.change_gravity_vector(gravity_vector); //!changing gravity compensation
        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
    }    
    //& INITIAL POSITION AND POSE REFERENCE
    if (bravo->compute_manipulability() < MAX_MANIPULABILITY){
        std::cout << "Initial manipulability condition number OK, setting ref" << std::endl;

        //& MAIN LOOPcoulomb_custom
        while (rclcpp::ok()) {
            //& GRAVITY COMPENSATION
            Eigen::VectorXd tau_gravity = bravo->kinodynamics.invDynamics(bravo->get_bravo_joint_states(), joint_velocity_cmd, Eigen::VectorXd::Zero(4));
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);

            //& GO TO CONFIGURATION HOME
            if (bravo->compute_manipulability() > MAX_MANIPULABILITY){
                bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);
                std::cerr << "[FATAL] Unsafe manipulability. Shutting down ROS."  << std::endl;
                rclcpp::shutdown();
                return;            
            }
            //& STIFFNESS CONTROL 
            else{
                Jacobian = bravo->kinodynamics.fixedJacobian(bravo->get_bravo_joint_states());
                wrench_command = stiffness_controller.compute_wrench(
                                bravo->kinodynamics.compute_SE3_error_fixed_axis(ref_joint_pos, bravo->get_bravo_joint_states()), 
                                stiffness_controller.get_twist_3D() - Jacobian*bravo->get_bravo_joint_velocities());
                //$  Joint commands               
                joint_velocity_cmd = Jacobian.transpose() * stiffness_controller.get_twist_3D();
                mA_joint_current_cmd  = bravo->torqueNm_2_currentmA(joint_torque_cmd);
                //& HANDLE FRICTION TO MAKE BRAVO AS COMPLIANT AS POSSIBLE
                joint_velocity_fdb = bravo->get_bravo_joint_velocities();
                if ((joint_velocity_fdb.array().abs() < 0.04).all()){
                    joint_velocity_friction = joint_velocity_cmd;
                }
                else{
                    joint_velocity_friction = joint_velocity_fdb;
                }
                mA_stiction_stribeck_custom =  bravo->kinodynamics.compute_I_ff_stribeck(joint_velocity_friction, coulomb_custom, stiction_custom, qd_stiction);
                mA_joint_current_cmd = mA_joint_current_cmd.array() + mA_stiction_stribeck_custom.array();
                joint_torque_cmd = bravo->kinodynamics.fixedJacobian(bravo->get_bravo_joint_states()).transpose() * wrench_command;
                mA_joint_current_cmd = bravo->torqueNm_2_currentmA(joint_torque_cmd) + mA_joint_current_cmd;
                
                //! CHECKING IN SATURATION IS HAPPENING
                bool exceeds_limit = false;
                for (int i = 0; i < bravo->get_number_joints(); i++) {
                    if (std::abs(mA_joint_current_cmd[i]) > MAX_CURRENT_mA) {
                        exceeds_limit = true;
                        break;
                    }
                }
                if (exceeds_limit) {
                    std::cout << "Current limit exceeded, SATURATING" << std::endl;
                    std::cout << "Current [0]: " << mA_joint_current_cmd[0]
                                    << " [1]: " << mA_joint_current_cmd[1]
                                    << " [2]: " << mA_joint_current_cmd[2]
                                    << " [3]: " << mA_joint_current_cmd[3]
                                    << std::endl;
                }
                bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);
                bravo->publish_bravo_joint_states();
            }            
        }
    }
    else{
        mA_joint_current_cmd << 0.0, 0.0, 0.0, 0.0;
        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, MAX_CURRENT_mA);
        std::cerr << "High initial manipulability condition number for current joint configuration. Finishing Program" << std::endl;
        rclcpp::shutdown();
        return;
    }

}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        const std::string urdf_filename = ament_index_cpp::get_package_share_directory("bpl_bravo_description") + "/urdf/bravo_5_dynamics_no_ee_pinocchio.urdf";
        const std::string tool_link = std::string("contact_point");  
        auto joystick         = std::make_shared<airbus_joystick_bravo_twist_ee>();
        auto bravo            = std::make_shared<bravo_handler<double>>(urdf_filename, tool_link);//(urdf_filename);
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



