/**
 *    @file  program_cp_compliance.cpp
 *    @brief Program to make interaction with Bravo arms. Design for cathode-protection tasks
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      5-Dec-2025
 *    Modification 4-Sep-2025
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
//#include "general_libs_unite/interaction/interaction_control.h"
#include "general_libs_unite/interaction/stiffness_control.h"

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;

void program_loop(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo7_handler<double>> bravo){     
    auto start_time = std::chrono::steady_clock::now();
    stiffness_control<double> stiffness_controller(bravo->get_number_joints());
    stiffness_controller.set_desired_force(Eigen::Vector3d(0.0, 0.0, 45.0)); // Desired force in Z
    //& PARAMETERS
    double MAX_CURRENT_mA = 2500.0;
    double VELOCITY_X_NOMINAL = 0.02; // m/s
    double GAIN_FORCE_X = 0.01; // N to m/s
    double MAX_MANIPULABILITY = 25.0;
    double MAX_RATIO_FORCE_ELLIPSOID = 5.0;
    double MAX_SAT_SPEED = 0.15; // m/s
    double SPEED_GO_HOME = 0.15; // adimensional? I don't know
    Eigen::Vector<double, 6>  HOME;
    HOME  << 3.10, 2.50, 0.4, 3.1, 1.4, 0.0;

    //$ FRICTION JOINT PARAMETERS (VALID FOR BRAVO7 OR BRAVO5)
    Eigen::VectorXd mA_joint_current_cmd(6), coulomb_custom(6), stiction_custom(6), qd_stiction(6);
    coulomb_custom  << 350.0, 440.0, 440.0, 350.0, 440.0, 350.0;
    stiction_custom << 400.0, 450.0, 450.0, 400.0, 450.0, 450.0;
    qd_stiction     << 0.006, 0.006, 0.006, 0.006, 0.006, 0.006;

    //& AUX VARIABLES, probably should be in stiffness controller
    Eigen::VectorXd ref_joint_pos;
    Eigen::Matrix<double, 6, Eigen::Dynamic> Jacobian;
    Jacobian.resize(Eigen::NoChange, bravo->get_number_joints());   
    Eigen::VectorXd joint_torque_cmd, joint_velocity_cmd joint_velocity_fdb;
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
    if (bravo->compute_manipulability() > MAX_MANIPULABILITY){
        std::cout << "Initial manipulability condition number OK, setting ref" << std::endl;

        //& MAIN LOOP
        while (rclcpp::ok()) {

            //& GO TO CONFIGURATION HOME
            if (bravo->compute_manipulability() > MAX_MANIPULABILITY){
            
            }
            //& STIFFNESS CONTROL 
            else{

                Jacobian = bravo->kinodynamics.fixedJacobian(bravo->get_bravo_joint_states());
                wrench_output = stiffness_controller.compute_joint_current(
                                bravo->kinodynamics.compute_SE3_error_fixed_axis(ref_joint_pos, bravo->get_bravo_joint_states()), 
                                stiffness_controller.get_twist_3D()- Jacobian*bravo->get_bravo_joint_velocities());
                //$  Joint commands               
                joint_velocity_cmd = Jacobian.transpose() * stiffness_controller.get_twist_3D();
                joint_current_cmd  = bravo->torqueNm_2_currentmA(wrench_output);
                //& HANDLE FRICTION TO MAKE BRAVO AS COMPLIANT AS POSSIBLE
                joint_velocity_fdb = bravo->get_bravo_joint_velocities();
                if ((joint_velocity_fdb.array().abs() < 0.04).all()){
                    joint_velocity_friction = joint_velocity_cmd;
                }
                else{
                    joint_velocity_friction = bravo_vel_feedback;
                }
                mA_stiction_stribeck_custom =  bravo->kinodynamics.compute_I_ff_stribeck(joint_velocity_friction, coulomb_custom, stiction_custom, qd_stiction);
                Eigen::VectorXd tau_gravity = bravo->manipulator_kin_dyn.invDynamics(bravo->get_bravo_joint_states(), joint_velocity_cmd, Eigen::VectorXd::Zero(6));
                mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
                mA_joint_current_cmd = mA_joint_current_cmd.array() + mA_stiction_stribeck_custom.array();

                joint_torque_cmd = bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states()).transpose() * wrench_command;
                joint_current_cmd = bravo->torqueNm_2_currentmA(joint_torque_cmd) + mA_joint_current_cmd;
                
                //! CHECKING IN SATURATION IS HAPPENING
                bool exceeds_limit = false;
                for (int i = 0; i < 6; i++) {
                    if (std::abs(joint_current_cmd[i]) > MAX_CURRENT_mA) {
                        exceeds_limit = true;
                        break;
                    }
                }
                if (exceeds_limit) {
                    std::cout << "Current limit exceeded, SATURATING" << std::endl;
                    std::cout << "Current [0]: " << joint_current_cmd[0]
                            << " [1]: " << joint_current_cmd[1]
                            << " [2]: " << joint_current_cmd[2]
                            << " [3]: " << joint_current_cmd[3]
                            << " [4]: " << joint_current_cmd[4]
                            << " [5]: " << joint_current_cmd[5]
                            << std::endl;
                }
                bravo->cmdJointCurrent_SAT(joint_current_cmd, MAX_CURRENT_mA);
                bravo->publish_bravo_joint_states();
                bravo->set_joint_whole_integration(bravo->get_bravo_joint_states());
                bravo->set_interpolated_q(bravo->get_bravo_joint_states());
                last_interpolation = std::chrono::high_resolution_clock::now();
            }
            
        }

    }
    else{
        std::cerr << "High initial manipulability condition number for current joint configuration. Finishing Program" << std::endl;
        return;
    }

}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        //const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_amir_2.urdf");     
        const std::string urdf_filename = ament_index_cpp::get_package_share_directory("bpl_bravo_description"); + "/urdf/bravo_5_dynamics.urdf";   
        const std::string tool_link = std::string("EE");
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



