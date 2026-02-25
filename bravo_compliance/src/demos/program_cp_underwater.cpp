/**
 *    @file  task_stiffness_fixed_axis_control_moving_forward.cpp
 *    @brief Unittest for the Bravo7 arm. The program runs a stiffness control
 *    in task space. It enforces compliance in the Z direction of the end-effector. 
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      4-Aug-2025
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
#include "general_libs_unite/interaction/interaction_control.h"



#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace bravo_utils;


Eigen::VectorXd compute_I_ff_stribeck_smooth(const Eigen::VectorXd& qd, const Eigen::VectorXd& I_coulomb, const Eigen::VectorXd& I_stiction, double v_stiction, double k,  double vel_eps){
    Eigen::VectorXd out(qd.size());
    for (int i = 0; i < qd.size(); ++i) {
        double v = qd[i];
        // Blending term: 1 / (1 + (|v| / v_stiction)^k)
        double blend = 1.0 / (1.0 + std::pow(std::abs(v) / v_stiction, k));
        // Smooth sign via tanh
        double sgn_smooth = std::tanh(v / vel_eps);
        // Combine
        out[i] = (I_coulomb[i] + (I_stiction[i] - I_coulomb[i]) * blend) * sgn_smooth;
    }
    return out;
}

Eigen::VectorXd compute_I_ff_stribeck_smooth(const Eigen::VectorXd& qd, const Eigen::VectorXd& I_coulomb, const Eigen::VectorXd& I_stiction, const Eigen::VectorXd& v_stiction, const Eigen::VectorXd& k,  const Eigen::VectorXd& vel_eps){
    Eigen::VectorXd out(qd.size());
    for (int i = 0; i < qd.size(); ++i) {
        double v = qd[i];
        // Blending term: 1 / (1 + (|v| / v_stiction)^k)
        double blend = 1.0 / (1.0 + std::pow(std::abs(v) / v_stiction[i], k[i]));
        // Smooth sign via tanh
        double sgn_smooth = std::tanh(v / vel_eps[i]);
        // Combine
        out[i] = (I_coulomb[i] + (I_stiction[i] - I_coulomb[i]) * blend) * sgn_smooth;
    }
    return out;
}

double ratio_force_ellipsoid (const Eigen::MatrixXd Jacobian){
    Eigen::MatrixXd Jacobian_linear = Jacobian.topRows(3);
    // Perform SVD: Jlin = U * S * V^T
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jacobian_linear, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s = svd.singularValues();
    // Numerical safeguard
    double eps = 1e-12;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) < eps) s(i) = eps;
    }
     // Motion and force ellipsoid radii
    Eigen::VectorXd motion_radii = s;           // proportional to velocity amplification
    Eigen::VectorXd force_radii  = s.cwiseInverse();  // reciprocal for force transmission
    double ratio = abs(force_radii[2]/ force_radii[0]);
    return ratio;
}

void program_loop(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo7_handler<double>> bravo){     
    auto start_time = std::chrono::steady_clock::now();
    double max_current_mA = 2500.0;
    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests  
        Eigen::Vector3d gravity_vector;
        gravity_vector << 0.0, 0.0, 9.81; 
        bravo->manipulator_kin_dyn.change_gravity_vector(gravity_vector); //!changing gravity compensation

        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
    }    

    Eigen::VectorXd mA_joint_current_cmd(6), coulomb_custom(6), stiction_custom(6);
    //!coulomb_custom << 300.0, 200.0, 200.0, 200.0, 200.0, 200.0;
    coulomb_custom << 350.0, 350.0, 350.0, 350.0, 350.0, 350.0;
    //!stiction_custom << 350.0, 350.0, 350.0, 350.0, 350.0, 350.0;
    stiction_custom << 400.0, 400.0, 400.0, 400.0, 400.0, 400.0;

    //&INITIAL POSITION AND POSE REFERENCE
    Eigen::VectorXd ref_joint_pos = bravo->get_bravo_joint_states();
    Eigen::Vector3d ref_ee_pos, current_ee_pos;
    Eigen::Vector3d ref_ee_vel, current_ee_vel, ref_ee_w, current_ee_w;
    Eigen::Matrix3d ref_ee_rot, current_ee_rot;
    Eigen::Vector3d position_error_fixed, force_estimation, position_error_global;
    std::tie(ref_ee_pos, ref_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(ref_joint_pos);
    ref_ee_vel << 0.0, 0.0, 0.0;
    ref_ee_w   << 0.0, 0.0, 0.0;

    Eigen::VectorXd joint_current_cmd = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd joint_torque_cmd(6), joint_velocity_cmd(6);

    bool low_cond = false;
    //& TASK SPACE STIFFNESS CONTROL
    //* Position stiffness and damping matrix
    Eigen::Vector3d Kxyz(700.0, 8000.0, 8000.0); // Stiffness gains for 3D control
    Eigen::MatrixXd positionStiffness = Kxyz.asDiagonal();
    Eigen::Vector3d Dxyz(20.0, 200.0, 200.0); // Damping gains for 3D control
    Eigen::MatrixXd positionDamping = Dxyz.asDiagonal();      
    //* Orientation stiffness and damping matrix
    Eigen::Vector3d Korientation(300.0, 300.0, 300.0); // Orientation stiffness gains
    Eigen::MatrixXd orientationStiffness = Korientation.asDiagonal();
    Eigen::Vector3d Dorientation(20.0, 20.0, 20.0); // Orientation damping gains
    Eigen::MatrixXd orientationDamping = Dorientation.asDiagonal();
    Eigen::MatrixXd stiffnessMatrix(6,6);
    stiffnessMatrix.block<3,3>(0,0) = positionStiffness;
    stiffnessMatrix.block<3,3>(3,3) = orientationStiffness;
    //! Velocity in Z
    double velocity_x_nominal = 0.03; // m/s
    double velocity_x_cmd = 0.0; 
    double gain_force_x = 0.0008; // N to m/s
    double desired_force = 50; //N
    std::chrono::high_resolution_clock::time_point last_force_control_sampling = std::chrono::high_resolution_clock::now(); 
    std::chrono::high_resolution_clock::time_point last_interpolation = std::chrono::high_resolution_clock::now(); 
    std::chrono::high_resolution_clock::time_point initial_sampling = std::chrono::high_resolution_clock::now(); 
    bool goHome = true;
    Eigen::Vector<double, 6>  HOME;
    //HOME  << 3.14, 2.516, 1.003, 3.14, 2.005, 0.0; 
    HOME  << 3.10, 2.50, 0.4, 3.1, 1.4, 0.0;
    bravo->set_interpolated_q(bravo->get_bravo_joint_states());

    while (rclcpp::ok()) {

        double ratio_force_manipulability = ratio_force_ellipsoid (bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states()));

        if ((bravo->compute_manipulability()>25) || (ratio_force_manipulability > 5.0) || goHome){
            if (bravo->compute_manipulability()>25){
                std::cout << "High manipulability: Going Home" << std::endl;
                goHome = true;
            }
            if (ratio_force_manipulability > 5.0){
                std::cout << "Bad force manipulability" << std::endl;
                goHome = true;
            }
            std::chrono::duration<double> elapsed_seconds = std::chrono::high_resolution_clock::now() - last_interpolation;
            double duration = bravo_utils::VAL_SAT<double>(elapsed_seconds.count(), 0.01, 0.0);
            Eigen::VectorXd error = bravo->go_to_configuration(HOME, 0.15 * Eigen::VectorXd::Ones(6), elapsed_seconds.count());            
            last_interpolation = std::chrono::high_resolution_clock::now();
            if ((error.array().abs() < 1e-2).all())
            {
                goHome = false;
                bravo->set_interpolated_q(bravo->get_bravo_joint_states());
                std::tie(ref_ee_pos, ref_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(bravo->get_bravo_joint_states());
            }
            last_force_control_sampling = std::chrono::high_resolution_clock::now(); 
            //std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 0.5 seconds
            // Eigen::VectorXd tau_gravity = bravo->manipulator_kin_dyn.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));
            // Eigen::VectorXd mA_joint_gravity_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
            // bravo->cmdJointCurrent_SAT(mA_joint_gravity_current_cmd, 1500.0);
            // stop_requested = true;
            // break; // exit loop gracefully
        }
        else if ((bravo->compute_manipulability()<=25) && (airbus_joy->enableBaseMotion)) {
            std::cout << "Teleoperating" << std::endl;
            //& JACOBIAN
            Eigen::Matrix<double, 6, 6> Jacobian;
            Jacobian = bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states());
            //& TWIST 
            Eigen::Vector<double, 6> twist_joy_fixed;
            float reduced_vel_factor = 0.9;
            float reduce_w_factor = 0.2; 
            twist_joy_fixed << airbus_joy->teleop_VelZ, airbus_joy->teleop_VelX, airbus_joy->teleop_VelY, airbus_joy->teleop_Wx, airbus_joy->teleop_Wy, airbus_joy->teleop_Wz;
            twist_joy_fixed.head<3>() = twist_joy_fixed.head<3>() * reduced_vel_factor; // Reduce linear velocity
            twist_joy_fixed.tail<3>() = twist_joy_fixed.tail<3>() * reduce_w_factor; // Reduce angular velocity
            bravo->move_ee_twist(twist_joy_fixed, Jacobian, 0.05);
            std::tie(ref_ee_pos, ref_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(bravo->get_bravo_joint_states());
            last_force_control_sampling = std::chrono::high_resolution_clock::now(); 
            last_interpolation = std::chrono::high_resolution_clock::now();
            bravo->set_interpolated_q(bravo->get_bravo_joint_states());
        }
        else{
            //std::cout << "Impedance" << std::endl;
            //& STIFFNESS CONTROL
            //* CALCULATE POSITION AND ORIENTATION ERROR
            std::tie(current_ee_pos, current_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(bravo->get_bravo_joint_states());
            Eigen::Vector3d position_error_world = ref_ee_pos - current_ee_pos;
            //* ORIENTATION ERROR 
            Eigen::Matrix3d R_err = ref_ee_rot * current_ee_rot.transpose();
            Eigen::Vector3d orientation_error;
            orientation_error << 
                0.5 * (R_err(2,1) - R_err(1,2)),
                0.5 * (R_err(0,2) - R_err(2,0)),
                0.5 * (R_err(1,0) - R_err(0,1));

            //* FORCE ESTIMATION and UPDATE VELOCITY REFERENCE (ADMITTANCE CONTROL IN Z)
            double force_x = position_error_world[0] * Kxyz[0]; // Simple spring model in Z
            velocity_x_cmd = velocity_x_nominal + gain_force_x * (desired_force - force_x); //! Adjust velocity based on force error
            double vel = bravo_utils::VAL_SAT<double>(velocity_x_cmd, 0.15, -0.15);
            std::chrono::duration<double> elapsed_seconds = std::chrono::high_resolution_clock::now() - last_force_control_sampling;
            ref_ee_pos[0] += vel * elapsed_seconds.count(); // Update reference position in Z
            last_force_control_sampling = std::chrono::high_resolution_clock::now(); 
            ref_ee_vel << vel, 0.0, 0.0; // Moving forward in Z with compliance
            Eigen:l:VectorXd twist_command(6);
            twist_command << vel, 0.0, 0.0, 0.0, 0.0, 0.0; // Desired twist in end-effector frame
            joint_velocity_cmd = bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states()).transpose() * twist_command;
            //* CALCULATE VELOCITY AND ANGULAR VELOCITY ERROR
            Eigen::VectorXd current_vel_twist(6);
            current_vel_twist = bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states()) * bravo->get_bravo_joint_velocities();
            Eigen::Vector3d vel_error_mobile = ref_ee_vel - current_vel_twist.head<3>();
            Eigen::Vector3d w_error_mobile   = ref_ee_w - current_vel_twist.tail<3>();
            //*TASK SPACE STIFFNESS CONTROL LAW
            Eigen::VectorXd task_space_stiffness(6);
            task_space_stiffness.head<3>() = positionStiffness * position_error_world;
            task_space_stiffness.tail<3>() = orientationStiffness * orientation_error;
            //* TASK SPACE DAMPING CONTROL LAW
            Eigen::VectorXd task_space_damping(6);
            task_space_damping.head<3>() = positionDamping * vel_error_mobile;
            task_space_damping.tail<3>() = orientationDamping * w_error_mobile;
            //* COMBINE STIFFNESS AND DAMPING FOR TORQUE COMMAND
            Eigen::VectorXd wrench_command = task_space_stiffness + task_space_damping;
            //wrench_command.tail<3>().setZero(); // No orientation control for simplicity //! TO BE REMOVED

            Eigen::VectorXd mA_stiction_stribeck_custom   =  compute_I_ff_stribeck_smooth(bravo->get_bravo_joint_velocities(), coulomb_custom, stiction_custom, 0.045, 100.0, 0.03);
            Eigen::VectorXd tau_gravity = bravo->manipulator_kin_dyn.invDynamics(bravo->get_bravo_joint_states(), joint_velocity_cmd, Eigen::VectorXd::Zero(6));
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
            mA_joint_current_cmd = mA_joint_current_cmd.array() + mA_stiction_stribeck_custom.array();

            joint_torque_cmd = bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states()).transpose() * wrench_command;
            joint_current_cmd = bravo->torqueNm_2_currentmA(joint_torque_cmd) + mA_joint_current_cmd;
        
            bool exceeds_limit = false;
            for (int i = 0; i < 6; i++) {
                if (std::abs(joint_current_cmd[i]) > max_current_mA) {
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
            bravo->cmdJointCurrent_SAT(joint_current_cmd, max_current_mA);
            bravo->publish_bravo_joint_states();
            bravo->set_joint_whole_integration(bravo->get_bravo_joint_states());
            bravo->set_interpolated_q(bravo->get_bravo_joint_states());
            last_interpolation = std::chrono::high_resolution_clock::now();
        }
    }
}

int main(int argc, char ** argv)
{
        rclcpp::init(argc, argv);
        const std::string urdf_filename = std::string("/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_amir_2.urdf");        
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



