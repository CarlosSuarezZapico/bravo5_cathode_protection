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
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

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


void program_loop(std::shared_ptr<airbus_joystick_bravo_twist_ee> airbus_joy, std::shared_ptr<bravo7_handler<double>> bravo){     
    auto start_time = std::chrono::steady_clock::now();
    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests      

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

    bravo->set_control_mode(set_bravo_control_mode::current_control);

    //&INITIAL POSITION AND POSE REFERENCE
    Eigen::VectorXd ref_joint_pos = bravo->get_bravo_joint_states();
    Eigen::Vector3d ref_ee_pos, current_ee_pos;
    Eigen::Vector3d ref_ee_vel, current_ee_vel, ref_ee_w, current_ee_w;
    Eigen::Matrix3d ref_ee_rot, current_ee_rot;
    Eigen::Vector3d position_error_fixed, force_estimation, position_error_global;
    std::tie(ref_ee_pos, ref_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(ref_joint_pos);
    ref_ee_vel << 0.0, 0.0, 0.0;
    ref_ee_w   << 0.0, 0.0, 0.0;

    mA_joint_current_cmd = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd joint_torque_cmd(6), joint_velocity_cmd(6);

    bool low_cond = false;
    //& TASK SPACE STIFFNESS CONTROL
    //* Position stiffness and damping matrix
    Eigen::Vector3d Kxyz(100.0, 3000.0, 3000.0); // Stiffness gains for 3D control
    Eigen::MatrixXd positionStiffness = Kxyz.asDiagonal();
    Eigen::Vector3d Dxyz(10.0, 30.0, 30.0); // Damping gains for 3D control
    Eigen::MatrixXd positionDamping = Dxyz.asDiagonal();      
    //* Orientation stiffness and damping matrix
    Eigen::Vector3d Korientation(100.0, 100.0, 100.0); // Orientation stiffness gains
    Eigen::MatrixXd orientationStiffness = Korientation.asDiagonal();
    Eigen::Vector3d Dorientation(5.0, 5.0, 5.0); // Orientation damping gains
    Eigen::MatrixXd orientationDamping = Dorientation.asDiagonal();
    Eigen::MatrixXd stiffnessMatrix = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd dampingMatrix   = Eigen::MatrixXd::Zero(6,6);
    stiffnessMatrix.block<3,3>(0,0) = positionStiffness;
    stiffnessMatrix.block<3,3>(3,3) = orientationStiffness;
    dampingMatrix.block<3,3>(0,0) = positionDamping;
    dampingMatrix.block<3,3>(3,3) = orientationDamping;

    //! Velocity in Z
    double velocity_x_nominal = 0.03; // m/s
    double velocity_x_cmd = 0.0; 
    double gain_force_x = 0.0008; // N to m/s
    double desired_force = 25; //N
    std::chrono::high_resolution_clock::time_point last_sampling = std::chrono::high_resolution_clock::now(); 
    std::chrono::high_resolution_clock::time_point initial_sampling = std::chrono::high_resolution_clock::now(); 
    Eigen::VectorXd xdd_des(6); xdd_des.setZero();
    
    double regularization = 1e-6; 

    while (rclcpp::ok()) {
       

        if (bravo->compute_manipulability()>25){
            std::cout << "High manipulability -> entering in gravity_compensation" << std::endl;
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(bravo->manipulator_kin_dyn.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)));
        }
        else{
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
            Eigen::VectorXd x_err(6);
            x_err.head<3>() = position_error_world;
            x_err.tail<3>() = orientation_error;
            
            //* JACOBIAN
            Eigen::MatrixXd J = bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states());

            //* JACOBIAN TIME DERIVATIVE
            Eigen::MatrixXd dJ_analytical = bravo->manipulator_kin_dyn.fixedJacobianDerivative(bravo->get_bravo_joint_states(), bravo->get_bravo_joint_velocities());
            Eigen::VectorXd Jdot_v = dJ_analytical * bravo->get_bravo_joint_velocities();

            //* CALCULATE VELOCITY AND ANGULAR VELOCITY ERROR
            Eigen::VectorXd current_vel_twist(6);
            current_vel_twist = J * bravo->get_bravo_joint_velocities();
            Eigen::Vector3d vel_error_mobile = ref_ee_vel - current_vel_twist.head<3>();
            Eigen::Vector3d w_error_mobile   = ref_ee_w - current_vel_twist.tail<3>();
            Eigen::VectorXd xdot_err(6);
            xdot_err.head<3>() = vel_error_mobile;
            xdot_err.tail<3>() = w_error_mobile;
            
            //* MASS MATRIX IN JOINT SPACE
            Eigen::MatrixXd Mass_joint_space = bravo->manipulator_kin_dyn.mass_joint_space(bravo->get_bravo_joint_states());
            Mass_joint_space.triangularView<Eigen::StrictlyLower>() = Mass_joint_space.transpose().triangularView<Eigen::StrictlyLower>();
            Eigen::MatrixXd M_reg = Mass_joint_space + regularization * Eigen::MatrixXd::Identity(6, 6);
            Eigen::MatrixXd M_inv = M_reg.inverse();
            // 6) Task-space inertia Lambda
            MatrixXd Mass_task_space_inv = J * M_inv * J.transpose();
            MatrixXd Mass_task_space = (Mass_task_space_inv + regularization * MatrixXd::Identity(6,6)).inverse();

            //* NONLINEAR EFFECTS
            Eigen::VectorXd nle = bravo->manipulator_kin_dyn.nle(bravo->get_bravo_joint_states(), bravo->get_bravo_joint_velocities());

            //* COMMAND ACCELERATION
            Eigen::VectorXd a_star = xdd_des - Jdot_v + stiffnessMatrix * x_err + dampingMatrix * xdot_err;

            //* TWO OPTIONS HERE: COMPUTE THE FORCE USING A SPECIFIC DESIRED INERTIA OR USING ROBOT'S INERTIA
            // 11) Desired task wrench
            // Option A: use actual Mass_task_space
            Eigen::VectorXd F_task = Mass_task_space * a_star;

            // Option B: use desired inertia M_d
            //Eigen::VectorXd F_task = M_d * a_star;

            Eigen::VectorXd mA_stiction_stribeck_custom   =  compute_I_ff_stribeck_smooth(bravo->get_bravo_joint_velocities(), coulomb_custom, stiction_custom, 0.045, 100.0, 0.03);
            Eigen::VectorXd tau_cmd = J.transpose() * F_task + nle;
            mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_cmd);
            mA_joint_current_cmd = mA_joint_current_cmd.array() + mA_stiction_stribeck_custom.array();

            //* FORCE ESTIMATION and UPDATE VELOCITY REFERENCE (ADMITTANCE CONTROL IN Z)
            double force_x = position_error_world[0] * Kxyz[0]; // Simple spring model in Z
            velocity_x_cmd = velocity_x_nominal + gain_force_x * (desired_force - force_x); //! Adjust velocity based on force error
            double vel = bravo_utils::VAL_SAT<double>(velocity_x_cmd, 0.15, -0.15);
            std::chrono::duration<double> elapsed_seconds = std::chrono::high_resolution_clock::now() - last_sampling;
            ref_ee_vel << vel, 0.0, 0.0;
            ref_ee_w << 0.0, 0.0, 0.0; 
            ref_ee_pos = ref_ee_pos + ref_ee_vel * elapsed_seconds.count(); // Update reference position in Z
            last_sampling = std::chrono::high_resolution_clock::now(); 
            // Moving forward in Z with compliance

        }

        std::cout << "Current [0]: " << mA_joint_current_cmd[0] <<" [1]: "<< mA_joint_current_cmd[1] <<" [2]: "<< mA_joint_current_cmd[2] << " [3]: " <<
                        mA_joint_current_cmd[3] <<" [4]: "<< mA_joint_current_cmd[4] <<" [5]: "<< mA_joint_current_cmd[5] <<std::endl;
        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, 1500.0);
        bravo->publish_bravo_joint_states();
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



