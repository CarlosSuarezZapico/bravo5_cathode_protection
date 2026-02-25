/**
 *    @file  compliance_3D_stiffness_control.cpp
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
#include "general_libs_unite/general_utils/general_utils.h"

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

    Eigen::VectorXd joint_current_cmd = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd joint_torque_cmd(6);

    bool low_cond = false;
    //& TASK SPACE STIFFNESS CONTROL
    //* Position stiffness and damping matrix
    Eigen::Vector3d Kxyz(600.0, 3000.0, 3000.0); // Stiffness gains for 3D control
    Eigen::MatrixXd positionStiffness = Kxyz.asDiagonal();
    Eigen::Vector3d Dxyz(10.0, 10.0, 10.0); // Damping gains for 3D control
    Eigen::MatrixXd positionDamping = Dxyz.asDiagonal();      
    //* Orientation stiffness and damping matrix
    Eigen::Vector3d Korientation(100.0, 100.0, 100.0); // Orientation stiffness gains
    Eigen::MatrixXd orientationStiffness = Korientation.asDiagonal();
    Eigen::Vector3d Dorientation(5.0, 5.0, 5.0); // Orientation damping gains
    Eigen::MatrixXd orientationDamping = Dorientation.asDiagonal();
    Eigen::MatrixXd stiffnessMatrix(6,6);
    stiffnessMatrix.block<3,3>(0,0) = positionStiffness;
    stiffnessMatrix.block<3,3>(3,3) = orientationStiffness;

    while (rclcpp::ok()) {
        //& STIFFNESS CONTROL
        //* CALCULATE POSITION AND ORIENTATION ERROR
        std::tie(current_ee_pos, current_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(bravo->get_bravo_joint_states());
        Eigen::Vector3d position_error_world = ref_ee_pos - current_ee_pos;
        //!ORIENTATION ERROR 
        Eigen::Matrix3d R_err = ref_ee_rot * current_ee_rot.transpose();
        Eigen::Vector3d orientation_error;
        orientation_error << 
            0.5 * (R_err(2,1) - R_err(1,2)),
            0.5 * (R_err(0,2) - R_err(2,0)),
            0.5 * (R_err(1,0) - R_err(0,1));
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
        Eigen::VectorXd tau_gravity = bravo->manipulator_kin_dyn.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));
        mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
        mA_joint_current_cmd = mA_joint_current_cmd.array() + mA_stiction_stribeck_custom.array();

        if (bravo->compute_manipulability()>25){
            std::cout << "High manipulability" << std::endl;
            joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
        }
        else{
            joint_torque_cmd = bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states()).transpose() * wrench_command;
            joint_current_cmd = bravo->torqueNm_2_currentmA(joint_torque_cmd) + mA_joint_current_cmd;
        }

        std::cout << "Current [0]: " << joint_current_cmd[0] <<" [1]: "<< joint_current_cmd[1] <<" [2]: "<< joint_current_cmd[2] << " [3]: " <<
                        joint_current_cmd[3] <<" [4]: "<< joint_current_cmd[4] <<" [5]: "<< joint_current_cmd[5] <<std::endl;
        bravo->cmdJointCurrent_SAT(joint_current_cmd, 1500.0);
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



