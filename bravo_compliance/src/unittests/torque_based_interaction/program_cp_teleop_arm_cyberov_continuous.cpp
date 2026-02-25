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
    //& PARAMETERS
    double MAX_CURRENT_mA = 2500.0;
    double DESIRED_FORCE  = 50.0; //N
    double VELOCITY_X_NOMINAL = 0.09; // m/s
    double GAIN_FORCE_X = 0.1; // N to m/s
    double MAX_MANIPULABILITY = 25.0;
    double MAX_RATIO_FORCE_ELLIPSOID = 5.0;
    double MAX_SAT_SPEED = 0.15; // m/s
    double SPEED_GO_HOME = 0.15; // adimensional? I don't know
    Eigen::Vector<double, 6>  HOME;
    HOME  << 3.10, 2.50, 0.4, 3.1, 1.4, 0.0;

    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests  
        Eigen::Vector3d gravity_vector;
        gravity_vector << 0.0, 0.0, 9.81;              //! GRAVITY VECTOR FOR ARM MOUNTING POINT
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
    Eigen::MatrixXd stiffnessMatrix(6,6);
    Eigen::Vector3d Kxyz(700.0, 8000.0, 8000.0); // Stiffness gains for 3D control
    Eigen::MatrixXd positionStiffness = Kxyz.asDiagonal();
    Eigen::Vector3d Dxyz(20.0, 200.0, 200.0); // Damping gains for 3D control
    Eigen::MatrixXd positionDamping = Dxyz.asDiagonal(); 
    stiffnessMatrix.block<3,3>(0,0) = positionStiffness;     
    //* Orientation stiffness and damping matrix
    Eigen::Vector3d Korientation(300.0, 300.0, 300.0); // Orientation stiffness gains
    Eigen::MatrixXd orientationStiffness = Korientation.asDiagonal();
    Eigen::Vector3d Dorientation(20.0, 20.0, 20.0); // Orientation damping gains
    Eigen::MatrixXd orientationDamping = Dorientation.asDiagonal();
    stiffnessMatrix.block<3,3>(3,3) = orientationStiffness;

    //! Velocity in Z    
    double velocity_x_cmd = 0.0; 
    double velocity_y_cmd = 0.0; 
    double velocity_z_cmd = 0.0;     
    std::chrono::high_resolution_clock::time_point last_force_control_sampling = std::chrono::high_resolution_clock::now(); 
    std::chrono::high_resolution_clock::time_point last_interpolation = std::chrono::high_resolution_clock::now(); 
    std::chrono::high_resolution_clock::time_point initial_sampling = std::chrono::high_resolution_clock::now(); 
    bool goHome = true;    
    Eigen::VectorXd twist_command(6);
    bravo->set_interpolated_q(bravo->get_bravo_joint_states());

    while (rclcpp::ok()) {

        double ratio_force_manipulability = ratio_force_ellipsoid (bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states()));

        if ((bravo->compute_manipulability()>MAX_MANIPULABILITY) || (ratio_force_manipulability > MAX_RATIO_FORCE_ELLIPSOID) || goHome){
            if (bravo->compute_manipulability()>MAX_MANIPULABILITY){
                std::cout << "High manipulability: Going Home" << std::endl;
                goHome = true;
            }
            if (ratio_force_manipulability > MAX_RATIO_FORCE_ELLIPSOID){
                std::cout << "Bad force manipulability" << std::endl;
                goHome = true;
            }
            std::chrono::duration<double> elapsed_seconds = std::chrono::high_resolution_clock::now() - last_interpolation;
            double duration = bravo_utils::VAL_SAT<double>(elapsed_seconds.count(), 0.01, 0.0);
            Eigen::VectorXd error = bravo->go_to_configuration(HOME, SPEED_GO_HOME * Eigen::VectorXd::Ones(6), elapsed_seconds.count());            
            last_interpolation = std::chrono::high_resolution_clock::now();
            if ((error.array().abs() < 1e-2).all())
            {
                goHome = false;
                bravo->set_interpolated_q(bravo->get_bravo_joint_states());
                std::tie(ref_ee_pos, ref_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(bravo->get_bravo_joint_states());
            }
            last_force_control_sampling = std::chrono::high_resolution_clock::now(); 
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

            if (airbus_joy->enableBaseMotion){
                Kxyz << 700.0, 2000.0, 2000.0;
                positionStiffness = Kxyz.asDiagonal();
                Dxyz << 20.0, 50.0, 50.0; // Damping gains for 3D control
                positionDamping = Dxyz.asDiagonal(); 
                Eigen::Vector<double, 6> twist_joy_fixed;
                float reduced_vel_factor = 0.9;
                float reduce_w_factor = 0.2; 
                twist_joy_fixed << airbus_joy->teleop_VelZ, airbus_joy->teleop_VelX, airbus_joy->teleop_VelY, airbus_joy->teleop_Wx, airbus_joy->teleop_Wy, airbus_joy->teleop_Wz;
                twist_joy_fixed.head<3>() = twist_joy_fixed.head<3>() * reduced_vel_factor; // Reduce linear velocity
                twist_joy_fixed.tail<3>() = twist_joy_fixed.tail<3>() * reduce_w_factor; // Reduce angular velocity
                //* FORCE ESTIMATION and UPDATE VELOCITY REFERENCE (ADMITTANCE CONTROL IN Z)
                double force_x = position_error_world[0] * Kxyz[0]; // Simple spring model in Z
                double force_y = position_error_world[1] * Kxyz[1]; // Simple spring model in Z
                double force_z = position_error_world[2] * Kxyz[2]; // Simple spring model in Z
                // velocity_x_cmd = twist_joy_fixed[0] + GAIN_FORCE_X * (DESIRED_FORCE - force_x); //! Adjust velocity based on force error
                // velocity_y_cmd = twist_joy_fixed[1] + GAIN_FORCE_X * (70.0 - force_y); //! Adjust velocity based on force error
                // velocity_z_cmd = twist_joy_fixed[2] + GAIN_FORCE_X * (70.0 - force_z); //! Adjust velocity based on force error
                velocity_x_cmd = twist_joy_fixed[0]; //! Adjust velocity based on force error
                velocity_y_cmd = twist_joy_fixed[1]; //! Adjust velocity based on force error
                velocity_z_cmd = twist_joy_fixed[2]; //! Adjust velocity based on force error                
                double vel_x = bravo_utils::VAL_SAT<double>(velocity_x_cmd, MAX_SAT_SPEED, -MAX_SAT_SPEED);
                double vel_y = bravo_utils::VAL_SAT<double>(velocity_y_cmd, MAX_SAT_SPEED, -MAX_SAT_SPEED);
                double vel_z = bravo_utils::VAL_SAT<double>(velocity_z_cmd, MAX_SAT_SPEED, -MAX_SAT_SPEED);
                std::chrono::duration<double> elapsed_seconds = std::chrono::high_resolution_clock::now() - last_force_control_sampling;
                ref_ee_pos[0] += vel_x * elapsed_seconds.count(); // Update reference position in Z
                ref_ee_pos[1] += vel_y * elapsed_seconds.count(); // Update reference position in Z
                ref_ee_pos[2] += vel_z * elapsed_seconds.count(); // Update reference position in Z
                last_force_control_sampling = std::chrono::high_resolution_clock::now(); 
                ref_ee_vel << vel_x, vel_y, vel_z; // Moving forward in Z with compliance
                twist_command << vel_x, vel_y, vel_z, airbus_joy->teleop_Wx, airbus_joy->teleop_Wy, airbus_joy->teleop_Wz; // Desired twist in end-effector frame

            }
            else{
                Kxyz << 500.0, 5000.0, 5000.0;
                positionStiffness = Kxyz.asDiagonal();
                Dxyz << 30.0, 200.0, 200.0; // Damping gains for 3D control
                positionDamping = Dxyz.asDiagonal(); 
                //* FORCE ESTIMATION and UPDATE VELOCITY REFERENCE (ADMITTANCE CONTROL IN Z)
                double factor = 2.0; 
                double force_x = position_error_world[0] * Kxyz[0] * factor; // Simple spring model in Z
                velocity_x_cmd = VELOCITY_X_NOMINAL + GAIN_FORCE_X * (DESIRED_FORCE - force_x); //! Adjust velocity based on force error
                std::cout << "Velocity_nominal " << VELOCITY_X_NOMINAL << " Other component: " << GAIN_FORCE_X * (DESIRED_FORCE - force_x)<< std::endl; 
                double vel_x = bravo_utils::VAL_SAT<double>(velocity_x_cmd, MAX_SAT_SPEED, -MAX_SAT_SPEED);
                std::chrono::duration<double> elapsed_seconds = std::chrono::high_resolution_clock::now() - last_force_control_sampling;
                ref_ee_pos[0] += vel_x * elapsed_seconds.count(); // Update reference position in Z
                std::cout << "ref_ee_pos{0} " << ref_ee_pos[0] << std::endl;
                last_force_control_sampling = std::chrono::high_resolution_clock::now(); 
                ref_ee_vel << vel_x, 0.0, 0.0; // Moving forward in Z with compliance
                twist_command << vel_x, 0.0, 0.0, 0.0, 0.0, 0.0; // Desired twist in end-effector frame
            }

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



