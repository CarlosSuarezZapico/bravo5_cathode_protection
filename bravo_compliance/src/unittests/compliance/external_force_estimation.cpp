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
#include "bravo7_version_2/bravo_cpp/force_estimator/estimator.hpp"
#include "general_libs_unite/joysticks/airbus_joystick.h"
#include "general_libs_unite/interaction/interaction_control.h"
#include "general_libs_unite/general_utils/general_utils.h"
#include "bravo7_version_2/msg/stamped_float32_array.hpp"
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

void program_loop(std::shared_ptr<airbus_joystick_compliance> airbus_joy, std::shared_ptr<bravo7_handler<double>> bravo, rclcpp::Publisher<bravo7_version_2::msg::StampedFloat32Array>::SharedPtr pub){     
    auto start_time = std::chrono::steady_clock::now();
    const std::string urdf_filename_force = "/home/carlos/cp_unite_ws/src/robot_descriptions/bpl_bravo_description/urdf/bravo_7_amir_2.urdf";


    while (!bravo->isConnected()){
        bravo->set_bravo_frequency_packet_exchange(200); //! Set frequency of requests
        // Check for 5-second timeout
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
            std::cerr << "Timeout: Bravo connection failed after 5 seconds.\n";
            break; // or handle timeout as needed
        }
    }    
    bravo->set_control_mode(set_bravo_control_mode::current_control);
    //& FORCE ESTIMATOR
    Estimator amir_force_estimator(urdf_filename_force);
    Eigen::VectorXd ref_joint_pos = bravo->get_bravo_joint_states();
    Eigen::VectorXd torque_cmd(6);
    Eigen::VectorXd joint_position_cmd(6), joint_velocity_cmd(6);
    torque_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    bool high_cond = false;
    Eigen::VectorXd stiction(6), viscous_damping(6), coulomb(6);
    viscous_damping << 34.45, 27.30, 14.82, 11.39, 15.68, 18.30;
    //stiction << 350.0, 350.0, 350.0, 350.0, 350.0, 350.0; // Stiction values for each joint
    stiction << 350.0, 350.0, 350.0, 350.0, 350.0, 350.0; // Stiction values for each joint
    coulomb << 300.0, 200.0, 200.0, 200.0, 200.0, 200.0;
    Eigen::VectorXd pos_gains(6), pos_gains_contact(6), vel_gains(6); 
    //pos_gains_contact << 4000.0, 4000.0, 4000.0, 2500.0, 2500.0, 2500.0; //COMPLIANCE VALUES
    pos_gains << 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0;
    //vel_gains << 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0; // Gains for velocity control
    vel_gains << 200.0, 200.0, 200.0, 200.0, 200.0, 200.0; // Gains for velocity control
    //pos_gains <<10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0;
    Eigen::Vector3d ref_ee_pos, current_ee_pos;
    Eigen::Matrix3d ref_ee_rot, current_ee_rot;
    std::tie(ref_ee_pos, ref_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(ref_joint_pos);
    Eigen::Vector3d position_error_fixed, force_estimation, position_error_global;
    force_estimation << 0.0, 0.0, 0.0;
    //& INITIALIZE FORCE ESTIMATOR
    amir_force_estimator.initialize(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));
    
    while (rclcpp::ok()) {
        if (bravo->compute_manipulability()>25){
            torque_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            bravo->cmdJointCurrent_SAT(torque_cmd, 2000.0);
            std::cout << "Manipulability is high, no command sent." << std::endl;
        }
        else{
            //& UPDATE FORCE ESTIMATOR
            amir_force_estimator.update(bravo->get_bravo_joint_states(), bravo->get_bravo_joint_currents());
            
            Eigen::Vector<double, 6> twist_joy, twist_fixed;
            double reduced_vel_factor = 1.0;
            twist_joy << reduced_vel_factor*airbus_joy->teleop_VelX, reduced_vel_factor*airbus_joy->teleop_VelY, reduced_vel_factor*airbus_joy->teleop_VelZ, 0.0, 0.0, 0.0;
            std::cout << "Diff called" << std::endl;
            std::tie(high_cond, joint_position_cmd, joint_velocity_cmd) = bravo->twist_diff_kin_2(twist_joy, 0.02);
            std::cout << "Position [0]: " << twist_joy[0] <<" [1]: "<< twist_joy[1] <<" [2]: "<< twist_joy[2] << " [3]: " <<
                            twist_joy[3] <<" [4]: "<< twist_joy[4] <<" [5]: "<< twist_joy[5] <<std::endl;

            // torque_cmd = pos_gains.array() * (bravo->signedAngleDistance(joint_position_cmd, bravo->get_bravo_joint_states())).array() + 
            //              stiction.array() * (bravo->signedAngleDistance(joint_position_cmd, bravo->get_bravo_joint_states())).array().sign();

            Eigen::ArrayXd pos_err = bravo->signedAngleDistance(joint_position_cmd, bravo->get_bravo_joint_states()).array();
            Eigen::ArrayXd vel_err = bravo->signedAngleDistance(joint_velocity_cmd, bravo->get_bravo_joint_velocities()).array();
            Eigen::VectorXd tau_gravity = bravo->manipulator_kin_dyn.invDynamics(bravo->get_bravo_joint_states(), joint_velocity_cmd, Eigen::VectorXd::Zero(6));
            Eigen::VectorXd mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
            //Eigen::VectorXd mA_stiction = stiction.array() * smoothSignEigen(pos_err, 0.0015); // Smooth sign function for stiction
            //Eigen::VectorXd mA_stiction =  stiction.array() * (joint_velocity_cmd *50.0).array().tanh();
            Eigen::VectorXd mA_stiction_stribeck =  compute_I_ff_stribeck_smooth(bravo->get_bravo_joint_velocities(), coulomb, stiction, 0.045, 100.0, 0.03);
            Eigen::VectorXd tau_viscous_damping = viscous_damping.array() * joint_velocity_cmd.array();
            Eigen::VectorXd mA_viscous_damping = bravo->torqueNm_2_currentmA(tau_viscous_damping);

            torque_cmd = pos_gains.array() * pos_err + vel_gains.array() * vel_err + mA_stiction_stribeck.array() + mA_joint_current_cmd.array(); 
                        // mA_viscous_damping.array();  
            
            bravo->cmdJointCurrent_SAT(torque_cmd, 2000.0);

            //& ESTIMATED WRENCH END-EFFECTOR
            geometry_msgs::msg::Wrench wrench_estimated_free_motion = amir_force_estimator.get_estimated_frame_force_torque("EE"); // Estimated current due to motion on arm
            geometry_msgs::msg::Wrench full_wrench = amir_force_estimator.get_frame_force_torque("EE"); // Wrench readings
            geometry_msgs::msg::Wrench AmirEstimate; 
            //& Use a threshold of 30, 20 newtons
            AmirEstimate.force.x = full_wrench.force.x - wrench_estimated_free_motion.force.x; 
            AmirEstimate.force.y = full_wrench.force.y - wrench_estimated_free_motion.force.y; 
            AmirEstimate.force.z = full_wrench.force.z - wrench_estimated_free_motion.force.z; 
            AmirEstimate.torque.x = full_wrench.torque.x - wrench_estimated_free_motion.torque.x; 
            AmirEstimate.torque.y = full_wrench.torque.y - wrench_estimated_free_motion.torque.y; 
            AmirEstimate.torque.z = full_wrench.torque.z - wrench_estimated_free_motion.torque.z; 
            //& Publish the estimated wrench
            bravo->publish_wrench_estimation_2(AmirEstimate);

            //Eigen::VectorXd torque_fdb = bravo->get_bravo_joint_torques();
            //Eigen::MatrixXd jacobian = bravo->manipulator_kin_dyn.localJacobian(bravo->get_bravo_joint_states());
            //Eigen::VectorXd wrench = jacobian * torque_fdb;
            //bravo->publish_wrench_estimation(wrench);
            // //*EE position error
            // std::tie(current_ee_pos, current_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(bravo->get_bravo_joint_states());
            // std::tie(ref_ee_pos, ref_ee_rot) = bravo->manipulator_kin_dyn.FK_ee(joint_position_cmd);
            // position_error_fixed[0] = abs(ref_ee_pos[0] - current_ee_pos[0]);
            // position_error_fixed[1] = abs(ref_ee_pos[1] - current_ee_pos[1]);
            // position_error_fixed[2] = abs(ref_ee_pos[2] - current_ee_pos[2]);
            // std::cout << "Position Error Fixed [0]: " << position_error_fixed[0] <<" [1]: "<< position_error_fixed[1] <<" [2]: "<< position_error_fixed[2] << std::endl;
            // position_error_global = ref_ee_rot * position_error_fixed;
            // bravo->publish_force_estimation(position_error_fixed);
            // std::cout << "Position Error Global [0]: " << position_error_global[0] <<" [1]: "<< position_error_global[1] <<" [2]: "<< position_error_global[2] << std::endl;

            //& PUBLISH ALL MSGS
            Eigen::VectorXd joint_current_fdb = bravo->get_bravo_joint_currents();
            bravo7_version_2::msg::StampedFloat32Array all_msg = bravo7_version_2::msg::StampedFloat32Array();
            all_msg.header.stamp = rclcpp::Clock().now();            
            all_msg.data = {pos_err[0], pos_err[1], pos_err[2], pos_err[3], pos_err[4], pos_err[5], vel_err[0], vel_err[1], vel_err[2], vel_err[3], vel_err[4], vel_err[5], 
                           joint_current_fdb[0], joint_current_fdb[1], joint_current_fdb[2], joint_current_fdb[3], joint_current_fdb[4], joint_current_fdb[5], 
                           joint_position_cmd[0], joint_position_cmd[1], joint_position_cmd[2], joint_position_cmd[3], joint_position_cmd[4], joint_position_cmd[5],
                           joint_velocity_cmd[0], joint_velocity_cmd[1], joint_velocity_cmd[2], joint_velocity_cmd[3], joint_velocity_cmd[4], joint_velocity_cmd[5]};
            pub->publish(all_msg);
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
        executor->add_node(bravo);
        auto pub = bravo->create_publisher<bravo7_version_2::msg::StampedFloat32Array>("/force_estimation/all_data", 10);

        std::thread executor_thread([&executor]() {
                executor->spin();
        });
        program_loop(joystick, bravo, pub);
        executor_thread.join();
        rclcpp::shutdown();
        return 0;
}



