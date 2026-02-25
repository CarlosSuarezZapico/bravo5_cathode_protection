/**
 *    @file  gravity_compensation_and_stiction_stribeck_smooth.cpp
 *    @brief Unittest for the Bravo7 arm. The program implements gravity compensation
 *           plus stiction compensation using a smooth Stribeck model.
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  1-Sep-2025
 *    Modification 1-Sep-2025
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
    bravo->set_control_mode(set_bravo_control_mode::current_control);
    Eigen::VectorXd mA_joint_current_cmd(6), coulomb_custom(6), stiction_custom(6);
    coulomb_custom << 300.0, 200.0, 200.0, 200.0, 200.0, 200.0;
    stiction_custom << 350.0, 350.0, 350.0, 350.0, 350.0, 350.0;
    mA_joint_current_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    bool high_cond = false;
  
    while (rclcpp::ok()) {
        Eigen::VectorXd mA_stiction_stribeck_custom   =  compute_I_ff_stribeck_smooth(bravo->get_bravo_joint_velocities(), coulomb_custom, stiction_custom, 0.045, 100.0, 0.03);
        Eigen::VectorXd tau_gravity = bravo->manipulator_kin_dyn.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));
        mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
        mA_joint_current_cmd = mA_joint_current_cmd.array() + mA_stiction_stribeck_custom.array(); // Add stiction compensation
        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, 2000.0);
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



