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

double sign(double x) {
    return (x > 0) - (x < 0);  // returns 1 if x>0, -1 if x<0, 0 if x==0
}

double compute_I_ff_stribeck(double qd, double I_coulomb, double I_stiction, double qd_stiction2) {
    double sgn = sign(qd);
    double exp_term = std::exp(-std::pow(std::abs(qd) / qd_stiction2, 2));
    double I_ff2 = (I_coulomb + (I_stiction - I_coulomb) * exp_term) * sgn;
    return I_ff2;
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
    Eigen::VectorXd mA_joint_current_cmd(6), stiction(6);
    stiction << 300.0, 300.0, 300.0, 300.0, 300.0, 300.0;
    mA_joint_current_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    bool high_cond = false;
  
    while (rclcpp::ok()) {
        Eigen::VectorXd mA_stiction =  stiction.array() * (bravo->get_bravo_joint_velocities() *100.0).array().tanh();
        //my_value = compute_I_ff_stribeck(fdb_data.angular_vel_rad_s, 14, 22, 0.005); *100.0).array().tanh();
        Eigen::VectorXd tau_gravity = bravo->manipulator_kin_dyn.invDynamics(bravo->get_bravo_joint_states(), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));
        mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
        mA_joint_current_cmd = mA_joint_current_cmd.array() + mA_stiction.array(); // Add stiction compensation
        // mA_joint_current_cmd =  (tau_gravity/(0.222*120.0))*1000.0;
        std::cout << "Torque Nm[0]: " << tau_gravity[0] <<" [1]: "<< tau_gravity[1] <<" [2]: "<< tau_gravity[2] << " [3]: " <<
        tau_gravity[3] <<" [4]: "<< tau_gravity[4] <<" [5]: "<< tau_gravity[5] <<std::endl;
        std::cout << "Current mA[0]: " << mA_joint_current_cmd[0] <<" [1]: "<< mA_joint_current_cmd[1] <<" [2]: "<< mA_joint_current_cmd[2] << " [3]: " <<
        mA_joint_current_cmd[3] <<" [4]: "<< mA_joint_current_cmd[4] <<" [5]: "<< mA_joint_current_cmd[5] <<std::endl;            
        bravo->cmdJointCurrent_SAT(mA_joint_current_cmd, 2000.0);
            
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



