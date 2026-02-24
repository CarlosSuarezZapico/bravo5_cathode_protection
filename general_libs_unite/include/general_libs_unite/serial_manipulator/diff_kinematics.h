/**
 *    @file  diff_kinematics.cpp
 *    @brief Differential inverse kinematics for serial manipulators
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      28-Nov-2025
 *    Modification 28-Nov-2025
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#ifndef _DIFF_KINEMATICS_
#define _DIFF_KINEMATICS_
#include <chrono>
#include <math.h>
#include <exception>
#include <type_traits>

#include "general_libs_unite/general_utils/general_utils.h"
#include "general_libs_unite/serial_manipulator/diff_kinematics.h"

using namespace general_utils;
using namespace Eigen;

template <typename T>
concept FloatingPoint = std::same_as<T, float> || std::same_as<T, double>;

template <FloatingPoint T> 
    class diff_kinematics{
        private:            
            Eigen::Vector<T, Eigen::Dynamic> joint_position_integration, joint_velocity_integration, joint_penalties;
            Eigen::Vector<T, 6> twist_3D;
            Eigen::Vector<int, 6> twist_3D_mask;
            std::chrono::high_resolution_clock::time_point last_integration_time, finish_integration_time, start_integration_time;
            std::size_t n_joints_robot;
        
        private:
            void motion_integration(double elapsed, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian);
            
            Eigen::Vector<T, Eigen::Dynamic> robot_ik_diff(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian_Input);

            void set_pos_joint_integration(Eigen::Vector<T, Eigen::Dynamic> q);

            void set_vel_joint_integration(Eigen::Vector<T, Eigen::Dynamic> q);
        
        public:
            diff_kinematics(int n_joints_robot_);

            void set_joint_penalization(Eigen::Vector<T, Eigen::Dynamic> joint_penalties_input);

            void set_twist_3D_mask(Eigen::Vector<int, 6> primary_mask_input);

            Eigen::Vector<T, Eigen::Dynamic> get_pos_joint_integration();

            Eigen::Vector<T, Eigen::Dynamic> get_vel_joint_integration();           

            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PseudoInverse(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrixInput);
            
            T cond_arm(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> J);

            T weight_joint_limit_smooth(T q, T qmin, T qinf, T qsup, T qmax);

            void motion_integration_loop(double sampling_time, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian);

            void reset_integration(Eigen::Vector<T, Eigen::Dynamic> q_arm);
    };

#endif 