/**
 *    @file  null_space_projection.cpp
 *    @brief Priority-based differential inverse kinematics with 
      null-space projection in velocity
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      30-Oct-2024
 *    Modification 30-Oct-2024
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#ifndef _NULL_SPACE_PROJECTION_
#define _NULL_SPACE_PROJECTION_
#include <chrono>
#include <math.h>
#include <exception>

#include "general_libs_unite/general_utils/general_utils.h"

using namespace std;
using namespace general_utils;
using namespace Eigen;

template <typename T> 
    class null_space_projection{
        private:            
            Eigen::Vector<T, Eigen::Dynamic> joint_position_integration, joint_velocity_integration, joint_penalties, secondary_vel;
            Eigen::Vector<T, 6> primary_vel;
            Eigen::Vector<int, 6> primary_vel_mask;
            std::chrono::high_resolution_clock::time_point last_integration_time, finish_integration_time, start_integration_time;
            std::size_t n_joints_robot;
        
        public:
            null_space_projection(int n_joints_robot_);

            void set_joint_penalization(Eigen::Vector<T, Eigen::Dynamic> joint_penalties_input);

            void set_primary_vel_mask(Eigen::Vector<int, 6> primary_mask_input);

            void set_primary_and_secondary(Eigen::Vector<T, 6> primary_input, Eigen::Vector<T, Eigen::Dynamic> secondary_input);

            Eigen::Vector<T, Eigen::Dynamic> get_pos_joint_integration();

            Eigen::Vector<T, Eigen::Dynamic> get_vel_joint_integration();
            
            void set_pos_joint_integration(Eigen::Vector<T, Eigen::Dynamic> q);

            void set_vel_joint_integration(Eigen::Vector<T, Eigen::Dynamic> q);
            
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PseudoInverse(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrixInput);
            
            T cond_arm(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> J);

            Eigen::Vector<T, Eigen::Dynamic> robot_ik_diff(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian_Input);

            T weight_joint_limit_smooth(T q, T qmin, T qinf, T qsup, T qmax);

            void motion_integration_loop(double sampling_time, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian);

            void motion_integration(double elapsed, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian);

            void reset_integration(Eigen::Vector<T, Eigen::Dynamic> q_arm);
    };

#endif 