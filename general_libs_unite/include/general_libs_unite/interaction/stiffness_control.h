/**
 *    @file  stiffness_control.h
 *    @brief Stiffnes control 
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
#ifndef _STIFFNESS_CONTROL_
#define _STIFFNESS_CONTROL_
#include <chrono>
#include <math.h>
#include <exception>
#include <type_traits>

#include "general_libs_unite/general_utils/general_utils.h"
#include "general_libs_unite/serial_manipulator/diff_kinematics.h"
#include "general_libs_unite/serial_manipulator/kinodynamics_manipulator.h"

using namespace general_utils;
using namespace Eigen;

enum class armModel{bravo5, bravo7};

template <FloatingPoint T> 
    class stiffness_control{
        private:            
            Eigen::Vector<T, Eigen::Dynamic> joint_position_integration, joint_velocity_integration, joint_penalties;
            Eigen::Matrix<T, 6, 6> stiffness_matrix;
            std::chrono::high_resolution_clock::time_point last_computed_action;
            std::size_t n_joints_robot;
            //& POSITION
            Eigen::Matrix<T, 3, 3> pos_stiff_matrix, pos_damp_matrix;
            Eigen::Vector<T, 3> Kposition{700.0, 8000.0, 8000.0};
            Eigen::Vector<T, 3> Dposition{20.0, 200.0, 200.0};

            //& ORIENTATION
            Eigen::Vector<T, 3> Korientation{300.0, 300.0, 300.0};
            Eigen::Vector<T, 3> Dorientation{20.0, 20.0, 20.0};
            Eigen::Matrix<T, 3, 3> orient_stiff_matrix, orient_damping_matrix;

            //& Z-axis interaction params
            Eigen::Vector<T, 3> nominal_velocity{0.0, 0.0, 0.0};
            Eigen::Vector<T, 3> gain_force{0.0, 0.0, 0.0};
            Eigen::Vector<T, 3> desired_force{0.0, 0.0, 0.0};
            Eigen::Vector<T, 3> ref_ee_pos;    //! REQUIRES careful initialization
            Eigen::Matrix<T, 3, 3> ref_ee_rot; //! REQUIRES careful initialization
            bool ref_initialized = false;

            //& SAFETY LIMITS
            Eigen::Vector<T, 3> MAX_TASK_VEL{0.2, 0.2, 0.2}; 
            T MIN_SAMPLING_TIME = 0.05; //seconds
            T MAX_TORQUE = 3.0; //Nm //! MAYBE DELETE

            Eigen::Vector<T, 6>   twist_3D;
            
            //& OUTPUT ACTION
            Eigen::Vector<T, 6> wrench_action;

        private:
            void motion_integration(double elapsed, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian);
            
            Eigen::Vector<T, Eigen::Dynamic> robot_ik_diff(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian_Input);

            T compute_Z_compliance_ratio(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian);
      
            void compute_motion(Eigen::Vector<T, 6> stiffness_wrench);

        public:
            stiffness_control();

            void set_pos_stiffness(Eigen::Vector<T, 3> Kposition_input);

            void set_desired_force(Eigen::Vector<T, 3> desired_force_input);

            void set_ref_ee_pose(Eigen::Vector<T, 3> position, Eigen::Matrix<T,3,3> orientation);

            void set_pos_damping(Eigen::Vector<T, 3> Dposition_input);

            void set_orient_stiffness(Eigen::Vector<T, 3> Korientation_input);

            void set_orient_damping(Eigen::Vector<T, 3> Dorientation_input);

            Eigen::Vector<T, 6> get_twist_3D();

            Eigen::Vector<T, 6> compute_wrench(Eigen::Vector<T, 6> SE3_pos_error, Eigen::Vector<T, 6> vel_error);
    };

#endif 