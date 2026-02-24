/**
 *    @file  stiffness_control.cpp
 *    @brief Priority-based differential inverse kinematics with 
      null-space projection in velocity
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  30-Oct-2024
 *    Modification 20-Nov-2025
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */

#include "general_libs_unite/interaction/stiffness_control.h"

template <FloatingPoint T> 
     stiffness_control<T>::stiffness_control(){
          twist_3D << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
          pos_stiff_matrix = Kposition.asDiagonal();
          pos_damp_matrix  = Dposition.asDiagonal();
          orient_stiff_matrix = Korientation.asDiagonal();
          orient_damping_matrix = Dorientation.asDiagonal();
          last_computed_action = std::chrono::high_resolution_clock::now();
     }

template <FloatingPoint T> 
     T stiffness_control<T>::compute_Z_compliance_ratio(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian){
          Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian_linear = Jacobian.topRows(3);
          // Perform SVD: Jlin = U * S * V^T
          Eigen::JacobiSVD<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> svd(Jacobian_linear, Eigen::ComputeThinU | Eigen::ComputeThinV);
          Eigen::Vector<T, Eigen::Dynamic> s = svd.singularValues();
          // Numerical safeguard
          T eps = 1e-12;
          for (int i = 0; i < s.size(); i++) {
               if (s(i) < eps) s(i) = eps;
          }
               // Motion and force ellipsoid radii
          Eigen::Vector<T, Eigen::Dynamic> motion_radii = s;           // proportional to velocity amplification
          Eigen::Vector<T, Eigen::Dynamic> force_radii  = s.cwiseInverse();  // reciprocal for force transmission
          T ratio = abs(force_radii[2]/ force_radii[0]);
          return ratio;
     }

template <FloatingPoint T> 
     void stiffness_control<T>::set_desired_force(Eigen::Vector<T, 3> desired_force_input){
          desired_force = desired_force_input;
     }

template <FloatingPoint T> 
     void stiffness_control<T>::set_ref_ee_pose(Eigen::Vector<T, 3> position, Eigen::Matrix<T,3,3> orientation){
          ref_ee_pos = position;
          ref_ee_rot = orientation; 
          ref_initialized = true;
          last_computed_action = std::chrono::high_resolution_clock::now();
     }

template <FloatingPoint T> 
     Eigen::Vector<T, 6> stiffness_control<T>::get_twist_3D(){
          return twist_3D;
     }

template <FloatingPoint T>
     Eigen::Vector<T, 6> stiffness_control<T>::compute_wrench(Eigen::Vector<T, 6> SE3_pos_error, Eigen::Vector<T, 6> vel_error){
          // Z-axis compliance control
          if(ref_initialized){
               //*TASK SPACE STIFFNESS CONTROL LAW
               Eigen::Vector<T, 6> task_space_stiffness, wrench_action;
               task_space_stiffness.template head<3>() = Kposition.cwiseProduct(SE3_pos_error.template head<3>());
               task_space_stiffness.template tail<3>() = Korientation.cwiseProduct(SE3_pos_error.template tail<3>());
               //* TASK SPACE DAMPING CONTROL LAW
               Eigen::Vector<T, 6> task_space_damping;
               task_space_damping.template head<3>() = Dposition.cwiseProduct(vel_error.template head<3>());
               task_space_damping.template tail<3>() = Dorientation.cwiseProduct(vel_error.template tail<3>());
               //* UPDATE MOTION REFERENCE
               compute_motion(task_space_stiffness);
               //* COMBINE STIFFNESS AND DAMPING FOR TORQUE COMMAND
               wrench_action = task_space_stiffness + task_space_damping;
               return wrench_action;
          }
          else{
               if (!ref_initialized){
                    std::cerr << "[Stiffness_control]: ❌ Reference end-effector pose not initialized. Call set_ref_ee_pose() before computing joint currents." << std::endl;
               }
               return Eigen::Vector<T, 6>::Zero();
          }
     }

template <FloatingPoint T>
     void stiffness_control<T>::compute_motion(Eigen::Vector<T, 6> stiffness_wrench){   
          Eigen::Vector<T, 3> vel_cmd; 
          vel_cmd = nominal_velocity - gain_force.cwiseProduct(desired_force - stiffness_wrench.template head<3>()); // task_space_stiffness.head<3>() is the estimated exerted force
          for (int i = 0; i < 3; ++i){
               twist_3D(i) = VAL_SAT<T>(vel_cmd(i), MAX_TASK_VEL(i), -MAX_TASK_VEL(i));
          }
          T duration = VAL_SAT<T>(std::chrono::duration<T>(std::chrono::high_resolution_clock::now() - last_computed_action).count(), MIN_SAMPLING_TIME, 0.0);
          ref_ee_pos += twist_3D.template head<3>() * duration; // Update reference position
          last_computed_action = std::chrono::high_resolution_clock::now();
     }

template class stiffness_control<double>;