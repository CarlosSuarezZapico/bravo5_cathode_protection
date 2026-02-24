/**
 *    @file  stiffness_control_position.cpp
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

#include "general_libs_unite/interaction/stiffness_control_position.h"

template <FloatingPoint T> 
     stiffness_control_position<T>::stiffness_control_position(){
          vel_ee  << 0.0, 0.0, 0.0;
          pos_stiff_matrix = Kposition.asDiagonal();
          pos_damp_matrix  = Dposition.asDiagonal();
          last_computed_action = std::chrono::high_resolution_clock::now();
     }

template <FloatingPoint T> 
     T stiffness_control_position<T>::compute_Z_compliance_ratio(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian){
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
     std::tuple<T, T> stiffness_control_position<T>::compute_X_compliance_ratios(const Eigen::Matrix<T, 3, Eigen::Dynamic> Jacobian_linear){
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
          T ratioXy = abs(force_radii[0]/ force_radii[1]);
          T ratioXz = abs(force_radii[0]/ force_radii[2]);
          return std::make_tuple(ratioXy, ratioXz);
     }

template <FloatingPoint T> 
     void stiffness_control_position<T>::set_desired_force(Eigen::Vector<T, 3> desired_force_input){
          desired_force = desired_force_input;
     }

template <FloatingPoint T> 
     void stiffness_control_position<T>::set_gain_force(Eigen::Vector<T, 3> gain_force_input){
          gain_force = gain_force_input;
     }

template <FloatingPoint T> 
     void stiffness_control_position<T>::set_max_vel(Eigen::Vector<T, 3> maximum_velocity_input){
          MAX_TASK_VEL = maximum_velocity_input;
     }

template <FloatingPoint T> 
     void stiffness_control_position<T>::set_nominal_vel(Eigen::Vector<T, 3> nominal_vel_input){
          nominal_velocity = nominal_vel_input;
     }

template <FloatingPoint T> 
     void stiffness_control_position<T>::set_ref_ee_position(Eigen::Vector<T, 3> position){
          ref_ee_pos = position;
          ref_initialized = true;
          last_computed_action = std::chrono::high_resolution_clock::now();
     }

template <FloatingPoint T> 
     Eigen::Vector<T, 3> stiffness_control_position<T>::get_ref_ee_position(){
          return ref_ee_pos;
     }

template <FloatingPoint T> 
     Eigen::Vector<T, 3> stiffness_control_position<T>::get_vel_ee(){
          return vel_ee;
     }

template <FloatingPoint T>
     Eigen::Vector<T, 3> stiffness_control_position<T>::compute_force_action(Eigen::Vector<T, 3> pos_error, Eigen::Vector<T, 3> vel_error, bool compute_motion_ref){
          // Z-axis compliance control
          if(ref_initialized){
               //*TASK SPACE STIFFNESS CONTROL LAW
               Eigen::Vector<T, 3> task_space_stiffness, wrench_action;
               task_space_stiffness = Kposition.cwiseProduct(pos_error);
               //* TASK SPACE DAMPING CONTROL LAW
               Eigen::Vector<T, 3> task_space_damping;
               task_space_damping = Dposition.cwiseProduct(vel_error);

               //* COMBINE STIFFNESS AND DAMPING FOR TORQUE COMMAND
               wrench_action = task_space_stiffness + task_space_damping;
               if (compute_motion_ref == true){
                    //std::cout << "Computing motion reference with stiffness wrench: " << task_space_stiffness.transpose() << " and damping wrench: " << task_space_damping.transpose() << std::endl;
                    compute_motion(wrench_action);
               }
               return wrench_action;
          }
          else{
               if (!ref_initialized){
                    std::cerr << "[stiffness_control_position]: ❌ Reference end-effector pose not initialized. Call set_ref_ee_pose() before computing joint currents." << std::endl;
               }
               return Eigen::Vector<T, 3>::Zero();
          }
     }

template <FloatingPoint T>
     void stiffness_control_position<T>::set_pos_stiffness(Eigen::Vector<T, 3> Kposition_input){
          Kposition = Kposition_input;
          pos_stiff_matrix = Kposition.asDiagonal();
     }

template <FloatingPoint T> 
     void stiffness_control_position<T>::set_pos_damping(Eigen::Vector<T, 3> Dposition_input){
          Dposition = Dposition_input;
          pos_damp_matrix = Dposition.asDiagonal();
     }

template <FloatingPoint T>
     void stiffness_control_position<T>::compute_motion(Eigen::Vector<T, 3> exerted_force){   
          Eigen::Vector<T, 3> vel_cmd; 
          debug_stiffness = exerted_force(0);
          if (true){
              vel_cmd = -(nominal_velocity - gain_force.cwiseProduct(desired_force - exerted_force)); // task_space_stiffness.head<3>() is the estimated exerted force
          }
          else{
               double F_scale = std::max(0.5, std::fabs(desired_force[0]));
               double Vf = 1.0* nominal_velocity[0]; 
               vel_cmd[0] = -(nominal_velocity[0] - Vf*std::tanh((-desired_force[0] - exerted_force[0])/ F_scale));  // task_space_stiffness.head<3>() is the estimated exerted force
               vel_cmd[1] = 0.0;
               vel_cmd[2] = 0.0;
          }          
          for (int i = 0; i < 3; ++i){
               vel_ee(i) = VAL_SAT<T>(vel_cmd(i), MAX_TASK_VEL(i), -MAX_TASK_VEL(i));
          }
          T duration = VAL_SAT<T>(std::chrono::duration<T>(std::chrono::high_resolution_clock::now() - last_computed_action).count(), MIN_SAMPLING_TIME, 0.0);
          debug_duration = duration; 
          ref_ee_pos += vel_ee * duration; // Update reference position
          last_computed_action = std::chrono::high_resolution_clock::now();
     }

template <FloatingPoint T>
     void stiffness_control_position<T>::debug_controller(){     
          std::cout << "DEBUG: Stiffness Velocity: " << vel_ee(0) << " Pos Error: " << position_error <<  " Desired Force: " << desired_force(0) << " Stiffness Wrench: " << debug_stiffness << " Ref: " << ref_ee_pos(0) << " Duration: " << debug_duration << std::endl;
     }

template class stiffness_control_position<double>;