/**
 *    @file  null_space_projection.cpp
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

#include "general_libs_unite/whole_body_control/null_space_projection.h"

template <typename T> 
     null_space_projection<T>::null_space_projection(int n_joints_robot_) : n_joints_robot(n_joints_robot_){
          primary_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
          primary_vel_mask << 1, 1, 1, 1, 1, 1;
          joint_position_integration.resize(n_joints_robot); 
          joint_velocity_integration.setZero(n_joints_robot);  // I think this resizes and initializes
          joint_penalties.setOnes(n_joints_robot);
          secondary_vel.setZero(n_joints_robot);
     }

template <typename T> 
     void null_space_projection<T>::set_joint_penalization(Eigen::Vector<T, Eigen::Dynamic> joint_penalties_input){
          joint_penalties = joint_penalties_input; //! A SIZE CHECK SHOULD BE PERFORMED
     }

template <typename T> 
     void null_space_projection<T>::set_primary_vel_mask(Eigen::Vector<int, 6> primary_mask_input){
          // Force all non-zero to 1, zero stays zero
          primary_vel_mask = (primary_mask_input.array() != 0).cast<int>();
     }

template <typename T> 
     void null_space_projection<T>::set_primary_and_secondary(Eigen::Vector<T, 6> primary_input, Eigen::Vector<T, Eigen::Dynamic> secondary_input){
          primary_vel = primary_input;
          secondary_vel = secondary_input;
     }

template <typename T> 
     Eigen::Vector<T, Eigen::Dynamic> null_space_projection<T>::get_pos_joint_integration(){
          return joint_position_integration;
     }

template <typename T> 
     void null_space_projection<T>::set_pos_joint_integration(Eigen::Vector<T, Eigen::Dynamic> q){
          if (q.size() != joint_position_integration.size()) {
               throw std::invalid_argument("[Diff_kinematics]: Dimension mismatch: input vector size does not match joint_position_integration size.");
          }
          joint_position_integration = q;
     }

template <typename T> 
     Eigen::Vector<T, Eigen::Dynamic> null_space_projection<T>::get_vel_joint_integration(){
          return joint_velocity_integration;
     }

template <typename T> 
     void null_space_projection<T>::set_vel_joint_integration(Eigen::Vector<T, Eigen::Dynamic> q){
          if (q.size() != joint_velocity_integration.size()) {
               throw std::invalid_argument("[Diff_kinematics]: Dimension mismatch: input vector size does not match joint_velocity_integration size.");
          }
          joint_velocity_integration = q;
     }

template <typename T>            
     Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> null_space_projection<T>::PseudoInverse(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrixInput){
          Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrixInputInv;
          matrixInputInv = matrixInput.completeOrthogonalDecomposition().pseudoInverse();
          return matrixInputInv;
     }

template <typename T>           
     T null_space_projection<T>::cond_arm(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> J){ 
          Eigen::JacobiSVD<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> svd(J);
          return svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
     }

/**
 * @brief Velocity Priority-based differential inverse kinematics with null-space projection
 * the primary_vel_mask is used to configured what dimensions are to be defined in the primary task
 * @param [in] joint_position_integration current whole-body joint position
 * @return whole-body velocity output or error 
 *  TODO: WORKING
 */
template <typename T>   
     Eigen::Vector<T, Eigen::Dynamic> null_space_projection<T>::robot_ik_diff(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian_Input){
          int task_dimensions = primary_vel_mask.sum();   
          Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian(task_dimensions, Jacobian_Input.cols());  
          Eigen::Vector<T, Eigen::Dynamic> primary_vel_task(task_dimensions);       
          int j = 0;
          for (int i = 0; i < primary_vel_mask.size(); ++i) {
               if (primary_vel_mask(i) == 1) {
                    Jacobian.row(j) = Jacobian_Input.row(i);
                    primary_vel_task(j) = primary_vel(i);
                    j++;
               }
          }
          Eigen::Vector<T, Eigen::Dynamic> vel_output_joint;
          //& Weight Matrix penalization for each joint
          Eigen::DiagonalMatrix<T, Eigen::Dynamic> WeightMatrix(joint_penalties);
          Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> invWeightMatrix;                    
          invWeightMatrix = WeightMatrix.inverse();
          //& Damping Least Square to handling singularities
          T epsilon = 0.045;
          T delta_max = 0.2;//0.06
          Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> localJacobian;
          localJacobian = Jacobian;
          Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> localJacobianT;
          localJacobianT = localJacobian.transpose();    
          Eigen::JacobiSVD<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> svd(localJacobian);
          T lowest_sing_value = svd.singularValues()(task_dimensions-1); 
          T delta;
          if (lowest_sing_value >= epsilon){
               delta = 0;
          }
          else{
               delta = (1 - pow((lowest_sing_value/epsilon), 2))*pow(delta_max, 2);
          }
          Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> component0;
          Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> component022;
          Eigen::MatrixXd component02, component021;
          Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> component01;
          component01 = (invWeightMatrix*localJacobianT);			
          component021 = (localJacobian*invWeightMatrix)*localJacobianT + pow(delta, 2)*MatrixXd::Identity(task_dimensions, task_dimensions);
          component02 = PseudoInverse(component021); 
          component022 = component02; 
          component0 = component01*component022;
          Eigen::Matrix<T, Eigen::Dynamic, 1> component1;               
          component1 = component0 * primary_vel_task;
          Eigen::Matrix<T, Eigen::Dynamic, 1> component2;
          Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> component21, I_njoints;
          component21 = (component0 * localJacobian);
          I_njoints = Eigen::MatrixXd::Identity(n_joints_robot, n_joints_robot);
          component2 = (I_njoints - component21) * secondary_vel;
          vel_output_joint = component1 + component2;			
          return vel_output_joint;
     }

     /**
      * @brief Joint Penalization function to avoid joint limits
      * @param [in] q current position
      * @param [in] q_min minimum limit
      * @param [in] q_max maximum limit
      * @param [in] q_inf infimum limit, smooths the curve before reaching minimum limit
      * @param [in] q_sup superior limit, smooths the curve before reaching maximum limit
      * @return penalization value correspondent to q joint value 
      *  TODO: WORKING
      */
template <typename T> 
     T null_space_projection<T>::weight_joint_limit_smooth(T q, T qmin, T qinf, T qsup, T qmax) {
          T ais = 2;
          T bis = -3 * (qmax + qsup);
          T cis = 6 * qmax * qsup;
          T dis = pow(qmax, 3) - 3 * pow(qmax, 2) * qsup;
          T aii = 2;
          T bii = -3 * (qmin + qinf);
          T cii = 6 * qmin * qinf;
          T dii = pow(qmin, 3) - 3 * pow(qmin, 2) * qinf;
          T w;
          if ((q >= qsup) && (q <= qmax)) {
          w = (1 / pow((qmax - qsup), 3)) * (ais * pow(q, 3) + bis * pow(q, 2) + cis * q + dis);
          } 
          else if ((q >= qmin) && (q <= qinf)) {
          w = (1 / pow((qmin - qinf), 3)) * (aii * pow(q, 3) + bii * pow(q, 2) + cii * q + dii);
          } 
          else if ((q > qinf) && (q < qsup)) {
          w = 1;
          } 
          else {
          w = pow(10, 40);
          }
          return w;
     }

     /**
      * @brief Motion integration of the differential IK
      * @param [in] sampling_time time resolution for continuous integration
      * @param [in] Jacobian used jacobian 
      *  TODO: WORKING
      */
template <typename T>
     void null_space_projection<T>::motion_integration_loop(double sampling_time, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian){
          std::chrono::duration<double> elapsed_integration= std::chrono::high_resolution_clock::now()-last_integration_time;
          if (elapsed_integration.count() > sampling_time) {
               finish_integration_time = std::chrono::high_resolution_clock::now();
               std::chrono::duration<double> elapsed = finish_integration_time - start_integration_time;
               motion_integration(elapsed.count(), Jacobian);
               start_integration_time = std::chrono::high_resolution_clock::now();
               last_integration_time = std::chrono::high_resolution_clock::now();
          }
     }

     /**
      * @brief Motion integration of the differential IK
      * @param [in] sampling_time time resolution for continuous integration
      * @param [in] Jacobian used jacobian 
      *  TODO: WORKING
      */
template <typename T>
     void null_space_projection<T>::motion_integration(double elapsed, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian){
          joint_velocity_integration = robot_ik_diff(Jacobian);
          joint_position_integration = joint_position_integration + joint_velocity_integration*elapsed;
     }

     /**
      * @brief Reset on motion integration of the differential IK
      * @param [in] q_arm reset to input arm configuration
      *  TODO: WORKING
      */
template <typename T>
     void null_space_projection<T>::reset_integration(Eigen::Vector<T, Eigen::Dynamic> q_arm){
          // Completely reset joint integration
          joint_position_integration = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(joint_position_integration.size());

          // If base joints should be 0 and arm joints should be set to q_arm:
          joint_position_integration.segment(4, q_arm.size()) = q_arm;

          // Optionally also reset any velocities or integration-related state
          joint_velocity_integration.setZero();
          start_integration_time = std::chrono::high_resolution_clock::now();
          last_integration_time = std::chrono::high_resolution_clock::now();
     }

template class null_space_projection<double>;