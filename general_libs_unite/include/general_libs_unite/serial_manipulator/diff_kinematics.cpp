/**
 *    @file  diff_kinematics.cpp
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

#include "general_libs_unite/serial_manipulator/diff_kinematics.h"

template <typename T> 
     diff_kinematics<T>::diff_kinematics(int n_joints_robot_) : n_joints_robot(n_joints_robot_){
          twist_3D << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
          twist_3D_mask << 1, 1, 1, 1, 1, 1;
          joint_position_integration.resize(n_joints_robot); 
          joint_velocity_integration.setZero(n_joints_robot);  // I think this resizes and initializes
          joint_penalties.setOnes(n_joints_robot);
     }

template <typename T> 
     void diff_kinematics<T>::set_joint_penalization(Eigen::Vector<T, Eigen::Dynamic> joint_penalties_input){
          joint_penalties = joint_penalties_input; //! A SIZE CHECK SHOULD BE PERFORMED
     }

template <typename T> 
     void diff_kinematics<T>::set_twist_3D_mask(Eigen::Vector<int, 6> twist_3D_mask_input){
          // Force all non-zero to 1, zero stays zero
          twist_3D_mask = (twist_3D_mask_input.array() != 0).cast<int>();
     }

template <typename T> 
     Eigen::Vector<T, Eigen::Dynamic> diff_kinematics<T>::get_pos_joint_integration(){
          return joint_position_integration;
     }

template <typename T> 
     void diff_kinematics<T>::set_pos_joint_integration(Eigen::Vector<T, Eigen::Dynamic> q){
          if (q.size() != joint_position_integration.size()) {
               throw std::invalid_argument("[Diff_kinematics]: Dimension mismatch: input vector size does not match joint_position_integration size.");
          }
          joint_position_integration = q;
     }

template <typename T> 
     Eigen::Vector<T, Eigen::Dynamic> diff_kinematics<T>::get_vel_joint_integration(){
          return joint_velocity_integration;
     }

template <typename T> 
     void diff_kinematics<T>::set_vel_joint_integration(Eigen::Vector<T, Eigen::Dynamic> q){
          if (q.size() != joint_velocity_integration.size()) {
               throw std::invalid_argument("[Diff_kinematics]: Dimension mismatch: input vector size does not match joint_velocity_integration size.");
          }
          joint_velocity_integration = q;
     }

template <typename T>            
     Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> diff_kinematics<T>::PseudoInverse(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrixInput){
          Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrixInputInv;
          matrixInputInv = matrixInput.completeOrthogonalDecomposition().pseudoInverse();
          return matrixInputInv;
     }

template <typename T>           
     T diff_kinematics<T>::cond_arm(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> J){ 
          Eigen::JacobiSVD<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> svd(J);
          return svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
     }

/**
 * @brief Velocity Priority-based differential inverse kinematics with null-space projection
 * the twist_3D_mask is used to configured what dimensions are to be defined in the twist_3D task
 * @param [in] joint_position_integration current whole-body joint position
 * @return whole-body joint velocity 
 *  TODO: WORKING
 */
template <typename T>   
     Eigen::Vector<T, Eigen::Dynamic> diff_kinematics<T>::robot_ik_diff(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian_Input){
          int task_dimensions = twist_3D_mask.sum();   
          Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian(task_dimensions, Jacobian_Input.cols());  
          Eigen::Vector<T, Eigen::Dynamic> twist_3D_task(task_dimensions);       
          int j = 0;
          for (int i = 0; i < twist_3D_mask.size(); ++i) {
               if (twist_3D_mask(i) == 1) {
                    Jacobian.row(j) = Jacobian_Input.row(i);
                    twist_3D_task(j) = twist_3D(i);
                    j++;
               }
          }
          Eigen::Vector<T, Eigen::Dynamic> vel_output_joint;
          vel_output_joint = PseudoInverse(Jacobian) * twist_3D_task;		
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
     T diff_kinematics<T>::weight_joint_limit_smooth(T q, T qmin, T qinf, T qsup, T qmax) {
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
     void diff_kinematics<T>::motion_integration_loop(double sampling_time, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian){
          std::chrono::duration<double> elapsed_integration= std::chrono::high_resolution_clock::now()-last_integration_time;
          float safety_factor = 3.0;
          if ((elapsed_integration.count() > safety_factor*sampling_time))
          {
               start_integration_time = std::chrono::high_resolution_clock::now();
               last_integration_time = std::chrono::high_resolution_clock::now();
               std::cerr << "[Diff_kinematics]:❗Dangerous elapsed time much higher than predefined sampling time\n";

          }
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
     void diff_kinematics<T>::motion_integration(double elapsed, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian){
          joint_velocity_integration = robot_ik_diff(Jacobian);
          joint_position_integration = joint_position_integration + joint_velocity_integration*elapsed;
     }

     /**
      * @brief Reset on motion integration of the differential IK
      * @param [in] q_arm reset to input arm configuration
      *  TODO: WORKING
      */
template <typename T>
     void diff_kinematics<T>::reset_integration(Eigen::Vector<T, Eigen::Dynamic> q_arm){
          if (q_arm.size() != n_joints_robot) {
               throw std::invalid_argument("[Diff_kinematics]: ❌ Dimension mismatch: input arm vector size does not match expected size.");
          }
          else{
               joint_position_integration = q_arm;
               joint_velocity_integration.setZero();
               start_integration_time = std::chrono::high_resolution_clock::now();
               last_integration_time  = std::chrono::high_resolution_clock::now();
          }
     }

template class diff_kinematics<double>;