/**
 *    @file   kinodynamics_manipulator.h
 *    @brief  Class handler for general purspose serial manipulator
 *            using eigen and pinocchio libraries
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Project UNITE
 *    Created      24-Nov-2023
 *    Modification 24-Nov-2025
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */

#ifndef _KINODYNAMICS_MANIPULATOR_
#define _KINODYNAMICS_MANIPULATOR_

#include "rclcpp/rclcpp.hpp"

#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp" 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/algorithm/crba.hpp>
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"

#include "bravo_compliance/bravo_cpp/utils/utils.h"

#include <iostream>
#include <chrono>

using namespace std;
using namespace Eigen;
using namespace pinocchio;
using namespace rclcpp;

template <bravo_control::Floating32or64 T_data>
class kinodynamics_manipulator {
	private:
	    std::vector<std::string> joint_names;
		std::string ee_frame;		
		//Pinocchio model
		Model model_manipulator;
		Data  data_manipulator;

	public:	 
		kinodynamics_manipulator(const std::string &urdf_filename, const std::string &toolLink): data_manipulator(){
			pinocchio::urdf::buildModel(urdf_filename, model_manipulator);
			ee_frame = toolLink;
			data_manipulator = Data(model_manipulator); 
			joint_names = model_manipulator.names; 			
		}
				
		Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> fixedJacobian(Eigen::Vector<T_data, Eigen::Dynamic> q,  const std::string frame_name){
			pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(frame_name);			
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>  J; // Jacobian matrix (6xN, where N is the number of joints)
			pinocchio::computeFrameJacobian (model_manipulator, data_manipulator, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
			return J;
		}

		Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> fixedJacobianPosition(Eigen::Vector<T_data, Eigen::Dynamic> q,  const std::string frame_name){
			pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(frame_name);			
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>  J; // Jacobian matrix (6xN, where N is the number of joints)
			pinocchio::computeFrameJacobian (model_manipulator, data_manipulator, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> Jpos = J.topRows(3); 
			return Jpos;
		}
		
		Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> fixedJacobian(Eigen::Vector<T_data, Eigen::Dynamic> q){
			pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(ee_frame);			
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>  J; // Jacobian matrix (6xN, where N is the number of joints)
			pinocchio::computeFrameJacobian (model_manipulator, data_manipulator, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
			return J;
		}

		Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> fixedJacobianDerivative(Eigen::Vector<T_data, Eigen::Dynamic> q, Eigen::Vector<T_data, Eigen::Dynamic> qdot){
			pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(ee_frame);			
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> dJ_analytical; 
			pinocchio::computeForwardKinematicsDerivatives(model_manipulator, data_manipulator, q, qdot, Eigen::Vector<T_data, Eigen::Dynamic>::Zero(model_manipulator.nv));
			pinocchio::getFrameJacobianTimeVariation(model_manipulator, data_manipulator, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, dJ_analytical);
			return dJ_analytical;			
		}
		
		Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> localJacobian(Eigen::Vector<T_data, Eigen::Dynamic> q, const std::string frame_name){
			pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(frame_name);			
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> J; 
			pinocchio::computeFrameJacobian (model_manipulator, data_manipulator, q, frame_id, pinocchio::LOCAL, J);
			return J;			
		}

		Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> localJacobianPosition(Eigen::Vector<T_data, Eigen::Dynamic> q, const std::string frame_name){
			pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(frame_name);			
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> J; 
			pinocchio::computeFrameJacobian (model_manipulator, data_manipulator, q, frame_id, pinocchio::LOCAL, J);
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> Jpos = J.topRows(3); 
			return Jpos;			
		}

		Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> localJacobian(Eigen::Vector<T_data, Eigen::Dynamic> q){
			pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(ee_frame);			
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> J; 
			pinocchio::computeFrameJacobian (model_manipulator, data_manipulator, q, frame_id, pinocchio::LOCAL, J);
			return J;			
		}

		Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> localJacobianDerivative(Eigen::Vector<T_data, Eigen::Dynamic> q, Eigen::Vector<T_data, Eigen::Dynamic> qdot){
			pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(ee_frame);			
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> dJ_analytical; 
			pinocchio::computeForwardKinematicsDerivatives(model_manipulator, data_manipulator, q, qdot, Eigen::Vector<T_data, Eigen::Dynamic>::Zero(model_manipulator.nv));
			pinocchio::getFrameJacobianTimeVariation(model_manipulator, data_manipulator, frame_id, pinocchio::LOCAL, dJ_analytical);
			return dJ_analytical;			
		}

		void change_gravity_vector(Eigen::Vector3d gravity_vector){			
			model_manipulator.gravity.linear() = gravity_vector;
		}

		Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> mass_joint_space(Eigen::Vector<T_data, Eigen::Dynamic> q){			
			pinocchio::crba(model_manipulator, data_manipulator, q);
			return data_manipulator.M;	
		}

		Eigen::Vector<T_data, Eigen::Dynamic> nle(Eigen::Vector<T_data, Eigen::Dynamic> q, Eigen::Vector<T_data, Eigen::Dynamic> v){			
			Eigen::Vector<T_data, Eigen::Dynamic> nle = pinocchio::nonLinearEffects(model_manipulator, data_manipulator, q, v);
			return nle;			
		}

		/**
		 * @brief Forward kinematics using pinocchio lib
		 * \todo NOT TESTED
		 * @param q joint pos
		 * @param JOINT_ID frame to which FK is done
		 * @return position an rotational matrix
		 */
		tuple<Eigen::Vector3d, Eigen::Matrix3d> FK(Eigen::Vector<T_data, Eigen::Dynamic> q, const int JOINT_ID){ 			
			forwardKinematics(model_manipulator,data_manipulator,q);
			Eigen::Vector3d & x   = data_manipulator.oMi[JOINT_ID].translation();
			Eigen::Matrix3d & R   = data_manipulator.oMi[JOINT_ID].rotation();
			return std::make_tuple(x, R);	
		}

		/**
		 * @brief Forward kinematics using pinocchio lib
		 * @param q joint pos
		 * @return position an rotational matrix of the end-effector frame
		 */
		tuple<Eigen::Vector3d, Eigen::Matrix3d> FK_ee(Eigen::Vector<T_data, Eigen::Dynamic> q){ 
			pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(ee_frame);			
			forwardKinematics(model_manipulator,data_manipulator,q);
			pinocchio::updateFramePlacements(model_manipulator, data_manipulator);
			Eigen::Vector3d & x   = data_manipulator.oMf[frame_id].translation();
			Eigen::Matrix3d & R   = data_manipulator.oMf[frame_id].rotation();
			return std::make_tuple(x, R);	
		}

		/**
		 * @brief Forward kinematics using pinocchio lib
		 * @param q joint pos
		 * @return position of the end-effector frame
		 */
		Eigen::Vector3d FK_ee_pos(const Eigen::VectorXd& q){
			const auto frame_id = model_manipulator.getFrameId(ee_frame);
			pinocchio::forwardKinematics(model_manipulator, data_manipulator, q);
			pinocchio::updateFramePlacements(model_manipulator, data_manipulator);
			return data_manipulator.oMf[frame_id].translation().template cast<double>();
		}

		/**
		 * @brief Compare two joint configurations and checks that none of the components 
		 *       is bigger  
		 * Todo: NOT TESTED
		 * @param[in] q0 joint pos 0
		 * @param[in] q1 joint pos 1
		 * @param[in] max_q_diff max angle allowed for all components
		 * @return bool, true means that there is a component at least with a bigger difference than max_q_diff 
		 */
        bool max_diff_joint_configurations(Eigen::Vector<T_data, Eigen::Dynamic> q0, Eigen::Vector<T_data, Eigen::Dynamic> q1, T_data max_q_diff){
			for (int i=0; i<q0.size(); i++){
				if (abs(q0[i]- q1[i]) > max_q_diff){
					return true;
				}
			}
			return false;
		}

		T_data cond_arm(Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> J){ 
			JacobiSVD<Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>> svd(J);
			return svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
		}

		Eigen::Vector<T_data, Eigen::Dynamic> invDynamics(Eigen::Vector<T_data, Eigen::Dynamic> q, Eigen::Vector<T_data, Eigen::Dynamic> v, Eigen::Vector<T_data, Eigen::Dynamic> a){
			Eigen::Vector<T_data, Eigen::Dynamic> tau;
			tau = pinocchio::rnea(model_manipulator, data_manipulator, q, v, a);
			return tau;
		}
		
		tuple<Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>> fwDynamicsAbaDerivatives(Eigen::Vector<T_data, Eigen::Dynamic> q, Eigen::Vector<T_data, Eigen::Dynamic> v, Eigen::Vector<T_data, Eigen::Dynamic> tau){
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> djoint_acc_dq = Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>::Zero(model_manipulator.nv,model_manipulator.nv);
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> djoint_acc_dv = Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>::Zero(model_manipulator.nv,model_manipulator.nv);
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> djoint_acc_dtau = Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>::Zero(model_manipulator.nv,model_manipulator.nv);	
			// Computes the forward dynamics (ABA) derivatives for all the joints of the robot
			pinocchio::computeABADerivatives(model_manipulator, data_manipulator, q, v, tau, djoint_acc_dq, djoint_acc_dv, djoint_acc_dtau);
			return {djoint_acc_dq, djoint_acc_dv, djoint_acc_dtau};
		}
		tuple<Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>> invDynamicsRNEADerivatives(Eigen::Vector<T_data, Eigen::Dynamic> q, Eigen::Vector<T_data, Eigen::Dynamic> v, Eigen::Vector<T_data, Eigen::Dynamic> a){
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> djoint_torque_dq = Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>::Zero(model_manipulator.nv,model_manipulator.nv);
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> djoint_torque_dv = Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>::Zero(model_manipulator.nv,model_manipulator.nv);
			Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> djoint_torque_da = Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic>::Zero(model_manipulator.nv,model_manipulator.nv);	
			// Computes the forward dynamics (ABA) derivatives for all the joints of the robot
			pinocchio::computeRNEADerivatives(model_manipulator, data_manipulator, q, v, a, djoint_torque_dq, djoint_torque_dv, djoint_torque_da);
			return {djoint_torque_dq, djoint_torque_dv, djoint_torque_da};
		}		

		Eigen::Vector<T_data, Eigen::Dynamic> compute_I_ff_stribeck_smooth(const Eigen::Vector<T_data, Eigen::Dynamic>& qd, const Eigen::Vector<T_data, Eigen::Dynamic>& I_coulomb, const Eigen::Vector<T_data, Eigen::Dynamic>& I_stiction, T_data v_stiction, T_data k,  T_data vel_eps){
			Eigen::Vector<T_data, Eigen::Dynamic> out(qd.size());
			for (int i = 0; i < qd.size(); ++i) {
				T_data v = qd[i];
				// Blending term: 1 / (1 + (|v| / v_stiction)^k)
				T_data blend = 1.0 / (1.0 + std::pow(std::abs(v) / v_stiction, k));
				// Smooth sign via tanh
				T_data sgn_smooth = std::tanh(v / vel_eps);
				// Combine
				out[i] = (I_coulomb[i] + (I_stiction[i] - I_coulomb[i]) * blend) * sgn_smooth;
			}
			return out;
		}

		Eigen::Vector<T_data, Eigen::Dynamic> compute_I_ff_stribeck_smooth(const Eigen::Vector<T_data, Eigen::Dynamic>& qd, const Eigen::Vector<T_data, Eigen::Dynamic>& I_coulomb, const Eigen::Vector<T_data, Eigen::Dynamic>& I_stiction, const Eigen::Vector<T_data, Eigen::Dynamic>& v_stiction, const Eigen::Vector<T_data, Eigen::Dynamic>& k,  const Eigen::Vector<T_data, Eigen::Dynamic>& vel_eps){
			Eigen::Vector<T_data, Eigen::Dynamic> out(qd.size());
			for (int i = 0; i < qd.size(); ++i) {
				T_data v = qd[i];
				// Blending term: 1 / (1 + (|v| / v_stiction)^k)
				T_data blend = 1.0 / (1.0 + std::pow(std::abs(v) / v_stiction[i], k[i]));
				// Smooth sign via tanh
				T_data sgn_smooth = std::tanh(v / vel_eps[i]);
				// Combine
				out[i] = (I_coulomb[i] + (I_stiction[i] - I_coulomb[i]) * blend) * sgn_smooth;
			}
			return out;
		}
	    
		Eigen::Vector<T_data, Eigen::Dynamic> compute_I_ff_stribeck(const Eigen::Vector<T_data, Eigen::Dynamic>& qd, const Eigen::Vector<T_data, Eigen::Dynamic>& I_coulomb,  const Eigen::Vector<T_data, Eigen::Dynamic>& I_stiction, const Eigen::Vector<T_data, Eigen::Dynamic>& qd_stiction2){
			Eigen::Vector<T_data, Eigen::Dynamic> sgn = qd.array().sign().matrix();
			Eigen::Vector<T_data, Eigen::Dynamic> exp_term = (-((qd.array().abs() / qd_stiction2.array()).square())).exp().matrix();
			Eigen::Vector<T_data, Eigen::Dynamic> I_ff = (I_coulomb.array() + (I_stiction.array() - I_coulomb.array()) * exp_term.array()).matrix();
			I_ff = I_ff.array() * sgn.array();
			return I_ff;
		}

		// Eigen::Vector<T_data, 6> torque_admittance_forceZ_controller(Eigen::Vector<T_data, 6> joint_pos_states, T_data desiredForce, T_data desired_vel, Eigen::Matrix<T_data, Eigen::Dynamic, Eigen::Dynamic> Jacobian, Eigen::Vector3d &ref_ee_pos, Eigen::Matrix3d &ref_ee_rot){
		// 	//& STIFFNESS CONTROL
        //     //* CALCULATE POSITION AND ORIENTATION ERROR
		// 	Eigen::Vector3d ref_ee_pos, current_ee_pos;
		// 	Eigen::Matrix3d ref_ee_rot, current_ee_rot;
        //     std::tie(current_ee_pos, current_ee_rot) = FK_ee(joint_pos_states);
        //     Eigen::Vector3d position_error_world = ref_ee_pos - current_ee_pos;
        //     //* ORIENTATION ERROR 
        //     Eigen::Matrix3d R_err = ref_ee_rot * current_ee_rot.transpose();
        //     Eigen::Vector3d orientation_error;
        //     orientation_error << 
        //         0.5 * (R_err(2,1) - R_err(1,2)),
        //         0.5 * (R_err(0,2) - R_err(2,0)),
        //         0.5 * (R_err(1,0) - R_err(0,1));
        //     //* FORCE ESTIMATION and UPDATE VELOCITY REFERENCE (ADMITTANCE CONTROL IN Z)
        //     T_data force_x = position_error_world[0] * Kxyz[0]; // Simple spring model in Z
        //     velocity_x_cmd = velocity_x_nominal + gain_force_x * (desired_force - force_x); //! Adjust velocity based on force error
        //     T_data vel = bravo_utils::VAL_SAT<T_data>(velocity_x_cmd, 0.15, -0.15);
        //     std::chrono::duration<T_data> elapsed_seconds = std::chrono::high_resolution_clock::now() - last_sampling;
        //     ref_ee_pos[0] += vel * elapsed_seconds.count(); // Update reference position in Z
        //     last_sampling = std::chrono::high_resolution_clock::now(); 
        //     ref_ee_vel << vel, 0.0, 0.0; // Moving forward in Z with compliance
        //     Eigen:l:Vector<T_data, Eigen::Dynamic> twist_command(6);
        //     twist_command << vel, 0.0, 0.0, 0.0, 0.0, 0.0; // Desired twist in end-effector frame
        //     joint_velocity_cmd = bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states()).transpose() * twist_command;
        //     //* CALCULATE VELOCITY AND ANGULAR VELOCITY ERROR
        //     Eigen::Vector<T_data, Eigen::Dynamic> current_vel_twist(6);
        //     current_vel_twist = bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states()) * bravo->get_bravo_joint_velocities();
        //     Eigen::Vector3d vel_error_mobile = ref_ee_vel - current_vel_twist.head<3>();
        //     Eigen::Vector3d w_error_mobile   = ref_ee_w - current_vel_twist.tail<3>();
        //     //*TASK SPACE STIFFNESS CONTROL LAW
        //     Eigen::Vector<T_data, Eigen::Dynamic> task_space_stiffness(6);
        //     task_space_stiffness.head<3>() = positionStiffness * position_error_world;
        //     task_space_stiffness.tail<3>() = orientationStiffness * orientation_error;
        //     //* TASK SPACE DAMPING CONTROL LAW
        //     Eigen::Vector<T_data, Eigen::Dynamic> task_space_damping(6);
        //     task_space_damping.head<3>() = positionDamping * vel_error_mobile;
        //     task_space_damping.tail<3>() = orientationDamping * w_error_mobile;
        //     //* COMBINE STIFFNESS AND DAMPING FOR TORQUE COMMAND
        //     Eigen::Vector<T_data, Eigen::Dynamic> wrench_command = task_space_stiffness + task_space_damping;
        //     //wrench_command.tail<3>().setZero(); // No orientation control for simplicity //! TO BE REMOVED

        //     Eigen::Vector<T_data, Eigen::Dynamic> mA_stiction_stribeck_custom   =  compute_I_ff_stribeck_smooth(bravo->get_bravo_joint_velocities(), coulomb_custom, stiction_custom, 0.045, 100.0, 0.03);
        //     Eigen::Vector<T_data, Eigen::Dynamic> tau_gravity = bravo->manipulator_kin_dyn.invDynamics(bravo->get_bravo_joint_states(), joint_velocity_cmd, Eigen::Vector<T_data, Eigen::Dynamic>::Zero(6));
        //     mA_joint_current_cmd = bravo->torqueNm_2_currentmA(tau_gravity);
        //     mA_joint_current_cmd = mA_joint_current_cmd.array() + mA_stiction_stribeck_custom.array();

        //     joint_torque_cmd = bravo->manipulator_kin_dyn.fixedJacobian(bravo->get_bravo_joint_states()).transpose() * wrench_command;
        //     joint_current_cmd = bravo->torqueNm_2_currentmA(joint_torque_cmd) + mA_joint_current_cmd;
		// }


};

template class kinodynamics_manipulator<T_data>;
template class kinodynamics_manipulator<float>;
#endif 
