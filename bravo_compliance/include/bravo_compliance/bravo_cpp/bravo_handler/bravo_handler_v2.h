/**
 *    @file  bravo_handler_v2.h
 *    @brief  Derived class handler for bravo7 arm from manipulator base class
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Project UNITE
 *    Created      17-Nov-2023
 *    Modification 25-Nov-2025
 *    Revision  ---
 *    Project   UNITE
 *    Compiler  gcc/g++
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#ifndef _BRAVO_HANDLER_V2_
#define _BRAVO_HANDLER_V2_

#include "rclcpp/rclcpp.hpp"
#include <cstdlib> 
#include <thread>
#include <atomic>
#include <cmath>
#include <memory>

#include <urdf/model.h>
#include <geometry_msgs/msg/twist.hpp>	
#include <geometry_msgs/msg/vector3.hpp>	
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>			
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "general_libs_unite/serial_manipulator/kinodynamics_manipulator.h"
#include "bravo_compliance/bravo_cpp/bravo_io_v2/bravo_udp_v2.h"
#include "bravo_compliance/bravo_cpp/utils/utils.h"

using namespace std;
using namespace Eigen;
using namespace bravo_utils;
using namespace bravo_control;

//& CONTROL MODES AND ARM STATES
enum set_bravo_control_mode {position_control, velocity_control, current_control, torque_control}; // Bravo list of control modes


template <bravo_control::Floating32or64 T_data>
	class bravo_handler : public rclcpp::Node{
		public: 
			//& PINOCCHIO LIBRARY FOR KINODYNAMICS
			kinodynamics_manipulator<T_data> kinodynamics;	
		private:
			//& LOW-LEVEL INTERFACE WITH BRAVO
			//bravo_udp<T_data> bravo_io; 
			std::unique_ptr<bravo_control::bravo_udp<T_data>> bravo_io;
			size_t number_joints; 
			bool joint_integration_inialized = false;	

			//& THREAD FOR BRAVO IO INPUT/OUTPUT
			std::thread bravo_io_thread;
			std::atomic<bool> bravo_io_run_thread;

			//& VARIABLE FOR SIMULATION
			Eigen::Vector<T_data, Eigen::Dynamic> sim_joint_whole_integration;
			std::chrono::high_resolution_clock::time_point sim_last_integration_time, sim_start_integration_time, sim_finish_integration_time;

			//& JOINT LIMITS
			Eigen::Vector<T_data, Eigen::Dynamic> upperJointLimits, upperJointInfLimits, lowerJointLimits, lowerJointInfLimits, maskJointLimits;
            //Eigen::Matrix<bool, -1, 1> _continuous_joints = Eigen::Matrix<bool, -1, 1>::Zero(6);
			Eigen::Array<bool, 6, 1>  _continuous;
			Eigen::Vector<T_data, Eigen::Dynamic> _upper_joint_limit;
			Eigen::Vector<T_data, Eigen::Dynamic> _lower_joint_limit;

			//& SAFETY VARIABLES - MAXIMUM JOINT VELOCITY & JOINT CURRENT
			Eigen::Vector<T_data, Eigen::Dynamic> max_q_vel;
			Eigen::Vector<T_data, Eigen::Dynamic> max_q_current;
			Eigen::Vector<T_data, Eigen::Dynamic> max_q_torque;

			//& CMD
			cmd_data<Eigen::Vector<T_data, 6>> cmdLocalTwist;  

			//& CURRENT MOTOR CONSTANT (Nm/A)
			Eigen::Vector<T_data, Eigen::Dynamic> motor_constants;
            
			//& JOINT VELOCITY LIMITS (rad/s) for interpolation
			Eigen::Vector<T_data, Eigen::Dynamic> _joint_vel_limits;
			Eigen::Vector<T_data, Eigen::Dynamic> interpolated_q_;

			//& DIFFERENTIAL KINEMATICS AND INTEGRATION VARIABLES
			Eigen::Vector<T_data, Eigen::Dynamic> joint_whole_integration, joint_vel_integration;

			//& TIMERS
			std::chrono::high_resolution_clock::time_point last_integration_time, start_integration_time, finish_integration_time;

			//& ROS PUBLISHERS SUBSCRIBBERS
			//$PUBLISHERS
			rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr       pubWrenchEstimation;
			rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr              pubWrenchEstimation_2;
			rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr      pubForceEstimation;
			rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr            pubFdbJointStates;
			rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr            pubFdbJointCurrents;
			rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr            pubFdbJointTorques;
			rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr            pubJointsStateRviz;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                  pubJointsStateFreq;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                  pubCurrentJointsFreq;
			rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                  pubFTFreq;
			
			bool print_warn_cond = true;
			T_data manipulability = 1000.0;
			T_data MAX_MANIPULABILITY = 25.0; //& Safety threshold for manipulability

		//MEMBER FUNCTIONS
		public:
			/**
			 * @brief Bravo7 constructor
			 * 
			 * @param urdf_filename urdf file location
			 * @param toolLink ee frame name
			 */
			bravo_handler(const std::string urdf_filename, const std::string &toolLink, bool real=true, bool ROS_enable = true);

			~bravo_handler();

			/**
			 * @brief Main low-layer interface for bravo IO. This thread is in charge on establish 
			 * a continous bilateral communication with the arm
			 */ 
			void bravo_io_thread_function();

			void set_control_mode(set_bravo_control_mode mode);

			void set_bravo_frequency_packet_exchange(T_data freq);

			void debug_jointFdb_last_msg_times();

			T_data get_manipulability();

			void set_interpolated_q(const Eigen::Vector<T_data, Eigen::Dynamic> &q);

			/**
			 * @brief Send joint position real robot(NOT THE GRIPPER)
			 * The function applies various safety checks before sending the joint command to the arm
			 * @param cmdJointPosition 
			 * @return void
			 */ 
			void cmdJointPos(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointPosition);

			Eigen::Vector<T_data, Eigen::Dynamic> std_2_eigen(std::vector<T_data> std_vector);

			Eigen::Vector<T_data, Eigen::Dynamic> torqueNm_2_currentmA(Eigen::Vector<T_data, Eigen::Dynamic> torqueNm);

			Eigen::Vector<T_data, Eigen::Dynamic> currentmA_2_torqueNm(Eigen::Vector<T_data, Eigen::Dynamic> currentmA);
			
			bool is_in_desired_configuration(T_data tolerance, Eigen::Vector<T_data, Eigen::Dynamic> goal_joint_position, Eigen::Vector<T_data, Eigen::Dynamic> current_joint_position);

			Eigen::Vector<T_data, Eigen::Dynamic> going2joint_interpolation(Eigen::Vector<T_data, Eigen::Dynamic> goal_joint_position, Eigen::Vector<T_data, Eigen::Dynamic> current_joint_position, T_data joint_vel, T_data sampling_time);
			
			bool go_to_JointPos(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointPosition, T_data joint_vel, T_data tolerance); 

			std::tuple<bool, bool, Eigen::Vector<T_data, Eigen::Dynamic>> go_to_joint_position(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdGoalJointPosition, T_data joint_vel, T_data tolerance, std::chrono::high_resolution_clock::time_point& last_call);

			void go_to_JointPos(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointPosition, T_data joint_vel);

            Eigen::Vector<T_data, Eigen::Dynamic> go_to_configuration(const Eigen::Vector<T_data, Eigen::Dynamic> desired_q, const Eigen::Vector<T_data, Eigen::Dynamic> v, const T_data dt);
			
			Eigen::Vector<T_data, Eigen::Dynamic> get_joint_error(const Eigen::Vector<T_data, Eigen::Dynamic> current_q, const Eigen::Vector<T_data, Eigen::Dynamic> desired_q);
 
			Eigen::Vector<T_data, Eigen::Dynamic> scale_joint_velocity(const Eigen::Vector<T_data, Eigen::Dynamic> dq);

			Eigen::Vector<T_data, Eigen::Dynamic> integrate_configuration(const Eigen::Vector<T_data, Eigen::Dynamic> current_q, const Eigen::Vector<T_data, Eigen::Dynamic> velocity);

			/**
			 * @brief Send joint current (NOT THE GRIPPER)
			 * @param cmdJointCurrent joint current cmd
			 * @return void
			 */
			void cmdJointCurrent(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointCurrent);

			void cmdJointCurrent_SAT(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointCurrent, const T_data MAX);

			void cmdJointTorque(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointTorqueIn);

			void cmdJointTorque_SAT(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointTorqueIn, const T_data MAX);

			/**
			 * @brief Send joint velocity (NOT THE GRIPPER)
			 * @param cmdJointVel joint vel
			 * @return void
			 */
			void cmdJointVel(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointVel);

			T_data compute_manipulability();

			T_data compute_manipulability_position();

			/**
			 * @brief Cmd twist end-effector local frame for real robot. The function makes an integration with the 
			 * differetial kinematics and send a whole-body joint position command to the arm.
			 * &NOT TESTED
			 * @param twist // The velocity that Bravo 7 receives must be in m/s and rad/s
			 * @param sampling_time integration time
			 * @return void
			 */
			void moveCmdTCPLocalTwist(const Eigen::Vector<T_data, Eigen::Dynamic> &twist, T_data sampling_time);

			/**
			 * @brief Cmd twist end-effector local frame for real robot. The function makes an integration with the 
			 * differetial kinematics and send a whole-body joint position command to the arm.
			 * &NOT TESTED
			 * @param twist // The velocity that Bravo 7 receives must be in m/s and rad/s
			 * @param sampling_time integration time
			 * @return bool, joint position command
			 */
			std::tuple<bool, Eigen::Vector<T_data, Eigen::Dynamic>> twist_diff_kin(const Eigen::Vector<T_data, Eigen::Dynamic> &twist, T_data sampling_time);
			std::tuple<bool, Eigen::Vector<T_data, Eigen::Dynamic>, Eigen::Vector<T_data, Eigen::Dynamic>> twist_diff_kin_2(const Eigen::Vector<T_data, Eigen::Dynamic> &twist, T_data sampling_time);

			/**
			 * @brief Cmd twist end-effector local frame for real robot. The output is the twist in the end-effector frame and
			 * a bravo instruction executes the motion. 
			 * &NOT TESTED
			 * @param twist // The velocity that Bravo 7 receives must be in m/s and rad/s
			 * @return void
			 */
			void moveCmdTCPLocalTwist_bravoInstruction(const Eigen::Vector<T_data, Eigen::Dynamic> &twist);

			/**
			 * @brief Send twist in ee frame using jacobian
			 * 
			 * &NOT TESTED
			 * @param twist // The velocity that Bravo 7 receives must be in m/s and rad/s
			 * @return void
			 */
			void setCmdTCPLocalTwist(const Eigen::Vector<T_data, Eigen::Dynamic> &twist);

			void move_ee_twist(const Eigen::Vector<T_data, Eigen::Dynamic> &twist, const Eigen::MatrixXd &jacobian, T_data sampling_time);

			/**
			 * @brief integrator for differential kinematics
			 * Todo: NOT TESTED
			 * @param[in] funcJacobian jacobian function type (local, fixed....) (NOT IMPLEMENTED)
			 * @param[in] sampling_time integration time
			 * @return void
			 */		
			void motion_integration(T_data sampling_time);

			void set_joint_whole_integration(const Eigen::Vector<T_data, Eigen::Dynamic> &q);

			Eigen::Vector<T_data, Eigen::Dynamic> get_joint_whole_integration();

			Eigen::Vector<T_data, Eigen::Dynamic> get_bravo_joint_states();

			size_t get_number_joints();

			Eigen::Vector<T_data, Eigen::Dynamic> get_bravo_joint_velocities();

			Eigen::Vector<T_data, Eigen::Dynamic> get_bravo_joint_currents();

			Eigen::Vector<T_data, Eigen::Dynamic> get_bravo_joint_torques();

			T_data weight_joint_limit_smooth(T_data q, T_data qmin, T_data qinf, T_data qsup, T_data qmax);

			/**
			 * @brief Based on joint states, and joint limits of the arm, the function returns the joint penalization
			 * Todo: NOT TESTED
			 * @return joint_penalization
			 */	
			Eigen::Vector<T_data, Eigen::Dynamic> get_bravo_joint_penalization();

			Eigen::Vector<T_data, Eigen::Dynamic> get_bravo_joint_penalization(std::vector<T_data> joint_states);

			/**
			 * @brief integrator for the simulator differential kinematics
			 * Todo: NOT TESTED
			 * @param[in] twist Task space twist in local frame
			 * @param[in] sampling_time integration time
			 */		
			void sim_motion_integration(T_data sampling_time, Eigen::Vector<T_data, Eigen::Dynamic> twist);

			Eigen::Vector<T_data, Eigen::Dynamic> get_sim_joint_whole_integration();
			
			/**
			 * @brief  Visualization and simulation
			 * @param q_arm arm joint position to plot
			 * &TODO JOINT NAMES MUST MATCH THE URDF 
			 */
			void publish_RVIZ_sim_bravo_joint_states(Eigen::Vector<T_data, Eigen::Dynamic> q_arm, T_data gripper);

			void publish_bravo_joint_states();

			void publish_bravo_joint_currents();

			void publish_bravo_joint_torques();
			
			void publish_wrench_estimation_2(const geometry_msgs::msg::Wrench &wrench_estimation);

			void publish_wrench_estimation(const Eigen::Vector<T_data, Eigen::Dynamic> force_estimation);

			void publish_force_estimation(const Eigen::Vector3d force_estimation);

    	    T_data signedAngleDistance(T_data goal, T_data current);

			Eigen::Vector<T_data, Eigen::Dynamic> signedAngleDistance(const Eigen::Vector<T_data, Eigen::Dynamic>& goal, const Eigen::Vector<T_data, Eigen::Dynamic>& current);
            
			//& SAFETY AND DEBUGGING
			bool isConnected();

			bool arm_is_healthy(const T_data max_time_without_fdb);

			void exceeding_joint_current_limit(Eigen::Vector<T_data, Eigen::Dynamic> q_cmd, Eigen::Vector<T_data, Eigen::Dynamic> MAX_CURRENT);
	};

#endif
