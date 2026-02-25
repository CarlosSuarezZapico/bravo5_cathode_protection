/**
 *    @file  bravo_handler_v2.h
 *    @brief  Derived class handler for bravo7 arm from manipulator base class
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Project UNITE
 *    Created      17-Nov-2023
 *    Modification 24-Feb-2025
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
#include "bravo_cathode_protection/bravo_cpp/bravo_io_v2/bravo_udp_v2.h"
#include "bravo_cathode_protection/bravo_cpp/utils/utils.h"
#include "bravo_cathode_protection/bravo_cpp/utils/bravo_dashboard.h"

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
				//& TERMINAL DASHBOARD FOR BRAVO UDP LOGS
				bravo_utils::TerminalDashboard terminal_dashboard_;
				std::atomic<bool> dashboard_enabled_{false};

				//& LOW-LEVEL INTERFACE WITH BRAVO
				std::unique_ptr<bravo_control::bravo_udp<T_data>> bravo_io;
			size_t number_joints; 

			//& THREAD FOR BRAVO IO INPUT/OUTPUT
			std::thread bravo_io_thread;
			std::atomic<bool> bravo_io_run_thread;

			//& JOINT LIMITS
			Eigen::Vector<T_data, Eigen::Dynamic> upperJointLimits, upperJointInfLimits, lowerJointLimits, lowerJointInfLimits, maskJointLimits;

			//& SAFETY VARIABLES - MAXIMUM JOINT VELOCITY & JOINT CURRENT
			Eigen::Vector<T_data, Eigen::Dynamic> max_q_vel;
			Eigen::Vector<T_data, Eigen::Dynamic> max_q_current;

			//& CURRENT MOTOR CONSTANT (Nm/A)
			Eigen::Vector<T_data, Eigen::Dynamic> motor_constants;

			//& ROS PUBLISHERS SUBSCRIBBERS
			//$PUBLISHERS
			rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr       pubWrenchEstimation;
			rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr            pubFdbJointStates;
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
			 * @param ip ip address of the arm
			 * @param port port for the communication
			 * @param ROS_enable if true, it initializes the ROS publishers for the feedback from the arm
			 */
			bravo_handler(const std::string urdf_filename, const std::string &toolLink, const std::string& ip, int port = 6789, bool ROS_enable = true);

			~bravo_handler();

			/**
			 * @brief Main low-layer interface for bravo IO. This thread is in charge on establish 
			 * a continous bilateral communication with the arm
			 */ 
			void bravo_io_thread_function();

			void set_control_mode(set_bravo_control_mode mode);

			void set_bravo_frequency_packet_exchange(T_data freq);

			T_data compute_manipulability();

			T_data compute_manipulability_position();

			Eigen::Vector<T_data, Eigen::Dynamic> std_2_eigen(std::vector<T_data> std_vector);

			Eigen::Vector<T_data, Eigen::Dynamic> torqueNm_2_currentmA(Eigen::Vector<T_data, Eigen::Dynamic> torqueNm);

			Eigen::Vector<T_data, Eigen::Dynamic> currentmA_2_torqueNm(Eigen::Vector<T_data, Eigen::Dynamic> currentmA);
			
			bool is_in_desired_configuration(T_data tolerance, Eigen::Vector<T_data, Eigen::Dynamic> goal_joint_position, Eigen::Vector<T_data, Eigen::Dynamic> current_joint_position);
            
			//& FUNCTIONS FOR CONTROL AND STATE FEEDBACK
			/**
			 * @brief Send joint current (NOT THE GRIPPER)
			 * @param cmdJointCurrent joint current cmd
			 * @return void
			 */
			void cmdJointCurrent(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointCurrent);

			void cmdJointCurrent_SAT(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointCurrent, const T_data MAX);

			/**
			 * @brief Send joint velocity (NOT THE GRIPPER)
			 * @param cmdJointVel joint vel
			 * @return void
			 */
			void cmdJointVelocity(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointVel);

			void cmdJointVelocity_SAT(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointVel, const T_data MAX);

			Eigen::Vector<T_data, Eigen::Dynamic> get_bravo_joint_states();

			Eigen::Vector<T_data, Eigen::Dynamic> get_bravo_joint_velocities();

			Eigen::Vector<T_data, Eigen::Dynamic> get_bravo_joint_currents();

			Eigen::Vector<T_data, Eigen::Dynamic> get_bravo_joint_torques();

			size_t get_number_joints();

			T_data weight_joint_limit_smooth(T_data q, T_data qmin, T_data qinf, T_data qsup, T_data qmax);

			/**
			 * @brief Based on joint states, and joint limits of the arm, the function returns the joint penalization
			 * Todo: NOT TESTED
			 * @return joint_penalization
			 */	
			Eigen::Vector<T_data, Eigen::Dynamic> get_bravo_joint_penalization();

			Eigen::Vector<T_data, Eigen::Dynamic> get_bravo_joint_penalization(std::vector<T_data> joint_states);
			
			/**
			 * @brief  Visualization and simulation
			 * @param q_arm arm joint position to plot
			 * &TODO JOINT NAMES MUST MATCH THE URDF 
			 */
			void publish_RVIZ_sim_bravo_joint_states(Eigen::Vector<T_data, Eigen::Dynamic> q_arm, T_data gripper);

			void publish_bravo_joint_states();

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
