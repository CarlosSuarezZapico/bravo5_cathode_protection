/**
 *    @file  bravo_handler_v2.cpp
 *    @brief  Bravo Arm Handler
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Project UNITE
 *    Created      17-Nov-2023
 *    Modification 25-Nov-2025
 *    Revision  ---
 *    Compiler  gcc/g++
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#include "bravo_compliance/bravo_cpp/bravo_handler/bravo_handler_v2.h"

template <bravo_control::Floating32or64 T_data>
    bravo_handler<T_data>::bravo_handler(const std::string urdf_filename, const std::string &toolLink, bool real, bool ROS_enable) : rclcpp::Node("bravo7_io_node"),
                    kinodynamics(urdf_filename, toolLink), bravo_io(nullptr), bravo_io_run_thread(real){

            
            // Load the URDF model using urdf::Model
            ::urdf::Model robot_model;
            if (!robot_model.initFile(urdf_filename)) {
                RCLCPP_FATAL(this->get_logger(), "[bravo_handler]: ❌ Failed to parse URDF file");
                std::cerr << "❌ Failed to parse URDF file: " << urdf_filename << std::endl;
                rclcpp::shutdown();
                std::exit(EXIT_FAILURE);
            }
            // Extract the robot name from the URDF
            std::string robot_name = robot_model.getName();
            if (robot_name == "bravo5"){
                RCLCPP_INFO(this->get_logger(), "[bravo_handler]: ✅ Initializing BRAVO 5 model");
                //bravo_io = bravo_control::bravo_udp<T_data>(bravo_control::bravo_udp<T_data>::ArmModel::BRAVO_5, "192.168.2.51");
                bravo_io = std::make_unique<bravo_control::bravo_udp<T_data>>(bravo_control::ArmModel::bravo5,"10.43.0.146");
                max_q_vel.resize(4); sim_joint_whole_integration.resize(4); motor_constants.resize(4);  
                motor_constants << 0.222, 0.222, 0.215, 0.209;
                max_q_vel << 1.0, 1.0, 1.0, 1.0;
                sim_joint_whole_integration << 1.7, 2.7, 0.66, 0.0;
            }
            else if (robot_name == "bravo7"){
                RCLCPP_INFO(this->get_logger(), "[bravo_handler]: ✅ Initializing BRAVO 7 model");
                //bravo_io = bravo_control::bravo_udp<T_data>(bravo_control::bravo_udp<T_data>::ArmModel::BRAVO_7, "192.168.2.51");
                bravo_io = std::make_unique<bravo_control::bravo_udp<T_data>>(bravo_control::ArmModel::bravo7,"192.168.2.51");
                max_q_vel.resize(6); sim_joint_whole_integration.resize(6);  motor_constants.resize(6);
                max_q_vel << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
                sim_joint_whole_integration << 1.7, 2.7, 0.66, 1.8, 3.0, 0.0;
                motor_constants << 0.222, 0.222, 0.215, 0.215, 0.215, 0.209;
            }
            else {
                RCLCPP_FATAL(this->get_logger(), "[bravo_handler]: ❌ Unknown robot model in URDF: %s", robot_name.c_str());
                RCLCPP_FATAL(this->get_logger(), "[bravo_handler]: ❌ The name of the robot in the URDF must be either 'bravo5' or 'bravo7'");
                rclcpp::shutdown();
                std::exit(EXIT_FAILURE);
            }

            if (ROS_enable){                
                //& FEEDBACK FROM BRAVO MANIPULATOR PUBLISHERS
                pubWrenchEstimation    = this->create_publisher<geometry_msgs::msg::WrenchStamped>  ("/bravo/fdb/wrench_estimation", 10);
                pubWrenchEstimation_2  = this->create_publisher<geometry_msgs::msg::Wrench>         ("/bravo/fdb/wrench_estimation_2", 10);
                pubForceEstimation     = this->create_publisher<geometry_msgs::msg::Vector3Stamped> ("/bravo/fdb/force_estimation", 10);
                pubFdbJointStates      = this->create_publisher<sensor_msgs::msg::JointState>       ("/bravo/fdb/joint_states", 10);
                pubFdbJointCurrents    = this->create_publisher<sensor_msgs::msg::JointState>       ("/bravo/fdb/joint_currents", 10); 
                pubFdbJointTorques     = this->create_publisher<sensor_msgs::msg::JointState>       ("/bravo/fdb/joint_torques", 10); 
                pubJointsStateRviz     = this->create_publisher<sensor_msgs::msg::JointState>       ("/bravo/cmd/joint_states", 10);  
                pubJointsStateFreq     = this->create_publisher<std_msgs::msg::Float32>             ("/bravo/frequency_joint_position", 10);  
                pubCurrentJointsFreq   = this->create_publisher<std_msgs::msg::Float32>             ("/bravo/frequency_joint_current", 10);  
                pubFTFreq              = this->create_publisher<std_msgs::msg::Float32>             ("/bravo/frequency_FT", 10);  
            }
            number_joints = bravo_io->get_number_joints()-1; //without gripper
            std::cout << "OK UNTIL HERE, NUMBER OF JOINTS IS " << number_joints;          
            //& DEFINING BRAVO JOINT LIMITS  
            //* The first, fourth and sixth joints are continuous so they have no limits
            T_data angle_limit_smooth = 10;
            maskJointLimits.resize(6);
            maskJointLimits << 0, 1, 1, 0, 1, 0;
            lowerJointLimits.resize(6);
            lowerJointLimits << -1000, 0.0, 0.0, -1000, 0.0, -1000;
            upperJointLimits.resize(6);
            upperJointLimits <<  1000, 3.14, 3.14, 1000, 3.14, 1000;
            Eigen::Vector<T_data, 6> offset;
            offset = angle_limit_smooth * (3.14 / 180) * Eigen::Vector<T_data, Eigen::Dynamic>::Ones(6);
            lowerJointInfLimits.resize(6);
            lowerJointInfLimits = lowerJointLimits + offset;
            upperJointInfLimits.resize(6);
            upperJointInfLimits = upperJointLimits - offset;
            //!amir limits
            _upper_joint_limit.resize(6);
            _upper_joint_limit << std::numeric_limits<T_data>::infinity(), M_PI, M_PI, 2 * M_PI, M_PI, std::numeric_limits<T_data>::infinity();
            _lower_joint_limit.resize(6);
            _lower_joint_limit << -std::numeric_limits<T_data>::infinity(), 0, 0, 0, 0, -std::numeric_limits<T_data>::infinity();
            _continuous.resize(6);
            _continuous << true, false, false, false, false, true;
            //& INTIALIZE EE TWIST COMMAND
            cmdLocalTwist.data << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            cmdLocalTwist.last_update = std::chrono::high_resolution_clock::now();
            //& SIMULATION OR REAL SETUP
            if (!real){
                //! SIMULATION
                //& INITIALIZE VARIABLES FOR SIMULATION                
                sim_last_integration_time   = std::chrono::high_resolution_clock::now();
                sim_start_integration_time  = std::chrono::high_resolution_clock::now();
                sim_finish_integration_time = std::chrono::high_resolution_clock::now();
            }
            else{
                //! REAL
                //& INTIALIZE TIMER FOR INTEGRATION
                last_integration_time = std::chrono::high_resolution_clock::now();
                start_integration_time = std::chrono::high_resolution_clock::now();
                finish_integration_time = std::chrono::high_resolution_clock::now();
                //&START BRAVO_IO THREAD for input and output
                bravo_io->set_dianosis_debug(true); //& SET TO TRUE FOR DEBUGGING
                bravo_io_thread = std::thread(&bravo_handler::bravo_io_thread_function, this); 
            }
        }

    template <bravo_control::Floating32or64 T_data>
        bravo_handler<T_data>::~bravo_handler(){
            bravo_io_run_thread = false;
            if (bravo_io_thread.joinable()) {
                bravo_io_thread.join();
            }
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::bravo_io_thread_function(){
            while (bravo_io_run_thread) {
                bravo_io->req_and_recv();
            }
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::set_control_mode(set_bravo_control_mode mode){
            switch (mode)
            {
            case set_bravo_control_mode::position_control:
                bravo_io->set_mode_all_devices(control_mode_states::joint_position_mode);
                break;
            case set_bravo_control_mode::velocity_control:
                bravo_io->set_mode_all_devices(control_mode_states::joint_velocity_mode);
                break;	
            case set_bravo_control_mode::current_control:
                bravo_io->set_mode_all_devices(control_mode_states::joint_current_mode);
                break;	
            case set_bravo_control_mode::torque_control:
                bravo_io->set_mode_all_devices(control_mode_states::joint_torque_mode);
                break;		
            default:
                bravo_io->set_mode_all_devices(control_mode_states::joint_position_mode);
                RCLCPP_FATAL(this->get_logger(), "[bravo_handler]: ❌ Control mode not recognized, setting to position control by default");
                break;
            }
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::set_bravo_frequency_packet_exchange(T_data freq){
            bravo_io->set_frequency_requests(freq);
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::debug_jointFdb_last_msg_times(){
            bravo_io->debug_jointFdb_last_msg_times();
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::cmdJointPos(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointPosition){
            std::chrono::duration<T_data> time_elapsed_without_jointstates = std::chrono::high_resolution_clock::now() - bravo_io->get_position_jointFdb_last_msg_time();
            if(cmdJointPosition.size() == number_joints){
                bool  motionSafe = true; 
                T_data MAX_JOINT_DISTANCE = 1.5; // in rads. 5.7 deg
                T_data maximum = 0.0;
                T_data MAX_TIME_WITHOUT_JOINTSTATES = 10.0; // in seconds
                //! SAFETY CHECK: verify that command joint position and joint state is not very far apart
                Eigen::Vector<T_data, Eigen::Dynamic> qdiff(number_joints);
                vector<T_data> joint_state = bravo_io->get_bravo_joint_states();
                for (int i=0; i<number_joints; i++){
                    qdiff(i) = abs(cmdJointPosition(i) - joint_state[i]);
                    if (qdiff(i) > maximum){
                        maximum = qdiff(i);
                        if (maximum > MAX_JOINT_DISTANCE){
                            motionSafe = false;
                            RCLCPP_FATAL(this->get_logger(), "[bravo7_handler]: ❌ CmdJointPos very far from current joint position");
                            break; //break the for loop
                        }
                    }
                }
                if ((time_elapsed_without_jointstates.count() <= MAX_TIME_WITHOUT_JOINTSTATES) && (motionSafe==true)
                                        && (bravo_io->get_position_jointFdb_received() == true)){
                    Eigen::Vector<T_data, Eigen::Dynamic> joint_cmd_position(number_joints);
                    for (int i = 0; i < number_joints; ++i) {
                        joint_cmd_position(i) = cmdJointPosition(i); // VAL_SAT<T_data>(cmdJointPosition(0), lowerJointLimits(0), upperJointLimits(0));
                    }
                    bravo_io->set_joint_cmd_position(joint_cmd_position);
                }
                //! ERROR DEBUGGING AND LOGGING
                else{
                    //RCLCPP_ERROR(this->get_logger(), "[bravo7_handler]: Error at sending joint motion command at bravo_handler::cmdJointPos()");
                    bravo_control::printLogEveryXSeconds("ERROR", "[bravo7_handler]:❗Error at sending joint motion command at bravo_handler::cmdJointPos()", 2.0);
                    if (time_elapsed_without_jointstates.count() > MAX_TIME_WITHOUT_JOINTSTATES){
                        if (!bravo_io->get_position_jointFdb_received()){
                            //RCLCPP_ERROR(this->get_logger(), "[bravo7_handler]: Bravo has not received any joint feedback");
                            bravo_control::printLogEveryXSeconds("ERROR", "[bravo7_handler]:❗Bravo has not received any joint feedback", 2.0);
                        }
                        else{                              
                            if (time_elapsed_without_jointstates.count()>100000){  //! why such a big number
                                //RCLCPP_ERROR(this->get_logger(), "[bravo7_handler]: Bravo has not received any joint feedback");
                                bravo_control::printLogEveryXSeconds("ERROR", "[bravo7_handler]:❗Bravo has not received any joint feedback", 2.0);
                            }
                            else{
                                std::cerr << "[bravo7_handler]:❗Time elapsed without joint feedback: " << time_elapsed_without_jointstates.count() << " seconds" << std::endl;
                            }
                        }    
                    }
                    if (!motionSafe){
                        std::cerr << "[bravo7_handler]:❗Difference between Cmd and Fbd" 
                            << " [" << abs(cmdJointPosition(0) - joint_state[0]) << ", "
                            << abs(cmdJointPosition(1) - joint_state[1]) << ", "
                            << abs(cmdJointPosition(2) - joint_state[2]) << ", "
                            << abs(cmdJointPosition(3) - joint_state[3]) << ", "
                            << abs(cmdJointPosition(4) - joint_state[4]) << ", "
                            << abs(cmdJointPosition(5)- joint_state[5]) 
                            << "] where the maximum allowed difference is " 
                            << MAX_JOINT_DISTANCE << std::endl;
                        std::cerr << "[bravo7_handler]: Cmd [" 
                            << cmdJointPosition(0) << ", " 
                            << cmdJointPosition(1) << ", " 
                            << cmdJointPosition(2) << ", " 
                            << cmdJointPosition(3) << ", " 
                            << cmdJointPosition(4) << ", " 
                            << cmdJointPosition(5) << "]" << std::endl;
                        std::cerr << "[bravo7_handler]: Fdb [" 
                            << joint_state[0] << ", " 
                            << joint_state[1] << ", " 
                            << joint_state[2] << ", " 
                            << joint_state[3] << ", " 
                            << joint_state[4] << ", " 
                            << joint_state[5] << "]" << std::endl;
                        std::cerr << "[bravo7_handler]: Time elapsed without joint feedback: " 
                            << time_elapsed_without_jointstates.count() << " seconds" << std::endl;
                    }
                }
            }
            else{
                std::cerr << "[bravo7_handler]: ❌ Number of components received for bravo_handler::cmdJointPos does not match the number of joints" << std::endl;
                rclcpp::shutdown(); // Clean up ROS 2 resources
                std::exit(EXIT_FAILURE); // Exit the program with an error code
            }
		}
    
    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::cmdJointTorque(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointTorqueIn){
            if(cmdJointTorqueIn.size() == number_joints){
                bravo_io->set_joint_cmd_torque(cmdJointTorqueIn);
            }
            else{
                RCLCPP_FATAL(this->get_logger(), "[bravo_handler]: ❌ Number of components received for bravo_handler::cmdJointTorque does not match the number of joints");
                rclcpp::shutdown(); // Clean up ROS 2 resources
                std::exit(EXIT_FAILURE); // Exit the program with an error code
            }
		}

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::cmdJointCurrent(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointCurrent){
            if(cmdJointCurrent.size() == number_joints){
                bravo_io->set_joint_cmd_current(cmdJointCurrent);
            }
            else{
                RCLCPP_FATAL(this->get_logger(), "[bravo_handler]: ❌ Number of components received for bravo_handler::cmdJointCurrent does not match the number of joints");
                rclcpp::shutdown(); // Clean up ROS 2 resources
                std::exit(EXIT_FAILURE); // Exit the program with an error code
            }
		}
    
        template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::cmdJointCurrent_SAT(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointCurrent, const T_data MAX){
            if(cmdJointCurrent.size() == number_joints){
                Eigen::Vector<T_data, Eigen::Dynamic> joint_cmd_current(number_joints);
                for (int i = 0; i < number_joints; i++){
                    joint_cmd_current(i) = VAL_SAT<T_data>(cmdJointCurrent(i), MAX, -MAX);
                }
                bravo_io->set_joint_cmd_current(joint_cmd_current);
            }
            else{
                RCLCPP_FATAL(this->get_logger(), "[bravo_handler]: ❌ Number of components received for bravo_handler::cmdJointCurrent does not match the number of joints");
                rclcpp::shutdown(); // Clean up ROS 2 resources
                std::exit(EXIT_FAILURE); // Exit the program with an error code
            }
		}

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::cmdJointVel(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointVel){
            if(cmdJointVel.size() == number_joints){
                Eigen::Vector<T_data, Eigen::Dynamic> joint_cmd_velocity(number_joints);
                for (int i = 0; i < number_joints; i++){
                    joint_cmd_velocity(i) = VAL_SAT<T_data>(cmdJointVel(i), max_q_vel(i), -max_q_vel(i));
                }
                bravo_io->set_joint_cmd_velocity(joint_cmd_velocity);
            }
            else{
                RCLCPP_FATAL(this->get_logger(), "[bravo_handler]: ❌ Number of components received for bravo_handler::cmdJointVel does not match the number of joints");
                rclcpp::shutdown(); // Clean up ROS 2 resources
                std::exit(EXIT_FAILURE); // Exit the program with an error code
            }
		}

        /**
         * @brief Evaluates if the arm is in the desired configuration
         * @param tolerance minimum error to consider the arm in the desired configuration
         * @param goal_joint_position desired joint configuration
         * @param current_joint_position current joint configuration
         * @return bool true if the arm is in the desired configuration
         */   
    template <bravo_control::Floating32or64 T_data>
        bool bravo_handler<T_data>::is_in_desired_configuration(T_data tolerance, Eigen::Vector<T_data, Eigen::Dynamic> goal_joint_position, Eigen::Vector<T_data, Eigen::Dynamic> current_joint_position){
            Eigen::Vector<T_data, Eigen::Dynamic> error; 
            error.resize(number_joints);
            for (int i=0; i< number_joints; i++){   
                error[i] = abs(fmod(goal_joint_position[i] - current_joint_position[i]+ 3 * M_PI, 2 * M_PI) - M_PI); //THE JOINTS HAVE A RANGE FROM 0.0 TO 6.28, for that reason we use this formula 
                if (error[i] > tolerance){
                    return false;
                }
            }
            return true;
        }

        /**
         * @brief Function to drive the arm to a goal joint position
         * @param goal_joint_position desired joint configuration
         * @param current_joint_position current joint configuration
         * @param joint_vel joint velocity used in the interpolation
         * @param sampling_time integration time
         * @return joint_cmd joint command to be sent to the arm
         */ 
    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::going2joint_interpolation(Eigen::Vector<T_data, Eigen::Dynamic> goal_joint_position, Eigen::Vector<T_data, Eigen::Dynamic> current_joint_position, T_data joint_vel, T_data sampling_time){            
            Eigen::Vector<T_data, Eigen::Dynamic> delta_joint = goal_joint_position - current_joint_position;     
            for (int i=0; i<number_joints; i++){   
                delta_joint[i] = fmod(goal_joint_position[i] - current_joint_position[i] + 3 * M_PI, 2 * M_PI) - M_PI; //THE JOINTS HAVE A RANGE FROM 0.0 TO 6.28, for that reason we use this formula 
            }       
            // Normalize the delta to get direction of movement and scale by velocity
            Eigen::Vector<T_data, Eigen::Dynamic> delta_joint_normalized = delta_joint.normalized();            
            Eigen::Vector<T_data, Eigen::Dynamic> joint_cmd = current_joint_position + delta_joint_normalized * joint_vel * sampling_time;
            // Make sure we don't overshoot the goal
            for (int i = 0; i < number_joints; i++) {
                if (std::abs(delta_joint[i]) < std::abs(0.02)) {
                    joint_cmd[i] = goal_joint_position[i];  // Clamp to goal if overshooting
                }
            }
            return joint_cmd;
        }

    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::std_2_eigen(std::vector<T_data> std_vector){
            Eigen::Vector<T_data, Eigen::Dynamic> eigen_vector;
            eigen_vector.resize(number_joints);
            for (int i=0; i<number_joints; i++){
                eigen_vector(i) = std_vector[i];
            }
            return eigen_vector;
        }
    
    template <bravo_control::Floating32or64 T_data>
    	Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::torqueNm_2_currentmA(Eigen::Vector<T_data, Eigen::Dynamic> torqueNm){
            Eigen::Vector<T_data, Eigen::Dynamic> mA_output;        
            mA_output = (torqueNm.array()*1000.0) / (motor_constants.array()*120.0);
            return mA_output;
        }
  
    template <bravo_control::Floating32or64 T_data>
	    Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::currentmA_2_torqueNm(Eigen::Vector<T_data, Eigen::Dynamic> currentmA){
            Eigen::Vector<T_data, Eigen::Dynamic> Nm_output;
            Nm_output = currentmA.array() * (motor_constants.array()*120.0)/ 1000.0;
            return Nm_output;
        }

    template <bravo_control::Floating32or64 T_data>
        bool bravo_handler<T_data>::go_to_JointPos(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointPosition, T_data joint_vel, T_data tolerance){
            if(bravo_io->get_control_mode() != bravo_control::control_mode_states::joint_position_mode){
                RCLCPP_WARN(this->get_logger(), "[bravo_handler]: Setting Bravo7 control mode to position");
                bravo_io->set_control_mode(bravo_control::control_mode_states::joint_position_mode);
                bravo_io->set_mode_all_devices(bravo_control::control_mode_states::joint_position_mode);
            }
            Eigen::Vector<T_data, Eigen::Dynamic> cmdIntermediateJointPosition;
            std::chrono::high_resolution_clock::time_point last_interpolation_time;
            T_data sampling_time = 0.05;
            while(not is_in_desired_configuration(tolerance, cmdJointPosition, std_2_eigen(bravo_io->get_bravo_joint_states()))){                
                std::chrono::duration<T_data> elapsed= std::chrono::high_resolution_clock::now()-last_interpolation_time;
                if (elapsed.count() > sampling_time){
                    cmdIntermediateJointPosition = going2joint_interpolation(cmdJointPosition, std_2_eigen(bravo_io->get_bravo_joint_states()), joint_vel, sampling_time);
                    last_interpolation_time = std::chrono::high_resolution_clock::now();
                    cmdJointPos(cmdIntermediateJointPosition);
                    //RCLCPP_WARN(this->get_logger(), "[bravo_handler]: Joint CMD  %f, %f, %f, %f, %f, %f ", cmdJointPosition[0], cmdJointPosition[1], cmdJointPosition[2], cmdJointPosition[3], cmdJointPosition[4], cmdJointPosition[5]);
                    //RCLCPP_WARN(this->get_logger(), "[bravo_handler]: Intermediate CMD  %f, %f, %f, %f, %f, %f ", cmdIntermediateJointPosition[0], cmdIntermediateJointPosition[1], cmdIntermediateJointPosition[2], cmdIntermediateJointPosition[3], cmdIntermediateJointPosition[4], cmdIntermediateJointPosition[5]);
                }                
            }
            return true;
        }
    
    template <bravo_control::Floating32or64 T_data>
        std::tuple<bool, bool, Eigen::Vector<T_data, Eigen::Dynamic>> bravo_handler<T_data>::go_to_joint_position(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdGoalJointPosition, T_data joint_vel, T_data tolerance, std::chrono::high_resolution_clock::time_point& last_call){
            Eigen::Vector<T_data, Eigen::Dynamic> cmdIntermediateJointPosition;
            cmdIntermediateJointPosition.resize(number_joints);
            T_data sampling_time = 0.05;
            bool arrived = is_in_desired_configuration(tolerance, cmdGoalJointPosition, std_2_eigen(bravo_io->get_bravo_joint_states())); 
            bool ready = false;            
            std::chrono::duration<T_data> elapsed= std::chrono::high_resolution_clock::now()- last_call;  
            if (elapsed.count() > sampling_time){  
                //* INTERPOLATION
                Eigen::Vector<T_data, Eigen::Dynamic> current_pos = std_2_eigen(bravo_io->get_bravo_joint_states());
                Eigen::Vector<T_data, Eigen::Dynamic> delta_joint;
                delta_joint.resize(number_joints);

                for (int i = 0; i < number_joints; i++) {
                    // Wrap difference into [-pi, pi]
                    delta_joint[i] = fmod(cmdGoalJointPosition[i] - current_pos[i] + 3 * M_PI, 2 * M_PI) - M_PI;

                    T_data max_step = joint_vel * sampling_time;

                    if (std::abs(delta_joint[i]) <= max_step) {
                        // If within one step, go directly to goal
                        cmdIntermediateJointPosition[i] = cmdGoalJointPosition[i];
                    } else {
                        // Step toward goal without overshooting
                        cmdIntermediateJointPosition[i] = current_pos[i] + max_step * ((delta_joint[i] > 0) ? 1.0 : -1.0);
                    }
                }
                ready = true;
                last_call = std::chrono::high_resolution_clock::now();
            }
            return std::make_tuple(arrived, ready, cmdIntermediateJointPosition);              
        }

    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::integrate_configuration(const Eigen::Vector<T_data, Eigen::Dynamic> current_q, const Eigen::Vector<T_data, Eigen::Dynamic> velocity)
            {
                if (current_q.size() != 6)
                {
                    std::cerr << "[integrate_configuration]: ❌ Invalid vector 'current_q' size." << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                if (velocity.size() != 6)
                {
                    std::cerr << "[integrate_configuration]: ❌ Invalid vector 'velocity' size." << std::endl;
                    std::exit(EXIT_FAILURE);
                }
                Eigen::Vector<T_data, Eigen::Dynamic> new_q = current_q + velocity;                
                for (int i = 0; i < 6; i++)
                {
                    if (_continuous(i))
                    {
                        while (new_q(i) > 2 * M_PI)
                        {
                            new_q(i) -= 2 * M_PI;
                        }
                        while (new_q(i) < 0)
                        {
                            new_q(i) += 2 * M_PI;
                        }
                    }
                    else
                    {
                        if (new_q(i) > _upper_joint_limit(i))
                        {
                            new_q(i) = _upper_joint_limit(i);
                        }
                        else if (new_q(i) < _lower_joint_limit(i))
                        {
                            new_q(i) = _lower_joint_limit(i);
                        }
                    }
                }
                return new_q;
            }

    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_joint_error(const Eigen::Vector<T_data, Eigen::Dynamic> current_q, const Eigen::Vector<T_data, Eigen::Dynamic> desired_q){
            Eigen::Vector<T_data, Eigen::Dynamic> error;
            error.resize(number_joints);
            if (current_q.size() != number_joints)
            {
                std::cerr << "[get_joint_error]: ❌ Invalid vector 'current_q' size." << std::endl;
                std::exit(EXIT_FAILURE);
            }
            if (desired_q.size() != number_joints)
            {
                std::cerr << "[get_joint_error]: ❌ Invalid vector 'desired_q' size." << std::endl;
                std::exit(EXIT_FAILURE);
            }        
            if ((desired_q.array() > _upper_joint_limit.array()).all() ||
                (desired_q.array() < _lower_joint_limit.array()).all())
            {
                std::cerr << "[get_joint_error]: 'desired_q' is out of joint limits." << std::endl;
                error = Eigen::Vector<T_data, Eigen::Dynamic>::Zero(number_joints);
                return error;
            }
;
            for (int i = 0; i < number_joints; i++)
            {
                if (_continuous(i))
                {
                    error(i) = desired_q(i) - current_q(i);
                    while(error(i) > M_PI)
                    {
                        error(i) -= 2 * M_PI;
                    }
                    while(error(i) < -M_PI)
                    {
                        error(i) += 2 * M_PI;
                    }
                }
                else
                {
                    error(i) = desired_q(i) - current_q(i);
                }
            }
            return error;
        }
    
    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::scale_joint_velocity(const Eigen::Vector<T_data, Eigen::Dynamic> dq)
        {
            T_data scaling_factor = 1;
            for (int i = 0; i < number_joints; i++)
            {
                scaling_factor = std::min({scaling_factor, _joint_vel_limits(i) / std::abs(dq(i))});
            }
            return scaling_factor * dq;
        }
    
    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::go_to_configuration(const Eigen::Vector<T_data, Eigen::Dynamic> desired_q, const Eigen::Vector<T_data, Eigen::Dynamic> v, const T_data dt)
        {
            Eigen::Vector<T_data, Eigen::Dynamic> current_q = get_bravo_joint_states();
            for (int i = 0; i < number_joints; i++)
            {
                if (std::abs(interpolated_q_(i) - current_q(i)) > 1.00)
                {
                    interpolated_q_(i) = current_q(i);
                }
            }            
            Eigen::Vector<T_data, Eigen::Dynamic> error = get_joint_error(interpolated_q_, desired_q);
            Eigen::Vector<T_data, Eigen::Dynamic> error_sign = error.array().sign();
            Eigen::Vector<T_data, Eigen::Dynamic> scaled_v = scale_joint_velocity(v);
            Eigen::Vector<T_data, Eigen::Dynamic> velocity;
            velocity.resize(number_joints);
            for (int i = 0; i < number_joints; i++)
            {
                if (std::abs(error(i)) > std::abs(scaled_v(i) * dt))
                {
                    velocity(i) = error_sign(i) * std::abs(scaled_v(i) * dt);
                }
                else
                {
                    velocity(i) = error(i);
                }
            }            
            interpolated_q_ = integrate_configuration(interpolated_q_, velocity);
            for (int i = 0; i < number_joints; i++)
            {
                if (std::isnan(desired_q(i)))
                {
                    interpolated_q_(i) = std::numeric_limits<T_data>::quiet_NaN();
                }
            }
            cmdJointPos(interpolated_q_);
            Eigen::Vector<T_data, Eigen::Dynamic> fb_error = get_joint_error(current_q, desired_q);
            for (int i = 0; i < number_joints; i++)
            {
                if (std::isnan(fb_error(i)))
                {
                    fb_error(i) = 0.0;
                }
            }
            return fb_error;
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::go_to_JointPos(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointPosition, T_data joint_vel){
            if(bravo_io->get_control_mode() != bravo_control::control_mode_states::joint_position_mode){
                RCLCPP_WARN(this->get_logger(), "[bravo_handler]:❗Setting Bravo7 control mode to position");
                bravo_io->set_control_mode(bravo_control::control_mode_states::joint_position_mode);
                bravo_io->set_mode_all_devices(bravo_control::control_mode_states::joint_position_mode);
            }
            Eigen::Vector<T_data, Eigen::Dynamic> cmdIntermediateJointPosition;
            std::chrono::high_resolution_clock::time_point last_interpolation_time;
            T_data sampling_time = 0.05;
            while(not is_in_desired_configuration(0.1, cmdJointPosition, std_2_eigen(bravo_io->get_bravo_joint_states()))){                
                std::chrono::duration<T_data> elapsed= std::chrono::high_resolution_clock::now()-last_interpolation_time;
                if (elapsed.count() > sampling_time){
                    cmdIntermediateJointPosition = going2joint_interpolation(cmdJointPosition, std_2_eigen(bravo_io->get_bravo_joint_states()), joint_vel, sampling_time);
                    last_interpolation_time = std::chrono::high_resolution_clock::now();
                    cmdJointPos(cmdIntermediateJointPosition);
                    //RCLCPP_WARN(this->get_logger(), "[bravo_handler]: Joint CMD  %f, %f, %f, %f, %f, %f ", cmdJointPosition[0], cmdJointPosition[1], cmdJointPosition[2], cmdJointPosition[3], cmdJointPosition[4], cmdJointPosition[5]);
                    //RCLCPP_WARN(this->get_logger(), "[bravo_handler]: Intermediate CMD  %f, %f, %f, %f, %f, %f ", cmdIntermediateJointPosition[0], cmdIntermediateJointPosition[1], cmdIntermediateJointPosition[2], cmdIntermediateJointPosition[3], cmdIntermediateJointPosition[4], cmdIntermediateJointPosition[5]);
                }                
            }
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::set_interpolated_q(const Eigen::Vector<T_data, Eigen::Dynamic> &q){
            interpolated_q_ = q;
        }

    template <bravo_control::Floating32or64 T_data>
        T_data bravo_handler<T_data>::compute_manipulability(){
            Eigen::MatrixXd jacobian = kinodynamics.localJacobian(std_2_eigen(bravo_io->get_bravo_joint_states()));
            T_data cond_number = kinodynamics.cond_arm(jacobian);
            return cond_number;
        }
    
    template <bravo_control::Floating32or64 T_data>
        T_data bravo_handler<T_data>::compute_manipulability_position(){
            Eigen::MatrixXd Jacobian = kinodynamics.fixedJacobian(std_2_eigen(bravo_io->get_bravo_joint_states()));
            Eigen::MatrixXd jacobian_pos = Jacobian.topRows(3);
            T_data cond_number = kinodynamics.cond_arm(jacobian_pos);
            return cond_number;
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::moveCmdTCPLocalTwist(const Eigen::Vector<T_data, Eigen::Dynamic> &twist, T_data sampling_time){ 
            std::chrono::duration<T_data> elapsed_joint_fdb = std::chrono::high_resolution_clock::now() - bravo_io->get_position_jointFdb_last_msg_time();
            if (elapsed_joint_fdb.count() < 0.2){
                T_data MAX_JOINT_DISTANCE = 1.0; // in rads. 10 deg
                T_data maximum = 0.0;
                //! SAFETY CHECK: verify that command joint position and joint state is not very far apart
                Eigen::Vector<T_data, Eigen::Dynamic> qdiff(number_joints);
                std::vector<T_data> joint_state = bravo_io->get_bravo_joint_states();
                for (int i=0; i<number_joints; i++){
                    qdiff(i) = abs(joint_whole_integration(i) - joint_state[i]);
                    if (qdiff(i) > maximum){
                        maximum = qdiff(i);
                    }
                    if (maximum > MAX_JOINT_DISTANCE){
                        for (int i=0; i<number_joints; i++){
                            joint_whole_integration(i) = joint_state[i];
                        }
                        start_integration_time = std::chrono::high_resolution_clock::now();
                    }
                }
                std::chrono::duration<T_data> elapsed_integration= std::chrono::high_resolution_clock::now() - last_integration_time;
                if (elapsed_integration.count() > sampling_time) {
                    Eigen::MatrixXd jacobian = kinodynamics.localJacobian(joint_whole_integration);
                    T_data cond_number = kinodynamics.cond_arm(jacobian);
                    joint_vel_integration = jacobian.inverse()*twist;
                    finish_integration_time = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<T_data> elapsed = finish_integration_time - start_integration_time;
                    joint_whole_integration = joint_whole_integration + joint_vel_integration*elapsed.count();
                    start_integration_time = std::chrono::high_resolution_clock::now();
                    last_integration_time = std::chrono::high_resolution_clock::now();
                    //! SAFETY CHECK: verify manipulability is below a safety threshold
                    //RCLCPP_INFO(this->get_logger(), "[bravo_handler]: The condition number is %.2f", cond_number);
                    manipulability = abs(cond_number);
                    if (manipulability < MAX_MANIPULABILITY){
                        if (!print_warn_cond){
                            RCLCPP_INFO(this->get_logger(), "[bravo_handler]: The condition number is within the safety margin");
                        }
                        cmdJointPos(joint_whole_integration);
                        print_warn_cond = true;
                    }
                    else{
                        if (print_warn_cond){
                            RCLCPP_WARN(this->get_logger(), "[bravo_handler]:❗The condition number is too high, for safety reasons the command will not be executed");                            
                        }

                        print_warn_cond = false;
                    }					
                }   
            }
		}

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::move_ee_twist(const Eigen::Vector<T_data, Eigen::Dynamic> &twist, const Eigen::MatrixXd &jacobian, T_data sampling_time){ 
            std::chrono::duration<T_data> elapsed_joint_fdb = std::chrono::high_resolution_clock::now() - bravo_io->get_position_jointFdb_last_msg_time();
            if (elapsed_joint_fdb.count() < 0.2){
                T_data MAX_JOINT_DISTANCE = 0.3; // in rads. 10 deg
                T_data maximum = 0.0;
                //! SAFETY CHECK: verify that command joint position and joint state is not very far apart
                Eigen::Vector<T_data, Eigen::Dynamic> qdiff(number_joints);
                std::vector<T_data> joint_state = bravo_io->get_bravo_joint_states();
                for (int i=0; i<number_joints; i++){
                    qdiff(i) = abs(joint_whole_integration(i) - joint_state[i]);
                    if (qdiff(i) > maximum){
                        maximum = qdiff(i);
                    }
                    if (maximum > MAX_JOINT_DISTANCE){
                        for (int i=0; i<number_joints; i++){
                            joint_whole_integration(i) = joint_state[i];
                        }
                        start_integration_time = std::chrono::high_resolution_clock::now();
                    }
                }
                std::chrono::duration<T_data> elapsed_integration= std::chrono::high_resolution_clock::now() - last_integration_time;
                if (elapsed_integration.count() > sampling_time) {
                    T_data cond_number = kinodynamics.cond_arm(jacobian);
                    joint_vel_integration = jacobian.inverse()*twist;
                    finish_integration_time = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<T_data> elapsed = finish_integration_time - start_integration_time;
                    joint_whole_integration = joint_whole_integration + joint_vel_integration*elapsed.count();
                    start_integration_time = std::chrono::high_resolution_clock::now();
                    last_integration_time = std::chrono::high_resolution_clock::now();
                    //! SAFETY CHECK: verify manipulability is below a safety threshold

                    manipulability = abs(cond_number);
                    if (manipulability < MAX_MANIPULABILITY){
                        if (!print_warn_cond){
                            RCLCPP_INFO(this->get_logger(), "[bravo_handler]: The condition number is within the safety margin");
                        }
                        cmdJointPos(joint_whole_integration);
                        print_warn_cond = true;
                    }
                    else{
                        if (print_warn_cond){
                            RCLCPP_WARN(this->get_logger(), "[bravo_handler]: The condition number is too high, for safety reasons the command will not be executed");                            
                        }
                        print_warn_cond = false;
                        std::cout<< "condition number too high: " << manipulability << std::endl;
                    }					
                }   
            }
		}

    template <bravo_control::Floating32or64 T_data>
        std::tuple<bool, Eigen::Vector<T_data, Eigen::Dynamic>> bravo_handler<T_data>::twist_diff_kin(const Eigen::Vector<T_data, Eigen::Dynamic> &twist, T_data sampling_time){ 
            std::chrono::duration<T_data> elapsed_joint_fdb = std::chrono::high_resolution_clock::now() - bravo_io->get_position_jointFdb_last_msg_time();
            if (elapsed_joint_fdb.count() < 0.8){
                T_data MAX_JOINT_DISTANCE = 1.0; // in rads. 10 deg
                T_data maximum = 0.0;
                //! SAFETY CHECK: verify that command joint position and joint state is not very far apart
                Eigen::Vector<T_data, Eigen::Dynamic> qdiff(number_joints);
                std::vector<T_data> joint_state = bravo_io->get_bravo_joint_states();
                for (int i=0; i<number_joints; i++){
                    qdiff(i) = abs(joint_whole_integration(i) - joint_state[i]);
                    if (qdiff(i) > maximum){
                        maximum = qdiff(i);
                    }
                    if (maximum > MAX_JOINT_DISTANCE){
                        for (int i=0; i<number_joints; i++){
                            joint_whole_integration(i) = joint_state[i];
                        }
                        start_integration_time = std::chrono::high_resolution_clock::now();
                    }
                }
                std::chrono::duration<T_data> elapsed_integration= std::chrono::high_resolution_clock::now() - last_integration_time;
                if (elapsed_integration.count() > sampling_time) {
                    Eigen::MatrixXd jacobian = kinodynamics.localJacobian(joint_whole_integration);
                    T_data cond_number = kinodynamics.cond_arm(jacobian);
                    joint_vel_integration = jacobian.inverse()*twist;
                    finish_integration_time = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<T_data> elapsed = finish_integration_time - start_integration_time;
                    joint_whole_integration = joint_whole_integration + joint_vel_integration*elapsed.count();
                    start_integration_time = std::chrono::high_resolution_clock::now();
                    last_integration_time = std::chrono::high_resolution_clock::now();
                    //! SAFETY CHECK: verify manipulability is below a safety threshold
                    //RCLCPP_INFO(this->get_logger(), "[bravo_handler]: The condition number is %.2f", cond_number);
                    manipulability = abs(cond_number);
                    if (manipulability < MAX_MANIPULABILITY){
                        if (!print_warn_cond){
                            RCLCPP_INFO(this->get_logger(), "[bravo_handler]: The condition number is within the safety margin");
                        }
                        return std::make_tuple(true, joint_whole_integration);
                        print_warn_cond = true;
                    }
                    else{
                        if (print_warn_cond){
                            RCLCPP_WARN(this->get_logger(), "[bravo_handler]: ❗ The condition number is too high, for safety reasons the command will not be executed");                            
                        }
                        return std::make_tuple(false, joint_whole_integration);
                        print_warn_cond = false;
                    }					
                }   
            }
            return std::make_tuple(false, joint_whole_integration); //! tHIS MAY BE A MISTAKE
		}
    
    template <bravo_control::Floating32or64 T_data>
        std::tuple<bool, Eigen::Vector<T_data, Eigen::Dynamic>, Eigen::Vector<T_data, Eigen::Dynamic>> bravo_handler<T_data>::twist_diff_kin_2(const Eigen::Vector<T_data, Eigen::Dynamic> &twist, T_data sampling_time){ 
            std::chrono::duration<T_data> elapsed_joint_fdb = std::chrono::high_resolution_clock::now() - bravo_io->get_position_jointFdb_last_msg_time();
            if (elapsed_joint_fdb.count() < 0.8){
                T_data MAX_JOINT_DISTANCE = 1.0; // in rads. 10 deg
                T_data maximum = 0.0;
                //! SAFETY CHECK: verify that command joint position and joint state is not very far apart
                Eigen::Vector<T_data, Eigen::Dynamic> qdiff(number_joints);
                std::vector<T_data> joint_state = bravo_io->get_bravo_joint_states();
                for (int i=0; i<number_joints; i++){
                    qdiff(i) = abs(joint_whole_integration(i) - joint_state[i]);
                    if (qdiff(i) > maximum){
                        maximum = qdiff(i);
                    }
                    if (maximum > MAX_JOINT_DISTANCE){
                        for (int i=0; i<number_joints; i++){
                            joint_whole_integration(i) = joint_state[i];
                        }
                        start_integration_time = std::chrono::high_resolution_clock::now();
                    }
                }
                std::chrono::duration<T_data> elapsed_integration= std::chrono::high_resolution_clock::now() - last_integration_time;
                if (elapsed_integration.count() > sampling_time) {
                    Eigen::MatrixXd jacobian = kinodynamics.localJacobian(joint_whole_integration);
                    T_data cond_number = kinodynamics.cond_arm(jacobian);
                    joint_vel_integration = jacobian.inverse()*twist;
                    finish_integration_time = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<T_data> elapsed = finish_integration_time - start_integration_time;
                    joint_whole_integration = joint_whole_integration + joint_vel_integration*elapsed.count();
                    start_integration_time = std::chrono::high_resolution_clock::now();
                    last_integration_time = std::chrono::high_resolution_clock::now();
                    //! SAFETY CHECK: verify manipulability is below a safety threshold
                    //RCLCPP_INFO(this->get_logger(), "[bravo_handler]: The condition number is %.2f", cond_number);
                    manipulability = abs(cond_number);
                    if (manipulability < MAX_MANIPULABILITY){
                        if (!print_warn_cond){
                            RCLCPP_INFO(this->get_logger(), "[bravo_handler]: The condition number is within the safety margin");
                        }
                        return std::make_tuple(true, joint_whole_integration, joint_vel_integration);
                        print_warn_cond = true;
                    }
                    else{
                        if (print_warn_cond){
                            RCLCPP_WARN(this->get_logger(), "[bravo_handler]: The condition number is too high, for safety reasons the command will not be executed");                            
                        }
                        return std::make_tuple(false, joint_whole_integration, joint_vel_integration);
                        print_warn_cond = false;
                    }					
                }   
            }
            return std::make_tuple(false, joint_whole_integration, joint_vel_integration); //! tHIS MAY BE A MISTAKE
		}

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::moveCmdTCPLocalTwist_bravoInstruction(const Eigen::Vector<T_data, Eigen::Dynamic> &twist){
            if(bravo_io->get_control_mode() != bravo_control::control_mode_states::local_twist_ee_mode){
                bravo_io->set_control_mode(bravo_control::control_mode_states::local_twist_ee_mode);
            }
            bravo_io->set_local_twist_ee(twist);
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::setCmdTCPLocalTwist(const Eigen::Vector<T_data, Eigen::Dynamic> &twist){
            cmdLocalTwist.data = twist;
            cmdLocalTwist.last_update = chrono::high_resolution_clock::now();
		}

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::set_joint_whole_integration(const Eigen::Vector<T_data, Eigen::Dynamic> &q){
            joint_whole_integration = q;
        }

    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_joint_whole_integration(){
            return joint_whole_integration;
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
    template <bravo_control::Floating32or64 T_data> 
        T_data bravo_handler<T_data>::weight_joint_limit_smooth(T_data q, T_data qmin, T_data qinf, T_data qsup, T_data qmax) {
            T_data ais = 2;
            T_data bis = -3 * (qmax + qsup);
            T_data cis = 6 * qmax * qsup;
            T_data dis = pow(qmax, 3) - 3 * pow(qmax, 2) * qsup;
            T_data aii = 2;
            T_data bii = -3 * (qmin + qinf);
            T_data cii = 6 * qmin * qinf;
            T_data dii = pow(qmin, 3) - 3 * pow(qmin, 2) * qinf;
            T_data w;
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
    
    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_penalization(){
            Eigen::Vector<T_data, Eigen::Dynamic> joint_penalization;
            std::vector<T_data> joint_states = bravo_io->get_bravo_joint_states();
            Eigen::Vector<T_data, Eigen::Dynamic> eigen_joint_states = Eigen::Map<Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), joint_states.size()-1);
            joint_penalization.resize(joint_states.size()-1);   
            for(int i=0; i<(joint_states.size()-1); i++){
                joint_penalization(i) = weight_joint_limit_smooth(eigen_joint_states(i), lowerJointLimits(i), lowerJointInfLimits(i), upperJointInfLimits(i), upperJointLimits(i));
            }
            return joint_penalization;
        }
    
    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_penalization(std::vector<T_data> joint_states){
            Eigen::Vector<T_data, Eigen::Dynamic> joint_penalization;
            Eigen::Vector<T_data, Eigen::Dynamic> eigen_joint_states = Eigen::Map<Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), joint_states.size());
            joint_penalization.resize(joint_states.size());   
            for(int i=0; i<(joint_states.size()); i++){
                joint_penalization(i) = weight_joint_limit_smooth(eigen_joint_states(i), lowerJointLimits(i), lowerJointInfLimits(i), upperJointInfLimits(i), upperJointLimits(i));
            }
            return joint_penalization;
        }

    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_states(){
            std::vector<T_data> joint_states = bravo_io->get_bravo_joint_states();
            // Map the std::vector to an Eigen::Vector<T_data, Eigen::Dynamic>
            //Copy all elements except for the last one which corresponds to the gripper joint state
            Eigen::Vector<T_data, Eigen::Dynamic> eigen_joint_states = Eigen::Map<Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), joint_states.size()-1);
            return eigen_joint_states;
        }
    
    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_velocities(){
            std::vector<T_data> joint_states = bravo_io->get_bravo_joint_velocities();
            // Map the std::vector to an Eigen::Vector<T_data, Eigen::Dynamic>
            //Copy all elements except for the last one which corresponds to the gripper joint state
            Eigen::Vector<T_data, Eigen::Dynamic> eigen_joint_states = Eigen::Map<Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), joint_states.size()-1);
            return eigen_joint_states;
        }
    
    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_currents(){
            std::vector<T_data> joint_states = bravo_io->get_bravo_joint_currents();
            // Map the std::vector to an Eigen::Vector<T_data, Eigen::Dynamic>
            //Copy all elements except for the last one which corresponds to the gripper joint state
            Eigen::Vector<T_data, Eigen::Dynamic> eigen_joint_states = Eigen::Map<Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), joint_states.size()-1);
            return eigen_joint_states;
        }
    
    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_torques(){
            std::vector<T_data> joint_states = bravo_io->get_bravo_joint_torques();
            // Map the std::vector to an Eigen::Vector<T_data, Eigen::Dynamic>
            //Copy all elements except for the last one which corresponds to the gripper joint state
            Eigen::Vector<T_data, Eigen::Dynamic> eigen_joint_states = Eigen::Map<Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), joint_states.size()-1);
            return eigen_joint_states;
        }
    
    template <bravo_control::Floating32or64 T_data>
        T_data bravo_handler<T_data>::get_manipulability(){
            return manipulability;
        }
    


    template <bravo_control::Floating32or64 T_data>       
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_sim_joint_whole_integration(){
            return sim_joint_whole_integration;
        }

    template <bravo_control::Floating32or64 T_data>  
    	void bravo_handler<T_data>::publish_RVIZ_sim_bravo_joint_states(Eigen::Vector<T_data, Eigen::Dynamic> q_arm, T_data gripper){
            sensor_msgs::msg::JointState msg_joint;
            msg_joint.header.stamp = rclcpp::Clock().now();  
            if (number_joints == 5) //bravo5
            {
                msg_joint.name = {"joint1", "joint2", "joint3", "joint4", "gripper"};
                msg_joint.position = {q_arm(0), q_arm(1), q_arm(2), q_arm(3), gripper};
            }  
            else
            {
                msg_joint.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"};
                msg_joint.position = {q_arm(0), q_arm(1), q_arm(2), q_arm(3), q_arm(4), q_arm(5), gripper};
            }
            pubJointsStateRviz->publish(msg_joint);
		}
    
    template <bravo_control::Floating32or64 T_data>  
    	void bravo_handler<T_data>::publish_bravo_joint_states(){
            sensor_msgs::msg::JointState msg_joint;
            msg_joint.header.stamp = rclcpp::Clock().now();   
            std::vector<T_data> position_jointFbd = bravo_io->get_bravo_joint_states(); 
            //std::vector<T_data> torque_jointFbd = bravo_io->get_bravo_joint_torques();
            if (position_jointFbd.size() == 7)
            { 
                msg_joint.name.resize(6);
                msg_joint.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
                msg_joint.position.resize(6);
                msg_joint.position = {position_jointFbd[0], position_jointFbd[1], position_jointFbd[2], position_jointFbd[3], position_jointFbd[4], position_jointFbd[5]};
                //msg_joint.effort = {torque_jointFbd[0], torque_jointFbd[1], torque_jointFbd[2], torque_jointFbd[3], torque_jointFbd[4], torque_jointFbd[5]};
                msg_joint.effort.resize(6); 
                msg_joint.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            }
            else if (position_jointFbd.size() == 5)
            {
                msg_joint.name.resize(4);
                msg_joint.name = {"joint1", "joint2", "joint3", "joint4"};
                msg_joint.position.resize(4);
                msg_joint.position = {position_jointFbd[0], position_jointFbd[1], position_jointFbd[2], position_jointFbd[3]};
                //msg_joint.effort   = {torque_jointFbd[0], torque_jointFbd[1], torque_jointFbd[2], torque_jointFbd[3]};
                msg_joint.effort.resize(4);
                msg_joint.effort = {0.0, 0.0, 0.0, 0.0};
            }
            pubFdbJointStates->publish(msg_joint);
        }
    
    template <bravo_control::Floating32or64 T_data>  
    	void bravo_handler<T_data>::publish_wrench_estimation_2(const geometry_msgs::msg::Wrench &wrench_estimation){
            pubWrenchEstimation_2->publish(wrench_estimation);
        }


    template <bravo_control::Floating32or64 T_data>  
    	void bravo_handler<T_data>::publish_wrench_estimation(const Eigen::Vector<T_data, Eigen::Dynamic> force_estimation){
            geometry_msgs::msg::WrenchStamped msg_wrench;
            msg_wrench.header.stamp = rclcpp::Clock().now();    
            msg_wrench.wrench.force.x = force_estimation(0);
            msg_wrench.wrench.force.y = force_estimation(1);
            msg_wrench.wrench.force.z = force_estimation(2);
            msg_wrench.wrench.torque.x = force_estimation(3);
            msg_wrench.wrench.torque.y = force_estimation(4);
            msg_wrench.wrench.torque.z = force_estimation(5);
            pubWrenchEstimation->publish(msg_wrench);
        }
    
    template <bravo_control::Floating32or64 T_data>  
    	void bravo_handler<T_data>::publish_force_estimation(const Eigen::Vector3d force_estimation){
            geometry_msgs::msg::Vector3Stamped msg_force; 
            msg_force.header.stamp = rclcpp::Clock().now();
            msg_force.vector.x = force_estimation(0);
            msg_force.vector.y = force_estimation(1);
            msg_force.vector.z = force_estimation(2);
            pubForceEstimation->publish(msg_force);
        }
    
    template <bravo_control::Floating32or64 T_data>  
    	T_data bravo_handler<T_data>::signedAngleDistance(T_data goal, T_data current) {
            T_data diff = fmod(goal - current, 2.0 * M_PI);
            if (diff < -M_PI) diff += 2.0 * M_PI;
            else if (diff > M_PI) diff -= 2.0 * M_PI;
            return diff;
    }
    template <bravo_control::Floating32or64 T_data>  
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::signedAngleDistance(const Eigen::Vector<T_data, Eigen::Dynamic>& goal, const Eigen::Vector<T_data, Eigen::Dynamic>& current) {
            Eigen::Vector<T_data, Eigen::Dynamic> diff = current - goal;
            for (int i = 0; i < diff.size(); ++i) {
                diff[i] = signedAngleDistance(goal[i], current[i]);
            }
            return diff;
    }

    template <bravo_control::Floating32or64 T_data>  
    	bool bravo_handler<T_data>::isConnected(){
            return bravo_io->isConnected();
        }
    
    template <bravo_control::Floating32or64 T_data>  
    	bool bravo_handler<T_data>::arm_is_healthy(const T_data max_time_without_fdb)
        {
            if (bravo_io->isConnected() && bravo_io->check_all_feedback_health(max_time_without_fdb))
            {
                return true;
            }
            else
            {
                return false;
            }
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::exceeding_joint_current_limit(Eigen::Vector<T_data, Eigen::Dynamic> q_cmd, Eigen::Vector<T_data, Eigen::Dynamic> MAX_CURRENT_mA){
            for (int i = 0; i < number_joints; i++) {
                    if (std::abs(q_cmd[i]) > std::abs(MAX_CURRENT_mA[i])){

                    }
                } 
        }

template class bravo_handler<double>;
//template class bravo_handler<float>; // PINOCCHIO IS INCOMPATIBLE WITH FLOATS