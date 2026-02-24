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
#include "bravo5_cp/bravo_cpp/bravo_handler/bravo_handler_v2.h"

template <bravo_control::Floating32or64 T_data>
    bravo_handler<T_data>::bravo_handler(const std::string urdf_filename, const std::string &toolLink, const std::string& ip, int port, bool ROS_enable) : rclcpp::Node("bravo7_io_node"),
                    kinodynamics(urdf_filename, toolLink), bravo_io(nullptr){

            
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
                bravo_io = std::make_unique<bravo_control::bravo_udp<T_data>>(bravo_control::ArmModel::bravo5, ip, port);
                max_q_vel.resize(4); sim_joint_whole_integration.resize(4); motor_constants.resize(4);  
                motor_constants << 0.222, 0.222, 0.215, 0.209;
                max_q_vel << 1.0, 1.0, 1.0, 1.0;
                sim_joint_whole_integration << 1.7, 2.7, 0.66, 0.0;
            }
            else if (robot_name == "bravo7"){
                RCLCPP_INFO(this->get_logger(), "[bravo_handler]: ✅ Initializing BRAVO 7 model");
                //bravo_io = bravo_control::bravo_udp<T_data>(bravo_control::bravo_udp<T_data>::ArmModel::BRAVO_7, "192.168.2.51");
                bravo_io = std::make_unique<bravo_control::bravo_udp<T_data>>(bravo_control::ArmModel::bravo7, ip, port);
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
                pubFdbJointStates      = this->create_publisher<sensor_msgs::msg::JointState>       ("/bravo/fdb/joint_states", 10);
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
            //& INTIALIZE EE TWIST COMMAND
            cmdLocalTwist.data << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            cmdLocalTwist.last_update = std::chrono::high_resolution_clock::now();

            //&START BRAVO_IO THREAD for input and output
            bravo_io->set_dianosis_debug(true); //& SET TO TRUE FOR DEBUGGING
            bravo_io_run_thread = true;
            bravo_io_thread = std::thread(&bravo_handler::bravo_io_thread_function, this); 
            
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
        void bravo_handler<T_data>::cmdJointVelocity(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointVel){
            if(cmdJointVel.size() == number_joints){
                bravo_io->set_joint_cmd_velocity(cmdJointVel);
            }
            else{
                RCLCPP_FATAL(this->get_logger(), "[bravo_handler]: ❌ Number of components received for bravo_handler::cmdJointVel does not match the number of joints");
                rclcpp::shutdown(); // Clean up ROS 2 resources
                std::exit(EXIT_FAILURE); // Exit the program with an error code
            }
		}
    
    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::cmdJointVelocity_SAT(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointVel, const T_data MAX){
            if(cmdJointVel.size() == number_joints){
                Eigen::Vector<T_data, Eigen::Dynamic> joint_cmd_velocity(number_joints);
                for (int i = 0; i < number_joints; i++){
                    joint_cmd_velocity(i) = VAL_SAT<T_data>(cmdJointVel(i), MAX, -MAX);
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
            const std::vector<T_data>& joint_states = bravo_io->get_bravo_joint_states(); // no copy

            if (joint_states.size() < 1) {
                throw std::runtime_error("[bravo_handler]: joint_states empty");
            }
            const Eigen::Index n = static_cast<Eigen::Index>(joint_states.size() - 1); // exclude gripper
            if (n <= 0) {
                throw std::runtime_error("[bravo_handler]: no non-gripper joints");
            }

            // Copy into Eigen vector (safe)
            return Eigen::Map<const Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), n);
        }
    
    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_velocities(){
            const std::vector<T_data>& joint_states = bravo_io->get_bravo_joint_velocities(); // no copy

            if (joint_states.size() < 1) {
                throw std::runtime_error("[bravo_handler]: joint_states empty");
            }
            const Eigen::Index n = static_cast<Eigen::Index>(joint_states.size() - 1); // exclude gripper
            if (n <= 0) {
                throw std::runtime_error("[bravo_handler]: no non-gripper joints");
            }

            // Copy into Eigen vector (safe)
            return Eigen::Map<const Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), n);
        }
    

    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_currents(){
            const std::vector<T_data>& joint_states = bravo_io->get_bravo_joint_currents(); // no copy

            if (joint_states.size() < 1) {
                throw std::runtime_error("[bravo_handler]: joint_states empty");
            }
            const Eigen::Index n = static_cast<Eigen::Index>(joint_states.size() - 1); // exclude gripper
            if (n <= 0) {
                throw std::runtime_error("[bravo_handler]: no non-gripper joints");
            }

            // Copy into Eigen vector (safe)
            return Eigen::Map<const Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), n);
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
            std::vector<T_data> velocity_jointFbd = bravo_io->get_bravo_joint_velocities(); 
            std::vector<T_data> current_jointFbd  = bravo_io->get_bravo_joint_currents();
            if (position_jointFbd.size() == 7) //! to improve
            { 
                msg_joint.name.resize(6);
                msg_joint.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
                msg_joint.position.resize(6);
                msg_joint.position = {position_jointFbd[0], position_jointFbd[1], position_jointFbd[2], position_jointFbd[3], position_jointFbd[4], position_jointFbd[5]};
                msg_joint.velocity.resize(6);
                msg_joint.velocity = {velocity_jointFbd[0], velocity_jointFbd[1], velocity_jointFbd[2], velocity_jointFbd[3], velocity_jointFbd[4], velocity_jointFbd[5]};
                msg_joint.effort.resize(6); 
                msg_joint.effort = {current_jointFbd[0], current_jointFbd[1], current_jointFbd[2], current_jointFbd[3], current_jointFbd[4], current_jointFbd[5]};
            }
            else if (position_jointFbd.size() == 5)//! to improve
            {
                msg_joint.name.resize(4);
                msg_joint.name = {"joint1", "joint2", "joint3", "joint4"};
                msg_joint.position.resize(4);
                msg_joint.position = {position_jointFbd[0], position_jointFbd[1], position_jointFbd[2], position_jointFbd[3]};
                msg_joint.velocity.resize(4);
                msg_joint.velocity = {velocity_jointFbd[0], velocity_jointFbd[1], velocity_jointFbd[2], velocity_jointFbd[3]};
                msg_joint.effort.resize(4);
                msg_joint.effort = {current_jointFbd[0], current_jointFbd[1], current_jointFbd[2], current_jointFbd[3]};
            }
            pubFdbJointStates->publish(msg_joint);
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