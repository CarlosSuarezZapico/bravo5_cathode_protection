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
#include "bravo_cathode_protection/bravo_cpp/bravo_handler/bravo_handler_v2.h"
#include <iostream>


template <bravo_control::Floating32or64 T_data>
    bravo_handler<T_data>::bravo_handler(const std::string urdf_filename,
                                         const std::string &toolLink,
                                         bravo_control::ArmModel robot_model,
                                         const std::string& ip,
                                         int port,
                                         std::shared_ptr<bravo_utils::Logger> shared_logger)
                    : kinodynamics(urdf_filename, toolLink),
                    logger_(shared_logger),
                    bravo_io(nullptr){

            if (!logger_) {
                logger_ = std::make_shared<bravo_utils::Logger>();
            }
            std::string resolved_ip = ip;

            auto udp_log_callback = [this](bravo_utils::LogLevel lvl, std::string_view msg) {
                logger_->log(lvl, msg);
            };

            switch (robot_model) {
                case bravo_control::ArmModel::bravo5:
                    BRAVO_LOG_INFO((*logger_), "[bravo_handler]: Initializing BRAVO 5 model");
                    if (resolved_ip.empty()) {
                        resolved_ip = "10.43.0.146";
                        BRAVO_LOG_WARN((*logger_), "[bravo_handler]: No IP provided, using default BRAVO 5 IP: ", resolved_ip);
                    }
                    bravo_io = std::make_unique<bravo_control::bravo_udp<T_data>>(bravo_control::ArmModel::bravo5, resolved_ip, port, udp_log_callback);
                    motor_constants.resize(4);
                    motor_constants << 0.222, 0.222, 0.215, 0.209;
                    break;

                case bravo_control::ArmModel::bravo7:
                    BRAVO_LOG_INFO((*logger_), "[bravo_handler]: Initializing BRAVO 7 model");
                    if (resolved_ip.empty()) {
                        resolved_ip = "192.168.2.51";
                        BRAVO_LOG_WARN((*logger_), "[bravo_handler]: No IP provided, using default BRAVO 7 IP: ", resolved_ip);
                    }
                    bravo_io = std::make_unique<bravo_control::bravo_udp<T_data>>(bravo_control::ArmModel::bravo7, resolved_ip, port, udp_log_callback);
                    motor_constants.resize(6);
                    motor_constants << 0.222, 0.222, 0.215, 0.215, 0.215, 0.209;
                    break;

                default:
                    BRAVO_LOG_ERROR((*logger_), "[bravo_handler]: ❌ Unsupported robot model enum value");
                    std::exit(EXIT_FAILURE);
            }

            number_joints = bravo_io->get_number_joints()-1; //without gripper       
            //& DEFINING BRAVO JOINT LIMITS  
            //* The first, fourth and sixth joints are continuous so they have no limits
            T_data angle_limit_smooth = 10;
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

            //&START BRAVO_IO THREAD for input and output
            bravo_io->set_dianosis_debug(true); //& SET TO TRUE FOR DEBUGGING
            bravo_io_run_thread = true;
            bravo_io_thread = std::thread(&bravo_handler::bravo_io_thread_function, this); 
            
        }

    template <bravo_control::Floating32or64 T_data>
        bravo_handler<T_data>::~bravo_handler(){
            bravo_io_run_thread = false;
            if (bravo_io) {
                bravo_io->stop_io_loop();
            }
            if (bravo_io_thread.joinable()) {
                bravo_io_thread.join();
            }
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::bravo_io_thread_function(){
            if (bravo_io) {
                bravo_io->req_and_recv();
            }
            bravo_io_run_thread = false;
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::set_control_mode(set_bravo_control_mode mode){
            switch (mode)
            {
            case set_bravo_control_mode::position_control:
                bravo_io->set_mode_all_devices(bravo_control::control_mode_states::joint_position_mode);
                break;
            case set_bravo_control_mode::velocity_control:
                bravo_io->set_mode_all_devices(bravo_control::control_mode_states::joint_velocity_mode);
                break;	
            case set_bravo_control_mode::current_control:
                bravo_io->set_mode_all_devices(bravo_control::control_mode_states::joint_current_mode);
                break;	
            case set_bravo_control_mode::torque_control:
                bravo_io->set_mode_all_devices(bravo_control::control_mode_states::joint_torque_mode);
                break;		
            default:
                bravo_io->set_mode_all_devices(bravo_control::control_mode_states::joint_position_mode);
                BRAVO_LOG_ERROR((*logger_), "[bravo_handler]: ❌ Control mode not recognized, setting to position control by default");
                break;
            }
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::set_bravo_frequency_packet_exchange(T_data freq){
            bravo_io->set_frequency_requests(freq);
        }

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::cmdJointCurrent(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointCurrent){
            const Eigen::Index joint_count = static_cast<Eigen::Index>(number_joints);
            if(cmdJointCurrent.size() == joint_count){
                bravo_io->set_joint_cmd_current(cmdJointCurrent);
            }
            else{
                BRAVO_LOG_ERROR((*logger_), "[bravo_handler]: ❌ Number of components received for cmdJointCurrent does not match the number of joints");
                std::exit(EXIT_FAILURE); // Exit the program with an error code
            }
		}
    
    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::cmdJointCurrent_SAT(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointCurrent, const T_data MAX){
            const Eigen::Index joint_count = static_cast<Eigen::Index>(number_joints);
            if(cmdJointCurrent.size() == joint_count){
                Eigen::Vector<T_data, Eigen::Dynamic> joint_cmd_current(joint_count);
                for (Eigen::Index i = 0; i < joint_count; i++){
                    if ((cmdJointCurrent(i) > MAX) || (cmdJointCurrent(i) < -MAX)) {
                        BRAVO_LOG_WARN((*logger_),
                                       "[bravo_handler]:❗cmdJointCurrent_SAT clipping joint ",
                                        i + 1,
                                       " current command from ",
                                       cmdJointCurrent(i),
                                       " to range [",
                                       -MAX,
                                       ", ",
                                       MAX,
                                       "]");
                    }
                    joint_cmd_current(i) = bravo_utils::VAL_SAT<T_data>(cmdJointCurrent(i), MAX, -MAX);
                }
                bravo_io->set_joint_cmd_current(joint_cmd_current);
            }
            else{
                BRAVO_LOG_ERROR((*logger_), "[bravo_handler]: ❌ Number of components received for cmdJointCurrent_SAT does not match the number of joints");
                std::exit(EXIT_FAILURE); // Exit the program with an error code
            }
		}

    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::cmdJointVelocity(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointVel){
            const Eigen::Index joint_count = static_cast<Eigen::Index>(number_joints);
            if(cmdJointVel.size() == joint_count){
                bravo_io->set_joint_cmd_velocity(cmdJointVel);
            }
            else{
                BRAVO_LOG_ERROR((*logger_), "[bravo_handler]: ❌ Number of components received for cmdJointVelocity does not match the number of joints");
                std::exit(EXIT_FAILURE); // Exit the program with an error code
            }
		}
    
    template <bravo_control::Floating32or64 T_data>
        void bravo_handler<T_data>::cmdJointVelocity_SAT(const Eigen::Vector<T_data, Eigen::Dynamic> &cmdJointVel, const T_data MAX){
            const Eigen::Index joint_count = static_cast<Eigen::Index>(number_joints);
            if(cmdJointVel.size() == joint_count){
                Eigen::Vector<T_data, Eigen::Dynamic> joint_cmd_velocity(joint_count);
                for (Eigen::Index i = 0; i < joint_count; i++){
                    joint_cmd_velocity(i) = bravo_utils::VAL_SAT<T_data>(cmdJointVel(i), MAX, -MAX);
                }
                bravo_io->set_joint_cmd_velocity(joint_cmd_velocity);
            }
            else{
                BRAVO_LOG_ERROR((*logger_), "[bravo_handler]: ❌ Number of components received for cmdJointVelocity_SAT does not match the number of joints");
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
            const Eigen::Index joint_count = static_cast<Eigen::Index>(number_joints);
            Eigen::Vector<T_data, Eigen::Dynamic> error; 
            error.resize(joint_count);
            for (Eigen::Index i = 0; i < joint_count; i++){   
                error[i] = abs(fmod(goal_joint_position[i] - current_joint_position[i]+ 3 * M_PI, 2 * M_PI) - M_PI); //THE JOINTS HAVE A RANGE FROM 0.0 TO 6.28, for that reason we use this formula 
                if (error[i] > tolerance){
                    return false;
                }
            }
            return true;
        }

    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::std_2_eigen(std::vector<T_data> std_vector){
            const Eigen::Index joint_count = static_cast<Eigen::Index>(number_joints);
            Eigen::Vector<T_data, Eigen::Dynamic> eigen_vector;
            eigen_vector.resize(joint_count);
            for (Eigen::Index i = 0; i < joint_count; i++){
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
            const Eigen::Index joint_count = static_cast<Eigen::Index>(joint_states.size() - 1);
            Eigen::Vector<T_data, Eigen::Dynamic> eigen_joint_states = Eigen::Map<Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), joint_count);
            joint_penalization.resize(joint_count);   
            for(Eigen::Index i = 0; i < joint_count; i++){
                joint_penalization(i) = weight_joint_limit_smooth(eigen_joint_states(i), lowerJointLimits(i), lowerJointInfLimits(i), upperJointInfLimits(i), upperJointLimits(i));
            }
            return joint_penalization;
        }
    
    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_penalization(std::vector<T_data> joint_states){
            Eigen::Vector<T_data, Eigen::Dynamic> joint_penalization;
            const Eigen::Index joint_count = static_cast<Eigen::Index>(joint_states.size());
            Eigen::Vector<T_data, Eigen::Dynamic> eigen_joint_states = Eigen::Map<Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), joint_count);
            joint_penalization.resize(joint_count);   
            for(Eigen::Index i = 0; i < joint_count; i++){
                joint_penalization(i) = weight_joint_limit_smooth(eigen_joint_states(i), lowerJointLimits(i), lowerJointInfLimits(i), upperJointInfLimits(i), upperJointLimits(i));
            }
            return joint_penalization;
        }

    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_states(){
            const std::vector<T_data>& joint_states = bravo_io->get_bravo_joint_states(); // no copy

            if (joint_states.size() < 1) {
                throw std::runtime_error("[bravo_handler]: ❌ joint_states empty");
            }
            const Eigen::Index n = static_cast<Eigen::Index>(joint_states.size() - 1); // exclude gripper
            if (n <= 0) {
                throw std::runtime_error("[bravo_handler]: ❌ no non-gripper joints");
            }

            // Copy into Eigen vector (safe)
            return Eigen::Map<const Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), n);
        }
    
    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_velocities(){
            const std::vector<T_data>& joint_states = bravo_io->get_bravo_joint_velocities(); // no copy

            if (joint_states.size() < 1) {
                throw std::runtime_error("[bravo_handler]: ❌ joint_states empty");
            }
            const Eigen::Index n = static_cast<Eigen::Index>(joint_states.size() - 1); // exclude gripper
            if (n <= 0) {
                throw std::runtime_error("[bravo_handler]: ❌ no non-gripper joints");
            }

            // Copy into Eigen vector (safe)
            return Eigen::Map<const Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), n);
        }
    

    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_currents(){
            const std::vector<T_data>& joint_states = bravo_io->get_bravo_joint_currents(); // no copy

            if (joint_states.size() < 1) {
                throw std::runtime_error("[bravo_handler]: ❌ joint_states empty");
            }
            const Eigen::Index n = static_cast<Eigen::Index>(joint_states.size() - 1); // exclude gripper
            if (n <= 0) {
                throw std::runtime_error("[bravo_handler]: ❌ no non-gripper joints");
            }

            // Copy into Eigen vector (safe)
            return Eigen::Map<const Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), n);
        }

    template <bravo_control::Floating32or64 T_data>
        Eigen::Vector<T_data, Eigen::Dynamic> bravo_handler<T_data>::get_bravo_joint_torques(){
            const std::vector<T_data>& joint_states = bravo_io->get_bravo_joint_torques(); // no copy

            if (joint_states.size() < 1) {
                throw std::runtime_error("[bravo_handler]: ❌ joint_states empty");
            }
            const Eigen::Index n = static_cast<Eigen::Index>(joint_states.size() - 1); // exclude gripper
            if (n <= 0) {
                throw std::runtime_error("[bravo_handler]: ❌ no non-gripper joints");
            }

            // Copy into Eigen vector (safe)
            return Eigen::Map<const Eigen::Vector<T_data, Eigen::Dynamic>>(joint_states.data(), n);
        }

    template <bravo_control::Floating32or64 T_data>
        size_t bravo_handler<T_data>::get_number_joints(){
            return number_joints;
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
            for (Eigen::Index i = 0; i < diff.size(); ++i) {
                diff[i] = signedAngleDistance(goal[i], current[i]);
            }
            return diff;
    }

    template <bravo_control::Floating32or64 T_data>  
    	bool bravo_handler<T_data>::isConnected(){
            return bravo_io->isConnected();
        }

    template <bravo_control::Floating32or64 T_data>
        T_data bravo_handler<T_data>::get_udp_rx_frequency_hz() const
        {
            return bravo_io->get_rx_packet_frequency_hz();
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

template class bravo_handler<double>;
//template class bravo_handler<float>; // PINOCCHIO IS INCOMPATIBLE WITH FLOATS
