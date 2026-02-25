/**
 *    @file  bravo_udp_v2.cpp
 *    @brief UDP interface for bravo in cpp
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  17-Nov-2023
 *    Modification 10-Jul-2025
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */

#include "bravo_cathode_protection/bravo_cpp/bravo_io_v2/bravo_udp_v2.h"

namespace bravo_control{ 

    template <typename T_data> 
        bravo_udp<T_data>::bravo_udp(bravo_control::ArmModel model,
                                     const std::string& ip,
                                     int port,
                                     bravo_utils::LogCallback cb)
            : running_loop(true), arm_model(model), 
            number_joints(model == bravo_control::ArmModel::bravo5 ? 5 : 7),  // Set number of joints based on arm model
            position_jointFdb(number_joints), 
            current_jointFdb(number_joints), 
            velocity_jointFdb(number_joints), 
            torque_jointFdb(number_joints), 
            joint_cmd_position(number_joints), 
            joint_cmd_velocity(number_joints), 
            joint_cmd_current(number_joints), 
            joint_cmd_torque(number_joints),
            logger_(std::move(cb))
        {
            //& UDP SOCKET CLIENT 
            //*TIMEOUT FOR RECV FUNCTION IN SOCKET
            struct timeval timeout;
            timeout.tv_sec = 0;  // Timeout in seconds
            timeout.tv_usec = 100000; // Timeout in microseconds Example: 2000 microseconds = 2 milliseconds
            if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
                //! ERROR: Socket creation failed
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Socket creation failed for bravo interface");
                std::exit(EXIT_FAILURE); 
            } 
            //& IMPORTANT: Socket timeout setting has to be done after socket creation
            if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout)) < 0) {
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Error setting socket timeout");
            }
            memset(&servaddr, 0, sizeof(servaddr));        
            servaddr.sin_family = AF_INET; 
            servaddr.sin_port = htons(port); // 
            if (inet_aton(ip.c_str(), &servaddr.sin_addr) == 0) {
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Invalid IP address: ", ip);
                std::exit(EXIT_FAILURE);            
            }
            BRAVO_LOG_INFO(logger_, "[bravo_UDP]: Using IP: ", ip, ", Port: ", port);
            // servaddr.sin_addr.s_addr = inet_addr("192.168.2.51"); // IP 192.168.2.4
            // std::cout << "[bravo_UDP]: Connected at IP: " << inet_ntoa(servaddr.sin_addr) << ", PORT: " << port << std::endl;
            if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1){
                //! ERROR: Failed to connect to the host!
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Failed to connect to Bravo Arm! (errno =",
                                errno, " (", strerror(errno), ")");
                close(sockfd);
                control_mode_state = control_mode_states::error;
                std::exit(EXIT_FAILURE);
            }
            else {
                BRAVO_LOG_INFO(logger_, "[bravo_UDP]: ✅ Connected to bravo using IP: ",
                               ip, "and port ", port, "...ok!");
            }
            control_mode_state = control_mode_states::joint_current_mode;
            // Initialize the joint positions (example: home position)
            joint_cmd_position = std::vector<T_data>(number_joints, 0.0);
            joint_cmd_velocity = std::vector<T_data>(number_joints, 0.0);
            joint_cmd_current  = std::vector<T_data>(number_joints, 0.0);
            joint_cmd_torque   = std::vector<T_data>(number_joints, 0.0);
            // Set device IDs based on arm model
            if (arm_model == bravo_control::ArmModel::bravo5) {
                device_ids.resize(5);
                device_ids = {0x05, 0x04, 0x03, 0x02, 0x01};  // For 5-DOF arm
            } else if (arm_model == bravo_control::ArmModel::bravo7) {
                device_ids.resize(7);
                device_ids = {0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01};  // For 7-DOF arm
            }
                //& BUILDING REQUESTING PACKAGING 
                init_prebuilt_packets();
            }

    template <typename T_data>
        int bravo_udp<T_data>::reconnect() 
        {
            if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1){
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Failed to connect to Bravo Arm! (errno =",
                                errno, " (", strerror(errno), ")");
                return -1;  // Return -1 to indicate failure
            }
            else {
                BRAVO_LOG_INFO(logger_, "[bravo_UDP]: ✅ Connection Successful!");
                return 0;  // Return 0 to indicate success
            }
        }

    template <typename T_data> 
        bool bravo_udp<T_data>::isConnected(const T_data max_time_without_fdb)
        {
            if(options_fdb_cmd.joint_pos_fdb){
                std::chrono::duration<T_data> elapsed_pos_fbd= std::chrono::high_resolution_clock::now() - position_jointFdb.last_msg;
                if ((elapsed_pos_fbd.count() > max_time_without_fdb) || !position_jointFdb.received){
                    //std::cout << "[bravo_UDP]: IsConnected? -> ❗POSITION joint feedback not received" << std::endl;
                    return false;
                }
            }
            if(options_fdb_cmd.joint_vel_fdb){
                std::chrono::duration<T_data> elapsed_vel_fbd= std::chrono::high_resolution_clock::now() - velocity_jointFdb.last_msg;
                if ((elapsed_vel_fbd.count() > max_time_without_fdb) || !velocity_jointFdb.received){
                    //std::cout << "[bravo_UDP]: IsConnected? -> ❗ VELOCITY joint feedback not received" << std::endl;
                    return false;
                }
            }
            if(options_fdb_cmd.joint_amp_fdb){
                std::chrono::duration<T_data> elapsed_current_fbd= std::chrono::high_resolution_clock::now() - current_jointFdb.last_msg;
                if ((elapsed_current_fbd.count() > max_time_without_fdb) || !current_jointFdb.received){
                    //std::cout << "[bravo_UDP]: IsConnected? -> ❗CURRENT joint feedback not received" << std::endl;
                    return false;
                }
            }
            if(options_fdb_cmd.joint_torque_fdb){
                std::chrono::duration<T_data> elapsed_torque_fbd= std::chrono::high_resolution_clock::now() - torque_jointFdb.last_msg;
                if ((elapsed_torque_fbd.count() > max_time_without_fdb) || !torque_jointFdb.received){
                    //std::cout << "[bravo_UDP]: IsConnected? -> ❗TORQUE joint feedback not received" << std::endl;
                    return false;
                }
            }
            return true;           
        }

    template <typename T_data>
        const std::vector<T_data>& bravo_udp<T_data>::get_bravo_joint_states() const
        {
            if (options_fdb_cmd.joint_pos_fdb){
                return position_jointFdb.data;
            }
            else{
                throw std::runtime_error("[bravo_UDP]: ❌ Joint position feedback not configured to be returned");
            }   
        }
    
    template <typename T_data>
        const std::vector<T_data>& bravo_udp<T_data>::get_bravo_joint_velocities() const
        {
            if (options_fdb_cmd.joint_vel_fdb){
                return velocity_jointFdb.data;
            }
            else{
                throw std::runtime_error("[bravo_UDP]: ❌ Joint velocities feedback not configured to be returned");
            }            
        }
    
    template <typename T_data>
        const std::vector<T_data>& bravo_udp<T_data>::get_bravo_joint_torques() const
        {
            if (options_fdb_cmd.joint_torque_fdb){
                return torque_jointFdb.data;
            }
            else{
                throw std::runtime_error("[bravo_UDP]: ❌ Joint torques feedback not configured to be returned");
            }            
        }

    template <typename T_data>
        const std::vector<T_data>& bravo_udp<T_data>::get_bravo_joint_currents() const
        {
            if (options_fdb_cmd.joint_amp_fdb){
                return current_jointFdb.data;
            }
            else{
                throw std::runtime_error("[bravo_UDP]: ❌ Joint currents feedback not configured to be returned");
            }            
        }

    template <typename T_data>
        control_mode_states bravo_udp<T_data>::get_control_mode()
        {
            return control_mode_state;
        }
    
    template <typename T_data>
        void bravo_udp<T_data>::set_control_mode(control_mode_states mode)
        {
            control_mode_state = mode;
            switch (control_mode_state)
            {  
                case control_mode_states::joint_position_mode:
                    set_mode_all_devices(control_mode_states::joint_position_mode); 
                    options_fdb_cmd.control_mode_cmd = control_mode_states::joint_position_mode;
                    break;
                case control_mode_states::joint_current_mode:
                    set_mode_all_devices(control_mode_states::joint_current_mode); 
                    options_fdb_cmd.control_mode_cmd = control_mode_states::joint_current_mode;
                    break;
                case control_mode_states::joint_torque_mode:
                    set_mode_all_devices(control_mode_states::joint_torque_mode); 
                    options_fdb_cmd.control_mode_cmd = control_mode_states::joint_torque_mode;
                    break;
                case control_mode_states::joint_velocity_mode:
                    set_mode_all_devices(control_mode_states::joint_velocity_mode); 
                    options_fdb_cmd.control_mode_cmd = control_mode_states::joint_velocity_mode;
                    break;      
                case control_mode_states::local_twist_ee_mode:
                    set_mode_all_devices(control_mode_states::joint_velocity_mode);
                    options_fdb_cmd.control_mode_cmd = control_mode_states::joint_velocity_mode;
                    break;                   
                default:
                    BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Control Mode not defined, programming error");
                    std::exit(EXIT_FAILURE);  // Terminate the program with failure status
                    break;
            }            
        }

    template <typename T_data>
        void bravo_udp<T_data>::set_joint_cmd_velocity(Eigen::Vector<T_data, Eigen::Dynamic> cmd)
        {
            if ((control_mode_state == control_mode_states::joint_velocity_mode) && (cmd.size() <= number_joints)){
                for (int i = 0; i < cmd.size(); ++i) {
                    joint_cmd_velocity[i] = static_cast<T_data>(cmd[i]);
                }
            }
            else{
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Changing to Joint Velocity Control Mode");
                options_fdb_cmd.control_mode_cmd = control_mode_states::joint_velocity_mode;
            }
        }

    template <typename T_data>    
        void bravo_udp<T_data>::set_joint_cmd_current(Eigen::Vector<T_data, Eigen::Dynamic>cmd)
        {
            if ((options_fdb_cmd.control_mode_cmd == control_mode_states::joint_current_mode) && (cmd.size() <= number_joints)){
                for (int i = 0; i < cmd.size(); ++i) {
                    joint_cmd_current[i] = static_cast<T_data>(cmd[i]);
                }
            }
            else{
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Changing to Joint Current Control Mode");
                options_fdb_cmd.control_mode_cmd = control_mode_states::joint_current_mode;
            }
        }
    
    template <typename T_data>    
        void bravo_udp<T_data>::set_joint_cmd_torque(Eigen::Vector<T_data, Eigen::Dynamic>cmd)
        {
            if ((options_fdb_cmd.control_mode_cmd == control_mode_states::joint_torque_mode) && (cmd.size() <= number_joints)){
                for (int i = 0; i < cmd.size(); ++i) {
                    joint_cmd_torque[i] = static_cast<T_data>(cmd[i]);
                }
            }
            else{
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Torque command not possible in this control mode");
            }
        }

    template <typename T_data>
        void bravo_udp<T_data>::set_joint_cmd_position(Eigen::Vector<T_data, Eigen::Dynamic>cmd)
        {
            if ((options_fdb_cmd.control_mode_cmd == control_mode_states::joint_position_mode) && (cmd.size() <= number_joints)){
                for (int i = 0; i < cmd.size(); ++i) {
                    joint_cmd_position[i] = cmd(i);
                }
            }
            else{
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Changing to Joint Position Control Mode");
                options_fdb_cmd.control_mode_cmd = control_mode_states::joint_position_mode;
            }
        }

    template <typename T_data>
        void bravo_udp<T_data>::set_local_twist_ee(Eigen::Vector<T_data, 6> twist_ee)
        {
            if (options_fdb_cmd.control_mode_cmd == control_mode_states::local_twist_ee_mode){
                //twist_ee is given in m/s and twist_ee_cmd in mm/s
                twist_ee_cmd[0] = twist_ee[0] * 1000.0; 
                twist_ee_cmd[1] = twist_ee[1] * 1000.0;
                twist_ee_cmd[2] = twist_ee[2] * 1000.0;
                twist_ee_cmd[3] = twist_ee[3];
                twist_ee_cmd[4] = twist_ee[4];
                twist_ee_cmd[5] = twist_ee[5];       
            }
            else{
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Local twist command not possible in this control mode");
            }     
        }

    template <typename T_data>    
        void bravo_udp<T_data>::set_dianosis_debug(bool x){
            diagnosis = x;
        }
    
    template <typename T_data> 
        void bravo_udp<T_data>::debug_jointFdb_last_msg_times()
        {
            auto now = std::chrono::high_resolution_clock::now();
            //& POSITION JOINT FEEDBACK
            if (options_fdb_cmd.joint_pos_fdb && position_jointFdb.received) {
                std::chrono::duration<T_data> elapsed = now - position_jointFdb.last_msg;
                BRAVO_LOG_INFO(logger_, "[bravo_UDP]: Position joint feedback time elapsed between two last readings: ",
                               elapsed.count(), " seconds");
            }
            //& AMPERAGE JOINT FEEDBACK
            if (options_fdb_cmd.joint_amp_fdb && current_jointFdb.received) {
                std::chrono::duration<T_data> elapsed = now - current_jointFdb.last_msg;
                BRAVO_LOG_INFO(logger_, "[bravo_UDP]: Amperage joint feedback time elapsed between two last readings: ",
                               elapsed.count(), " seconds");
            }
            //& VELOCITY JOINT FEEDBACK
            if (options_fdb_cmd.joint_vel_fdb && velocity_jointFdb.received) {
                std::chrono::duration<T_data> elapsed = now - velocity_jointFdb.last_msg;
                BRAVO_LOG_INFO(logger_, "[bravo_UDP]: Velocity joint feedback time elapsed between two last readings: ",
                               elapsed.count(), " seconds");
            }
            // & TORQUE JOINT FEEDBACK
            if (options_fdb_cmd.joint_torque_fdb && torque_jointFdb.received) {
                std::chrono::duration<T_data> elapsed = now - torque_jointFdb.last_msg;
                BRAVO_LOG_INFO(logger_, "[bravo_UDP]: Torque joint feedback time elapsed between two last readings: ",
                               elapsed.count(), " seconds");
            }
        }
    
    template <typename T_data> 
        std::chrono::high_resolution_clock::time_point bravo_udp<T_data>::get_position_jointFdb_last_msg_time()
        {
            return position_jointFdb.last_msg;
        }

    template <typename T_data> 
        bool bravo_udp<T_data>::get_position_jointFdb_received(){
            return position_jointFdb.received;
        }
    
    template <typename T_data> 
        size_t bravo_udp<T_data>::get_number_joints(){
            return number_joints;
        }
    
    template <typename T_data> 
        void bravo_udp<T_data>::set_mode_all_devices(control_mode_states mode)
        {
            Packet set_mode_packet; 
            set_mode_packet.packetID = packetID::MODE;
            set_mode_packet.deviceID = 0xFF;
            set_mode_packet.data[0] = mode;
            set_mode_packet.dataLength =1;
            uint8_t set_mode_encoded_packet[7];
            memset(set_mode_encoded_packet, 0, 7);  
            encodePacket_unite(set_mode_encoded_packet, &set_mode_packet);
            send(sockfd, set_mode_encoded_packet, sizeof(set_mode_encoded_packet), 0);
        }

    template <typename T_data> 
        void bravo_udp<T_data>::set_frequency_requests(T_data frequency){
            if (frequency <= 0) throw std::invalid_argument("[bravo_UDP]: frequency must be > 0");
            request_time_delay_sec = T_data(1) / std::abs(frequency);
        }
    
    template <typename T_data> 
        bool bravo_udp<T_data>::check_feedback_rate(T_data MAX_TIME, std::chrono::high_resolution_clock::time_point time_point)
        {
            std::chrono::duration<T_data> elapsed_counter = std::chrono::high_resolution_clock::now() - time_point;
            if (elapsed_counter.count() > MAX_TIME){
                return false;
            }
            else{
                return true;
            }
        }
    
    //! to complete
    template <typename T_data>
        bool bravo_udp<T_data>::check_all_feedback_health(T_data MAX_TIME) 
        {
            auto check_feedback = [&](auto& fb, bool enabled) -> bool {
                if (!enabled) return true; // If feedback is not enabled, skip check
                if (!fb.received) return false;
                auto elapsed = std::chrono::duration<float>( std::chrono::high_resolution_clock::now() - fb.last_msg).count();
                return elapsed <= MAX_TIME;
            };
            // Check all enabled feedbacks
            if (!check_feedback(position_jointFdb,    options_fdb_cmd.joint_pos_fdb))       return false;
            if (!check_feedback(velocity_jointFdb,    options_fdb_cmd.joint_vel_fdb))       return false;
            if (!check_feedback(torque_jointFdb,      options_fdb_cmd.joint_torque_fdb))    return false;
            if (!check_feedback(current_jointFdb,     options_fdb_cmd.joint_amp_fdb))       return false;
            return true;
        }

    template <typename T_data> 
        bool bravo_udp<T_data>::joint_configuration_difference_safety(std::vector<T_data> joint_configuration_1, std::vector<T_data> joint_configuration_2, T_data max_tolerance)
        {
            T_data maximum = 0.0;
            size_t n_joints = number_joints - 1; //! NO GRIPPER 
            std::vector<T_data> qdiff(n_joints);
            for (int i=0; i<n_joints; i++){   
                qdiff[i] = abs(fmod(joint_configuration_1[i] - joint_configuration_2[i]+ 3 * M_PI, 2 * M_PI) - M_PI); //THE JOINTS HAVE A RANGE FROM 0.0 TO 6.28, for that reason we use this formula 
                if (qdiff[i] > maximum){
                    maximum = qdiff[i];
                }
            }
            if ((maximum > max_tolerance)){
                return false;
            }
            else{
                return true;
            }                
        }  

    template <typename T_data> 
        void bravo_udp<T_data>::req_and_recv()
        {
            bool request_cmd_fdb = true; 
            bool pos_fdb_received = false, vel_fdb_received = false, torque_fdb_received = false, current_fdb_received = false;
            while  (running_loop){           
                //& REQUESTING FEEDBACK AND COMMANDS AT A SPECIFIC FREQUENCY RATE
                elapsed_request = std::chrono::high_resolution_clock::now()-last_request_time;
                if ((elapsed_request.count() > request_time_delay_sec)){
                    //& REQUESTING AND SENDING PACKAGES -------------------------------------------------------------
                    if (request_cmd_fdb){                  
                        //* SEND ONLY FEEDBACK REQUESTS UNTIL FEEDBACK RECEIVED
                        request_command();
                        request_feedback();        
                        request_cmd_fdb = false; // Reset the flag after sending requests
                    }
                    //& RECEIVING AND PROCESSING PACKAGES -------------------------------------------------------------
                    std::array<uint8_t, 20> buffer{};
                    int len = ::recv(sockfd, buffer.data(), buffer.size(), 0);
                    //* SET TIMEOUT TO AVOID BLOCKING
                    if (len < 0) {
                        //* ERROR: No data received AFTER TIMEOUT
                        BRAVO_LOG_WARN(logger_, "[bravo_UDP]: recv() No data received from Bravo Arm");
                        if (errno == EWOULDBLOCK || errno == EAGAIN) {
                            BRAVO_LOG_WARN(logger_, "[bravo_UDP]:❗recv() timed out");
                            request_cmd_fdb = true; // Set the flag to request feedback again
                        } else {
                           BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ recv() failed with error: ", strerror(errno));
                        }
                        continue; // Skip to the next iteration of the loop
                    }
                    //* PROCESSING RECEIVED PACKETS
                    else
                    {
                        std::vector<Packet> packets;
                        std::vector<uint8_t> vec_buffer(buffer.begin(), buffer.begin() + len);
                        packets = packet_reader.receive_bytes_decode(vec_buffer);                    
                        for (size_t i = 0; i < packets.size(); i++) {
                            if (packets[i].packetID == packetID::POSITION) {
                                if (processJointFdbPacket(packets[i], position_jointFdb)){
                                    pos_fdb_received = true;
                                }       
                            }
                            else if (packets[i].packetID == packetID::VELOCITY) {
                                if (processJointFdbPacket(packets[i], velocity_jointFdb)){
                                    vel_fdb_received = true;
                                } 
                            }
                            else if (packets[i].packetID == packetID::CURRENT) {
                                if (processJointFdbPacket(packets[i], current_jointFdb))
                                    current_fdb_received = true;
                            }
                            else if (packets[i].packetID == packetID::TORQUE) {
                                if (processJointFdbPacket(packets[i], torque_jointFdb))
                                    torque_fdb_received = true;
                            }
                            else if (packets[i].packetID == packetID::MODE) {
                                processControlModePacket(packets[i]);
                            }
                            //* PLOT IN TERMINAL INCOMING WARNING MESSAGES FROM BRAVO7
                            else if (packets[i].packetID == 0x68) {
                                bravoDiagnoseWarnings(packets[i], diagnosis);
                            }
                            else{
                                BRAVO_LOG_WARN(logger_, "[bravo_UDP]:❗Unknown packet ID received: ",
                                               packets[i].packetID);
                            }
                        }
                        //* MAKE SURE ALL FEEDBACKS ARE RECEIVED TO REQUEST MORE DATA
                        if ((options_fdb_cmd.joint_pos_fdb == pos_fdb_received) && (options_fdb_cmd.joint_amp_fdb == current_fdb_received) 
                                && (options_fdb_cmd.joint_vel_fdb == vel_fdb_received) && (options_fdb_cmd.joint_torque_fdb == torque_fdb_received)){
                            request_cmd_fdb = true;
                            pos_fdb_received = false, vel_fdb_received = false, torque_fdb_received = false, current_fdb_received = false;
                        }
                    }   
                    last_request_time = std::chrono::high_resolution_clock::now(); // Update the last request time
                }
            }
        }


}
