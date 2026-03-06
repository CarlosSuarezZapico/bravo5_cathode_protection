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
#include <stdexcept>

namespace bravo_control{ 

    template <typename T_data> 
        bravo_udp<T_data>::bravo_udp(bravo_control::ArmModel model,
                                     const std::string& ip,
                                     int port,
                                     bravo_utils::LogCallback cb)
            : arm_model(model), 
            number_joints(model == bravo_control::ArmModel::bravo5 ? 5 : 7),  // Set number of joints based on arm model
            control_mode_state(control_mode_states::joint_current_mode),
            command_mode_state(control_mode_states::joint_current_mode),
            running_loop(true),
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
            //! IMPORTANT:TIMEOUT FOR RECV FUNCTION IN SOCKET (PREVENTS THE PROGRAM TO GET STUCK IN RECV)
            struct timeval timeout;
            timeout.tv_sec = 0;  // Timeout in seconds
            timeout.tv_usec = 100000; // Timeout in microseconds Example: 2000 microseconds = 2 milliseconds
            if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Socket creation failed for bravo interface");
                throw std::runtime_error("[bravo_UDP]: Socket creation failed for bravo interface");
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
                close(sockfd);
                sockfd = -1;
                throw std::invalid_argument("[bravo_UDP]: Invalid IP address: " + ip);
            }
            BRAVO_LOG_INFO(logger_, "[bravo_UDP]: Using IP: ", ip, ", Port: ", port);
            if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1){
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Failed to connect to Bravo Arm! (errno =", errno, " (", strerror(errno), ")");
                close(sockfd);
                sockfd = -1;
                throw std::runtime_error("[bravo_UDP]: Failed to connect to Bravo Arm");
            }
            else {
                BRAVO_LOG_INFO(logger_, "[bravo_UDP]: ✅ Connected to bravo using IP: ", ip, " and port ", port, "...ok!");
            }
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
        bravo_udp<T_data>::~bravo_udp()
        {
            running_loop = false;

            if (sockfd >= 0) {
                close(sockfd);
                sockfd = -1;
            }

            delete[] encoded_jointFbd_request;
            delete[] encoded_single_jointCmd_request;
            delete[] encoded_localTwisteeCmd_request;
            delete[] encoded_feedback_request;
            delete[] encoded_whole_jointCmd_request;
            delete[] encoded_jointCmd_feedback_request;
            delete[] encoded_taskCmd_feedback_request;
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
                BRAVO_LOG_INFO(logger_, "[bravo_UDP]: ✅ Reconnection Successful!");
                return 0;  // Return 0 to indicate success
            }
        }

    template <typename T_data> 
        bool bravo_udp<T_data>::isConnected(const T_data max_time_without_fdb)
        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            if(options_fdb_cmd.joint_pos_fdb){
                std::chrono::duration<T_data> elapsed_pos_fbd= std::chrono::high_resolution_clock::now() - position_jointFdb.last_msg;
                if ((elapsed_pos_fbd.count() > max_time_without_fdb) || !position_jointFdb.received){
                    //BRAVO_LOG_WARN(logger_, "[bravo_UDP]:❗IsConnected? -> POSITION joint feedback not received");
                    return false;
                }
            }
            if(options_fdb_cmd.joint_vel_fdb){
                std::chrono::duration<T_data> elapsed_vel_fbd= std::chrono::high_resolution_clock::now() - velocity_jointFdb.last_msg;
                if ((elapsed_vel_fbd.count() > max_time_without_fdb) || !velocity_jointFdb.received){
                    //BRAVO_LOG_WARN(logger_, "[bravo_UDP]:❗IsConnected? -> VELOCITY joint feedback not received");
                    return false;
                }
            }
            if(options_fdb_cmd.joint_amp_fdb){
                std::chrono::duration<T_data> elapsed_current_fbd= std::chrono::high_resolution_clock::now() - current_jointFdb.last_msg;
                if ((elapsed_current_fbd.count() > max_time_without_fdb) || !current_jointFdb.received){
                    //BRAVO_LOG_WARN(logger_, "[bravo_UDP]:❗IsConnected? -> CURRENT joint feedback not received");
                    return false;
                }
            }
            if(options_fdb_cmd.joint_torque_fdb){
                std::chrono::duration<T_data> elapsed_torque_fbd= std::chrono::high_resolution_clock::now() - torque_jointFdb.last_msg;
                if ((elapsed_torque_fbd.count() > max_time_without_fdb) || !torque_jointFdb.received){
                    //BRAVO_LOG_WARN(logger_, "[bravo_UDP]:❗IsConnected? -> TORQUE joint feedback not received");
                    return false;
                }
            }
            return true;           
        }

    template <typename T_data>
        std::vector<T_data> bravo_udp<T_data>::get_bravo_joint_states() const
        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            if (options_fdb_cmd.joint_pos_fdb){
                return position_jointFdb.data;
            }
            else{
                throw std::runtime_error("[bravo_UDP]: ❌ Joint position feedback not configured to be returned");
            }   
        }
    
    template <typename T_data>
        std::vector<T_data> bravo_udp<T_data>::get_bravo_joint_velocities() const
        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            if (options_fdb_cmd.joint_vel_fdb){
                return velocity_jointFdb.data;
            }
            else{
                throw std::runtime_error("[bravo_UDP]: ❌ Joint velocities feedback not configured to be returned");
            }            
        }
    
    template <typename T_data>
        std::vector<T_data> bravo_udp<T_data>::get_bravo_joint_torques() const
        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            if (options_fdb_cmd.joint_torque_fdb){
                return torque_jointFdb.data;
            }
            else{
                throw std::runtime_error("[bravo_UDP]: ❌ Joint torques feedback not configured to be returned");
            }            
        }

    template <typename T_data>
        std::vector<T_data> bravo_udp<T_data>::get_bravo_joint_currents() const
        {
            std::lock_guard<std::mutex> lock(io_mutex_);
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
            std::lock_guard<std::mutex> lock(io_mutex_);
            return control_mode_state;
        }
    
    template <typename T_data>
        void bravo_udp<T_data>::set_control_mode(control_mode_states mode)
        {
            control_mode_states mode_to_send;
            switch (mode) {
                case control_mode_states::joint_position_mode:
                case control_mode_states::joint_current_mode:
                case control_mode_states::joint_torque_mode:
                case control_mode_states::joint_velocity_mode:
                    mode_to_send = mode;
                    break;
                case control_mode_states::local_twist_ee_mode:
                    mode_to_send = control_mode_states::joint_velocity_mode;
                    break;
                default:
                    throw std::invalid_argument("[bravo_UDP]: unsupported control mode for command streaming");
            }            

            {
                std::lock_guard<std::mutex> lock(io_mutex_);
                command_mode_state = mode;
            }
            set_mode_all_devices(mode_to_send);
        }

    template <typename T_data>
        void bravo_udp<T_data>::set_joint_cmd_velocity(Eigen::Vector<T_data, Eigen::Dynamic> cmd)
        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            const Eigen::Index cmd_size = cmd.size();
            if ((command_mode_state == control_mode_states::joint_velocity_mode) &&
                (cmd_size <= static_cast<Eigen::Index>(number_joints))) {
                for (Eigen::Index i = 0; i < cmd_size; ++i) {
                    joint_cmd_velocity[static_cast<size_t>(i)] = static_cast<T_data>(cmd[i]);
                }
            }
            else{
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Velocity command ignored: set control mode to JOINT VELOCITY first");
            }
        }

    template <typename T_data>    
        void bravo_udp<T_data>::set_joint_cmd_current(Eigen::Vector<T_data, Eigen::Dynamic>cmd)
        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            const Eigen::Index cmd_size = cmd.size();
            if ((command_mode_state == control_mode_states::joint_current_mode) &&
                (cmd_size <= static_cast<Eigen::Index>(number_joints))) {
                for (Eigen::Index i = 0; i < cmd_size; ++i) {
                    joint_cmd_current[static_cast<size_t>(i)] = static_cast<T_data>(cmd[i]);
                }
            }
            else{
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Current command ignored: set control mode to JOINT CURRENT first");
            }
        }
    
    template <typename T_data>    
        void bravo_udp<T_data>::set_joint_cmd_torque(Eigen::Vector<T_data, Eigen::Dynamic>cmd)
        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            const Eigen::Index cmd_size = cmd.size();
            if ((command_mode_state == control_mode_states::joint_torque_mode) &&
                (cmd_size <= static_cast<Eigen::Index>(number_joints))) {
                for (Eigen::Index i = 0; i < cmd_size; ++i) {
                    joint_cmd_torque[static_cast<size_t>(i)] = static_cast<T_data>(cmd[i]);
                }
            }
            else{
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Torque command ignored: set control mode to JOINT TORQUE first");
            }
        }

    template <typename T_data>
        void bravo_udp<T_data>::set_joint_cmd_position(Eigen::Vector<T_data, Eigen::Dynamic>cmd)
        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            const Eigen::Index cmd_size = cmd.size();
            if ((command_mode_state == control_mode_states::joint_position_mode) &&
                (cmd_size <= static_cast<Eigen::Index>(number_joints))) {
                for (Eigen::Index i = 0; i < cmd_size; ++i) {
                    joint_cmd_position[static_cast<size_t>(i)] = cmd(i);
                }
            }
            else{
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Position command ignored: set control mode to JOINT POSITION first");
            }
        }

    template <typename T_data>
        void bravo_udp<T_data>::set_local_twist_ee(Eigen::Vector<T_data, 6> twist_ee)
        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            if (command_mode_state == control_mode_states::local_twist_ee_mode){
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
            diagnosis.store(x, std::memory_order_relaxed);
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

            std::lock_guard<std::mutex> lock(io_mutex_);
            control_mode_state = mode;
        }

    template <typename T_data> 
        void bravo_udp<T_data>::set_frequency_requests(T_data frequency){
            if (frequency <= 0) throw std::invalid_argument("[bravo_UDP]: frequency must be > 0");
            std::lock_guard<std::mutex> lock(io_mutex_);
            request_time_delay_sec = T_data(1) / std::abs(frequency);
        }

    template <typename T_data>
        T_data bravo_udp<T_data>::get_rx_packet_frequency_hz() const
        {
            return rx_packet_frequency_hz.load(std::memory_order_relaxed);
        }

    template <typename T_data>
        void bravo_udp<T_data>::set_max_time_without_fdb(int timeout_ms)
        {
            if (timeout_ms <= 0) {
                throw std::invalid_argument("[bravo_UDP]: feedback timeout must be > 0 ms");
            }
            MAX_TIME_WITHOUT_FBD_ms.store(timeout_ms, std::memory_order_relaxed);
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
    
    template <typename T_data> 
        bool bravo_udp<T_data>::check_all_feedback_health(T_data MAX_TIME) 
        {
            std::lock_guard<std::mutex> lock(io_mutex_);
            const auto now = std::chrono::high_resolution_clock::now();
            auto check_feedback = [&](auto& fb, bool enabled) -> bool {
                if (!enabled) return true; // If feedback is not enabled, skip check
                if (!fb.received) return false;
                auto elapsed = std::chrono::duration<T_data>(now - fb.last_msg).count();
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
        void bravo_udp<T_data>::req_and_recv()
        {
            bool request_cmd_fdb = true; 
            bool pos_fdb_received = false, vel_fdb_received = false, torque_fdb_received = false, current_fdb_received = false;
            auto last_no_data_warn = std::chrono::steady_clock::now() - std::chrono::seconds(10);
            auto last_timeout_warn = std::chrono::steady_clock::now() - std::chrono::seconds(10);
            auto last_full_feedback_cycle = std::chrono::steady_clock::time_point{};
            bool have_last_full_feedback_cycle = false;
            while  (running_loop){           
                //& REQUESTING FEEDBACK AND COMMANDS AT A SPECIFIC FREQUENCY RATE
                T_data request_delay_sec = T_data(0);
                {
                    std::lock_guard<std::mutex> lock(io_mutex_);
                    elapsed_request = std::chrono::high_resolution_clock::now() - last_request_time;
                    request_delay_sec = request_time_delay_sec;
                }
                if ((elapsed_request.count() > request_delay_sec)){
                    //& REQUESTING AND SENDING PACKAGES -------------------------------------------------------------
                    if (request_cmd_fdb){                  
                        //* SEND ONLY FEEDBACK REQUESTS UNTIL FEEDBACK RECEIVED
                        request_command();
                        request_feedback();        
                        request_cmd_fdb = false; // Reset the flag after sending requests
                    }
                    //& RECEIVING AND PROCESSING PACKAGES -------------------------------------------------------------
                    std::array<uint8_t, 20> buffer{};
                    const ssize_t len = ::recv(sockfd, buffer.data(), buffer.size(), 0);
                    if (len <= 0) {
                        const auto now = std::chrono::steady_clock::now();
                        const int timeout_ms = MAX_TIME_WITHOUT_FBD_ms.load(std::memory_order_relaxed);
                        // Hard stop if no full feedback cycle for too long
                        if (have_last_full_feedback_cycle &&
                            (now - last_full_feedback_cycle > std::chrono::milliseconds(timeout_ms))) {
                            rx_packet_frequency_hz.store(T_data(0), std::memory_order_relaxed);
                            BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ Feedback timeout (>", timeout_ms,
                                            " ms). Communication with Bravo arm appears lost; stopping bravo_io thread.");
                            running_loop = false;
                            break;
                        }
                        if (len == 0) {
                            // UDP: valid zero-length datagram, not peer-closed connection
                            if (now - last_no_data_warn > std::chrono::milliseconds(50)) {
                                BRAVO_LOG_WARN(logger_, "[bravo_UDP]:❗recv() returned 0-byte UDP datagram");
                                last_no_data_warn = now;
                            }
                            request_cmd_fdb = true;
                            continue;
                        }
                        const int recv_errno = errno; // valid only for len < 0
                        if (recv_errno == EINTR) {
                            continue; // interrupted by signal
                        }
                        if (recv_errno == EWOULDBLOCK || recv_errno == EAGAIN) {
                            if (now - last_timeout_warn > std::chrono::milliseconds(50)) {
                                BRAVO_LOG_WARN(logger_, "[bravo_UDP]:❗recv() timed out");
                                last_timeout_warn = now;
                            }
                            request_cmd_fdb = true;
                            continue;
                        }
                        BRAVO_LOG_ERROR(logger_, "[bravo_UDP]: ❌ recv() failed with error: ", std::strerror(recv_errno));
                        // Optional: reconnect only on hard socket/network errors
                        if (recv_errno == ENOTCONN || recv_errno == ECONNRESET ||
                            recv_errno == ENETDOWN || recv_errno == ENETUNREACH) {
                            reconnect();
                            request_cmd_fdb = true;
                        }
                        continue;
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
                                bravoDiagnoseWarnings(packets[i], diagnosis.load(std::memory_order_relaxed));
                            }
                            else{
                                BRAVO_LOG_WARN(logger_, "[bravo_UDP]:❗Unknown packet ID received: ",
                                               packets[i].packetID);
                            }
                        }
                        //* MAKE SURE ALL FEEDBACKS ARE RECEIVED TO REQUEST MORE DATA
                        if ((options_fdb_cmd.joint_pos_fdb == pos_fdb_received) && (options_fdb_cmd.joint_amp_fdb == current_fdb_received) 
                                && (options_fdb_cmd.joint_vel_fdb == vel_fdb_received) && (options_fdb_cmd.joint_torque_fdb == torque_fdb_received)){
                            const auto now = std::chrono::steady_clock::now();
                            if (have_last_full_feedback_cycle) {
                                const T_data dt = std::chrono::duration<T_data>(now - last_full_feedback_cycle).count();
                                if (dt > T_data(1e-6)) {
                                    rx_packet_frequency_hz.store(T_data(1) / dt, std::memory_order_relaxed);
                                }
                            }
                            last_full_feedback_cycle = now;
                            have_last_full_feedback_cycle = true;

                            request_cmd_fdb = true;
                            pos_fdb_received = false, vel_fdb_received = false, torque_fdb_received = false, current_fdb_received = false;
                        }
                    }   
                }
                else {
                    std::this_thread::sleep_for(std::chrono::microseconds(100));
                }
            }
        }

    template <typename T_data>
        void bravo_udp<T_data>::stop_io_loop()
        {
            running_loop = false;
        }


}
