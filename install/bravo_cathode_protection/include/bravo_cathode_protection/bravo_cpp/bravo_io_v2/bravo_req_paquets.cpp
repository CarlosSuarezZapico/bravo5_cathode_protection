/**
 *    @file  bravo_req_paquets.cpp
 *    @brief Prebuilt UDP request packets for the Bravo manipulator
 *    Depending on the desired feedback adn command, the UDP request packets are built
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  22-Jun-2025
 *    Modification 25-Nov-2025
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */

#include "bravo_cathode_protection/bravo_cpp/bravo_io_v2/bravo_udp_v2.h"

namespace bravo_control{ 

    template <typename T> 
        void bravo_udp<T>::init_prebuilt_packets()
        {
            //Packages are concatenated to request multiple feedbacks at once           
            encoded_single_jointCmd_request    = new uint8_t[size_per_jointCmd_request];
            encoded_localTwisteeCmd_request    = new uint8_t[size_Local_Twist_ee_request];
            size_whole_jointCmd_request        = number_joints * size_per_jointCmd_request; //6 is the size of packetID, deviceID and dataLength
            encoded_whole_jointCmd_request     = new uint8_t[size_whole_jointCmd_request];                      

            //& Build feedback request packets from configuration 
            std::tie(encoded_feedback_request, size_feedback_request) = encoded_req_jointFdb_paquet();  
            //*Prebuild joint command + feedback request packet
            size_jointCmd_feedback_request = size_feedback_request + number_joints * size_per_jointCmd_request;
            encoded_jointCmd_feedback_request = new uint8_t[size_jointCmd_feedback_request]; 
            std::copy(encoded_feedback_request, encoded_feedback_request + size_feedback_request, encoded_jointCmd_feedback_request);  
            //*Prebuild task command + feedback request packet
            size_taskCmd_feedback_request = size_feedback_request + size_Local_Twist_ee_request;
            encoded_taskCmd_feedback_request = new uint8_t[size_taskCmd_feedback_request]; 
            std::copy(encoded_feedback_request, encoded_feedback_request + size_feedback_request, encoded_taskCmd_feedback_request);  
        }
  
    template <typename T> 
        std::pair<uint8_t*, std::size_t> bravo_udp<T>::encoded_req_jointFdb_paquet()
        {
            Packet whole_jointFbd_request; 
            whole_jointFbd_request.packetID = packetID::REQUEST;
            whole_jointFbd_request.deviceID = 0xFF;
            std::size_t index = 0;
            if (options_fdb_cmd.joint_pos_fdb) {
                whole_jointFbd_request.data[index++] = packetID::POSITION;
            }
            if (options_fdb_cmd.joint_amp_fdb) {
                whole_jointFbd_request.data[index++] = packetID::CURRENT;
            }
            if (options_fdb_cmd.joint_vel_fdb) {
                whole_jointFbd_request.data[index++] = packetID::VELOCITY;
            }
            if (options_fdb_cmd.joint_torque_fdb) {
                whole_jointFbd_request.data[index++] = packetID::TORQUE;
            }
            whole_jointFbd_request.dataLength = index;      
            size_t size_whole_jointFdb_request;
            if(index ==0){
                BRAVO_LOG_ERROR(logger_, "[bravo_UDP]:❌ No feedback configure to be requested");
                size_whole_jointFdb_request = 0; 
            } 
            else{
                size_whole_jointFdb_request = 6 + index; //6 is the size of the packetID, deviceID and dataLength
            }
            uint8_t *encoded_jointFbd_request = new uint8_t[size_whole_jointFdb_request];   
            memset(encoded_jointFbd_request, 0, size_whole_jointFdb_request); 
            encodePacket_unite(encoded_jointFbd_request, &whole_jointFbd_request);
            return std::make_pair(encoded_jointFbd_request, size_whole_jointFdb_request);
        }

    template <typename T> 
        void bravo_udp<T>::request_feedback()
        {
            //& REQUEST ALL FEEDBACK
            sendto(sockfd, encoded_feedback_request, size_feedback_request , 0, (const struct sockaddr *)&servaddr, sizeof(servaddr));
            last_request_time = std::chrono::high_resolution_clock::now();
        }
    
    template <typename T> 
        void bravo_udp<T>::request_command()
        {
            //& REQUEST COMMAND AND FEEDBACK
            packetID cmd_packetID = packetID::POSITION; // Default command packet ID
            bool joint_command = true; // Flag to check if joint command is set
            std::vector<T> joint_cmd; 
            //* DETERMINE CURRENT CONTROL MODE
            switch (options_fdb_cmd.control_mode_cmd){
                case control_mode_states::joint_position_mode:
                    cmd_packetID = packetID::POSITION;
                    joint_cmd = joint_cmd_position;
                    break;                
                case control_mode_states::joint_velocity_mode:
                    cmd_packetID = packetID::VELOCITY;
                    joint_cmd = joint_cmd_velocity;
                    break;
                case control_mode_states::joint_torque_mode:
                    cmd_packetID = packetID::TORQUE;
                    joint_cmd = joint_cmd_torque;
                    break;
                case control_mode_states::joint_current_mode:
                    cmd_packetID = packetID::CURRENT;
                    joint_cmd = joint_cmd_current;
                    break;              
                case control_mode_states::local_twist_ee_mode:
                    cmd_packetID = packetID::KM_END_VEL_LOCAL;
                    joint_command = false; // No joint command for local twist
                    break;         
                default:
                    break;
            }
            //* WHOLE JOINT COMMAND PACKET
            if (joint_command)
            {
                for(int i= 0; i<number_joints; i++){
                    cmdJoint_encodePacket_unite(encoded_single_jointCmd_request, cmd_packetID, device_ids[i], joint_cmd[i]);                         
                    std::copy(encoded_single_jointCmd_request, encoded_single_jointCmd_request + size_per_jointCmd_request, encoded_whole_jointCmd_request + i * size_per_jointCmd_request);       
                }
                sendto(sockfd, encoded_whole_jointCmd_request, number_joints * size_per_jointCmd_request, 0, (const struct sockaddr *)&servaddr, sizeof(servaddr)); 
            }
            //* TASK-SPACE COMMAND PACKET
            else{
                float twist_ee_cmd_list[6];
                for (size_t i = 0; i < 6; ++i)
                    twist_ee_cmd_list[i] = static_cast<float>(twist_ee_cmd[i]);
                encode_packet_float_list(encoded_localTwisteeCmd_request, cmd_packetID, 0x0E, twist_ee_cmd_list, 6);
                sendto(sockfd, encoded_localTwisteeCmd_request, size_Local_Twist_ee_request , 0, (const struct sockaddr *)&servaddr, sizeof(servaddr)); //10 is the size of each joint packet
            }
            last_request_time = std::chrono::high_resolution_clock::now();
        }

    template <typename T> 
        void bravo_udp<T>::request_command_and_feedback(){
            //& REQUEST COMMAND AND FEEDBACK
            packetID cmd_packetID = packetID::POSITION; // Default command packet ID
            bool joint_command = true; // Flag to check if joint command is set
            std::vector<T> joint_cmd; 
            //* DETERMINE CURRENT CONTROL MODE
            switch (options_fdb_cmd.control_mode_cmd)
            {
                case control_mode_states::joint_position_mode:
                    cmd_packetID = packetID::POSITION;
                    joint_cmd = joint_cmd_position;
                    break;                
                case control_mode_states::joint_velocity_mode:
                    cmd_packetID = packetID::VELOCITY;
                    joint_cmd = joint_cmd_velocity;
                    break;
                case control_mode_states::joint_torque_mode:
                    cmd_packetID = packetID::TORQUE;
                    joint_cmd = joint_cmd_torque;
                    break;
                case control_mode_states::joint_current_mode:
                    cmd_packetID = packetID::CURRENT;
                    joint_cmd = joint_cmd_current;
                    break;              
                case control_mode_states::local_twist_ee_mode:
                    cmd_packetID = packetID::KM_END_VEL_LOCAL;
                    joint_command = false; // No joint command for local twist
                    break;         
                default:
                    break;
            }
            //* WHOLE JOINT COMMAND PACKET
            if (joint_command)
            {
                for(int i= 0; i<number_joints; i++){
                    cmdJoint_encodePacket_unite(encoded_single_jointCmd_request, cmd_packetID, device_ids[i], joint_cmd[i]);                         
                    std::copy(encoded_single_jointCmd_request, encoded_single_jointCmd_request + size_per_jointCmd_request, encoded_jointCmd_feedback_request + size_feedback_request + i * size_per_jointCmd_request);       
                }
                sendto(sockfd, encoded_jointCmd_feedback_request, number_joints * size_per_jointCmd_request + size_feedback_request, 0, (const struct sockaddr *)&servaddr, sizeof(servaddr)); 
            }
            //* TASK-SPACE COMMAND PACKET
            else
            {
                float twist_ee_cmd_list[6];
                for (size_t i = 0; i < 6; ++i)
                    twist_ee_cmd_list[i] = static_cast<float>(twist_ee_cmd[i]);
                encode_packet_float_list(encoded_localTwisteeCmd_request, cmd_packetID, 0x0E, twist_ee_cmd_list, 6);
                std::copy(encoded_localTwisteeCmd_request,  encoded_localTwisteeCmd_request + size_Local_Twist_ee_request,  encoded_taskCmd_feedback_request + size_feedback_request); 
                sendto(sockfd, encoded_taskCmd_feedback_request, size_feedback_request + size_Local_Twist_ee_request , 0, (const struct sockaddr *)&servaddr, sizeof(servaddr)); //10 is the size of each joint packet
            }
            last_request_time = std::chrono::high_resolution_clock::now();
        }
}


