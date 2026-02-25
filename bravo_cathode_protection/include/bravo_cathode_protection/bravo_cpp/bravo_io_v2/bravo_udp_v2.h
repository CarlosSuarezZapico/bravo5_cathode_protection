/**
 *    @file  bravo_udp_v2.h
 *    @brief UDP interface for bravo in cpp, not dependendent on ROS2 on purpose
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Project UNITE
 *    Created      24-Nov-2025
 *    Modification 24-Nov-2025
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */

#ifndef _BRAVO_UDP_
#define _BRAVO_UDP_

#include <vector>
#include <thread>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <algorithm>
#include <unordered_map>
#include <string>

//bravo libraries
#include "bravo_cathode_protection/bravo_cpp/bravo_io_v2/packetID.h"
#include "bravo_cathode_protection/bravo_cpp/bravo_io_v2/bplprotocol_unite.h"
#include "bravo_cathode_protection/bravo_cpp/utils/utils.h"
#include "bravo_cathode_protection/bravo_cpp/utils/bravo_logger.h"


namespace bravo_control
{ 
    enum class ArmModel { bravo5, bravo7 };
    enum control_mode_states {error=-2, unknown=-1, standby=0x00, disable=0x01, joint_position_mode=0x02, joint_velocity_mode=0x03, joint_current_mode=0x04, joint_torque_mode=0x0B, local_twist_ee_mode=5}; // Bravo list of control modes
  
    struct io_options{
        //* INPUTS
        bool joint_amp_fdb    = true; 
        bool joint_torque_fdb = false;
        bool joint_pos_fdb    = true;
        bool joint_vel_fdb    = true;
        //* OUTPUTS
        control_mode_states control_mode_cmd = control_mode_states::joint_current_mode; // Default control mode is joint position
    };

    template <typename T> 
        struct feedback{    
            std::vector<T> data;                                      // Main information of the variable (joint pos, joint torques....)  
            bool received = false;                                    // Structure used to know if topic received and
            std::chrono::high_resolution_clock::time_point last_msg;  // elapsed time since last msg received 
            std::vector<bool> joints_received;
            // Constructor to initialize joints_received array to false
            feedback(size_t N_joints){
                data.resize(N_joints);
                joints_received.resize(N_joints);
                joints_received.assign(N_joints, false);
            }
        }; 

    template<typename T_data>
    concept Floating32or64 = std::is_same_v<T_data, float> || std::is_same_v<T_data, double>;

    //& BRAVO_UDP CLASS TO HANDLE LOW-LEVEL UDP COMMUNICATION WITH BRAVO7 MANIPULATOR
    template <Floating32or64 T_data> 
        class bravo_udp {
            private:
                std::atomic<bool> running_loop;
                ArmModel arm_model;

                size_t number_joints;

                control_mode_states control_mode_state; 
                io_options options_fdb_cmd; //& IO OPTIONS

                //& FEEDBACK FROM BRAVO
                //$ JOINT FEEDBACK
                feedback<T_data> position_jointFdb, current_jointFdb, velocity_jointFdb, torque_jointFdb;
                std::vector<int> device_ids; // gripper, joint6, joint5, joint4, joint3, joint2, joint1
                bool diagnosis = true;

                //& COMMANDS
                std::vector<T_data> joint_cmd_position; 
                std::vector<T_data> joint_cmd_velocity;
                std::vector<T_data> joint_cmd_current ;
                std::vector<T_data> joint_cmd_torque  ;
                std::vector<T_data> twist_ee_cmd  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Local Twist ee in m/s and rad/s

                //& socket UDP for communication
                int sockfd;
                struct sockaddr_in servaddr; 

                //& REQUEST AND RECEIVE FREQUENCIES
                T_data request_time_delay_sec  = 0.0005;
                std::chrono::high_resolution_clock::time_point last_request_time;
                std::chrono::duration<T_data> elapsed_request;
                
                //&PACKET READER 
                PacketReader  packet_reader;   

                //& PREBUILT PACKETS FOR COMMANDS AND REQUESTS
                size_t size_per_jointCmd_request              = 10; //BPLProtocol.encode_packet(2, PacketID.VELOCITY, BPLProtocol.encode_floats([-10.1])) is 10 bytes in size per joint
                size_t size_Local_Twist_ee_request            = 30; //BPLProtocol.encode_packet(0x0E, PacketID.KM_END_VEL_LOCAL, BPLProtocol.encode_floats([0.0, 0.0, 0.0, 1.0, 1.0 ,1.0]))
      
                uint8_t *encoded_jointFbd_request;       
                uint8_t *encoded_single_jointCmd_request;     
                uint8_t *encoded_localTwisteeCmd_request;

                //& NEW
                size_t size_feedback_request;
                uint8_t *encoded_feedback_request;

                size_t size_whole_jointCmd_request; 
                uint8_t *encoded_whole_jointCmd_request;

                size_t size_jointCmd_feedback_request;
                uint8_t *encoded_jointCmd_feedback_request;

                size_t size_taskCmd_feedback_request;
                uint8_t *encoded_taskCmd_feedback_request;

                //& LOGGER
                bravo_utils::Logger logger_;

            //& PRIVATE METHODS
            private:
                std::pair<uint8_t*, std::size_t> encoded_req_fdb_paquet();

                std::pair<uint8_t*, std::size_t> encoded_req_jointFdb_paquet();

                void request_command_and_feedback();

                void request_feedback();

                void request_command();

                bool processJointFdbPacket(Packet& packet, feedback<T_data>& joint_feedback);

                void processControlModePacket(Packet& packet);

                void bravoDiagnoseWarnings(Packet& packet, bool diagnosisEnabled);
        
            public:            
            //& METHODS        

                bravo_udp(bravo_control::ArmModel model,
                          const std::string& ip,
                          int port = 6789,
                          bravo_utils::LogCallback cb = {});

                int reconnect();

                bool isConnected(const T_data max_time_without_fdb = 0.3);
                
                control_mode_states get_control_mode();

                void set_control_mode(control_mode_states mode);

                void set_joint_cmd_velocity(Eigen::Vector<T_data, Eigen::Dynamic>cmd);

                void set_joint_cmd_current(Eigen::Vector<T_data, Eigen::Dynamic>cmd);

                void set_joint_cmd_position(Eigen::Vector<T_data, Eigen::Dynamic>cmd);

                void set_joint_cmd_torque(Eigen::Vector<T_data, Eigen::Dynamic>cmd);

                void set_local_twist_ee(Eigen::Vector<T_data, 6> twist_ee);

                void set_dianosis_debug(bool x);
 
                void debug_jointFdb_last_msg_times();
                                
                const std::vector<T_data>& get_bravo_joint_states() const;

                const std::vector<T_data>& get_bravo_joint_currents() const;

                const std::vector<T_data>& get_bravo_joint_velocities() const;

                const std::vector<T_data>& get_bravo_joint_torques() const;

                std::chrono::high_resolution_clock::time_point get_position_jointFdb_last_msg_time();

                bool get_position_jointFdb_received();

                size_t get_number_joints();

                void set_mode_all_devices(control_mode_states mode);

                void set_frequency_requests(T_data frequency);

                bool check_feedback_rate(T_data MAX_TIME, std::chrono::high_resolution_clock::time_point time_point);
                 
                bool check_all_feedback_health(T_data MAX_TIME); 

                bool joint_configuration_difference_safety(std::vector<T_data> joint_configuration_1, std::vector<T_data> joint_configuration_2, T_data max_tolerance);

                void req_and_recv();                 

                void processDiagnosticByte(uint8_t byte, int byteIndex);

                void init_prebuilt_packets();

                void set_logger(bravo_utils::LogCallback cb) { logger_.set_callback(std::move(cb)); }
        };

        template class bravo_udp<double>;
        template class bravo_udp<float>;
}
#endif 
