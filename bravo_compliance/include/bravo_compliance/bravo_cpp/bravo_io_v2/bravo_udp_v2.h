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

//bravo libraries
#include "bravo_compliance/bravo_cpp/bravo_io_v2/packetID.h"
#include "bravo_compliance/bravo_cpp/bravo_io_v2/bplprotocol_unite.h"
#include "bravo_compliance/bravo_cpp/utils/utils.h"


using namespace std;
using namespace bravo_utils;

namespace bravo_control{ 
    enum class ArmModel { bravo5, bravo7 };
    enum control_mode_states {error=-2, unknown=-1, standby=0x00, disable=0x01, joint_position_mode=0x02, joint_velocity_mode=0x03, joint_current_mode=0x04, joint_torque_mode=0x0B, local_twist_ee_mode=5}; // Bravo list of control modes
    enum fdb_return{joint_pos, joint_current, FT_ee};
    
    inline void printLogEveryXSeconds(const std::string& level, const std::string& message, float seconds) {
        using Clock = std::chrono::steady_clock;
        static std::unordered_map<std::string, Clock::time_point> lastPrintMap;
        auto now = Clock::now();
        auto& lastPrint = lastPrintMap[message];  // Creates entry if not exists
        float elapsed = std::chrono::duration<float>(now - lastPrint).count();
        if (elapsed >= seconds) {
            if (level == "INFO") {
                std::cout << "[INFO] " << message << std::endl;
            } else if (level == "WARN") {
                std::cout << "[WARN] " << message << std::endl;
            } else if (level == "ERROR") {
                std::cerr << "[ERROR] " << message << std::endl;
            } else {
                std::cout << "[INFO] " << message << std::endl;
            }
            lastPrint = now;
        }
    }

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

    template<Floating32or64 T>
    using EigenVectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;

    //& BRAVO_UDP CLASS TO HANDLE LOW-LEVEL UDP COMMUNICATION WITH BRAVO7 MANIPULATOR
    template <Floating32or64 T_data> 
        class bravo_udp {
            private:
                std::atomic<bool> running_loop;
                ArmModel arm_model;

                size_t number_joints;

                control_mode_states control_mode_state; 
                io_options options_fdb_cmd; //& IO OPTIONS

                //& SAFETY VARIABLES
                static constexpr T_data MAX_TIME_WIHTOUT_FBD                = 0.3; //seconds
                static constexpr T_data MAXIMUM_Q_DISTANCE                  = 0.8; //in rads 10 deg

                //& FEEDBACK FROM BRAVO
                //$ JOINT FEEDBACK
                feedback<T_data> position_jointFdb, current_jointFdb, velocity_jointFdb, torque_jointFdb;
                vector<int> device_ids; // gripper, joint6, joint5, joint4, joint3, joint2, joint1
                bool diagnosis = true;

                //& COMMANDS
                bool cmd_initial = true; 
                vector<T_data> joint_cmd_position; 
                vector<T_data> joint_cmd_velocity;
                vector<T_data> joint_cmd_current ;
                vector<T_data> joint_cmd_torque  ;
                vector<T_data> twist_ee_cmd  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Local Twist ee in m/s and rad/s

                //& socket UDP for communication
                int sockfd;
                struct sockaddr_in servaddr; 

                //& REQUEST AND RECEIVE FREQUENCIES
                T_data request_time_delay_sec  = 0.0005;
                unsigned int REQ_FDB_PER_CMD = 5; 
                std::chrono::high_resolution_clock::time_point last_request_time;
                std::chrono::duration<T_data> elapsed_request;

                //&CONTROL MODE
                packetID control_mode_packet = packetID::POSITION;
                
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
                /**
                 * @brief Constructor UDP_BravoInterfaceNode, with a derivation from class Node to 
                 * use publishers and subscribers
                 * TODO: WORKING
                 */
                bravo_udp(bravo_control::ArmModel model, const std::string& ip, int port = 6789);
                /**
                 * @brief If connection is lost we tried to reconnect to bravo using the UDP socket
                 * TODO: NOT TESTED
                 */
                int reconnect();
                /**
                 * @brief Returns true if the connection with bravo is established and with continuous feedback
                 * Makes checks on the feedback configured to be received (options_fdb_cmd)
                 * TODO: NOT TESTED
                 */
                bool isConnected(const T_data max_time_without_fdb = 0.3);
                /**
                 * @brief Returns the current control mode of bravo
                 * enum control_mode_states {error=-2, unknown=-1, standby=0x00, disable=0x01, position=0x02, velocity=0x03, current=0x04, local_twist_ee=5};
                 * TODO: WORKING
                 */
                control_mode_states get_control_mode();
                /**
                 * @brief Sets the current control mode of bravo
                 * enum control_mode_states {error=-2, unknown=-1, standby=0x00, disable=0x01, position=0x02, velocity=0x03, current=0x04, local_twist_ee=5};
                 * TODO: WORKING
                 */
                void set_control_mode(control_mode_states mode);

                /**
                 * @brief Command whole joint position to bravo
                 * TODO: WORKING
                 */
                void set_joint_cmd_velocity(Eigen::Vector<T_data, Eigen::Dynamic>cmd);
                /**
                 * @brief Command whole joint current to bravo
                 * TODO: WORKING
                 */
                void set_joint_cmd_current(Eigen::Vector<T_data, Eigen::Dynamic>cmd);
                /**
                 * @brief Command whole joint position to bravo
                 * TODO: WORKING
                 */
                void set_joint_cmd_position(Eigen::Vector<T_data, Eigen::Dynamic>cmd);
                /**
                 * @brief Command whole joint torque to bravo
                 * TODO: WORKING
                 */
                void set_joint_cmd_torque(Eigen::Vector<T_data, Eigen::Dynamic>cmd);
                /**
                 * @brief Command the local twist of the end effector when in local twist control mode
                 * @param twist_ee array with the twist of the end effector [vx, vy, vz, wx, wy, wz] in m/s and rad/s
                 * TODO: WORKING
                 */
                void set_local_twist_ee(Eigen::Vector<T_data, 6> twist_ee);
                /**
                 * @brief Configure the diagnosis to be printed out on screen
                 * TODO: WORKING
                 */
                void set_dianosis_debug(bool x);
 
                void debug_jointFdb_last_msg_times();
                                
                std::vector<T_data>& get_bravo_joint_states();

                std::vector<T_data>& get_bravo_joint_currents();

                std::vector<T_data>& get_bravo_joint_velocities();

                std::vector<T_data>& get_bravo_joint_torques();

                std::chrono::high_resolution_clock::time_point get_position_jointFdb_last_msg_time();

                bool get_position_jointFdb_received();

                size_t get_number_joints();
                
                /**
                 * @brief Function to set the operating mode of all axis
                 * TODO: NOT TESTED
                 */
                void set_mode_all_devices(control_mode_states mode);

                /**
                 * @brief Function to set the frequency of the requests to bravo
                 */
                void set_frequency_requests(T_data frequency);

                /**
                 * @brief Function to check we are receiving the feedback at safety frequency
                 * @param MAX_TIME maximum time allowed for delay
                 * @param time_point point in time where we received the last reading
                 *  TODO: WORKING
                 */
                bool check_feedback_rate(T_data MAX_TIME, std::chrono::high_resolution_clock::time_point time_point);
                 
                /**
                 * @brief Function to check we are receiving all feedback at safety frequency
                 * The feedback is checked based on the options set in options_fdb_cmd
                 * @param MAX_TIME maximum time allowed for last feedback reading
                 * @return true if all feedbacks are received within the MAX_TIME, false otherwise
                 *  TODO: WORKING
                 */
                bool check_all_feedback_health(T_data MAX_TIME); 
                /**
                 * @brief Function to calculate the difference between two joint configurations
                 *  if the difference between two joints is greater than the tolerance, the function returns false
                 *  TODO: WORKING
                 */
                bool joint_configuration_difference_safety(std::vector<T_data> joint_configuration_1, std::vector<T_data> joint_configuration_2, T_data max_tolerance);
                /**
                 * @brief Function thread to request feedback information periodically and motion commands
                 *  The data requests rate is kept constant to prevent instability of the program.
                 *  TODO: WORKING
                 */
                void req_and_recv();                 

                void processDiagnosticByte(uint8_t byte, int byteIndex);

                void init_prebuilt_packets();
        };

        template class bravo_udp<double>;
        template class bravo_udp<float>;
}
#endif 