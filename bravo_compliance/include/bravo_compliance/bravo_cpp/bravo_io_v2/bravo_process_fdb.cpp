/**
 *    @file  bravo_process_fdb.cpp
 *    @brief Publish in the logger the error messages received from the Bravo UDP
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      22-Jun-2025
 *    Modification 25-Nov-2025
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */

#include "bravo_compliance/bravo_cpp/bravo_io_v2/bravo_udp_v2.h"

namespace bravo_control{     
    /**
     * @brief Fulfill information from the joint feedback packet (it can be position, velocity or current)
     *  @param packet The packet received from the Bravo UDP
     *  @param joint_feedback The feedback object to be filled with the joint data
     *  TODO: UNKNOWN
     */
    template <typename T> 
        bool bravo_udp<T>::processJointFdbPacket(Packet& packet, feedback<T>& joint_feedback) {
            float data = decodeFloat_unite(packet.data);
            if (arm_model == bravo_control::ArmModel::bravo5) {
                switch (packet.deviceID) {
                    case 0x01: joint_feedback.data[4] = data; joint_feedback.joints_received[4] = true; break;
                    case 0x02: joint_feedback.data[3] = data; joint_feedback.joints_received[3] = true; break;
                    case 0x03: joint_feedback.data[2] = data; joint_feedback.joints_received[2] = true; break;
                    case 0x04: joint_feedback.data[1] = data; joint_feedback.joints_received[1] = true; break;
                    case 0x05: joint_feedback.data[0] = data; joint_feedback.joints_received[0] = true; break;
                    default:
                        //std::cout <<"[bravo5_UDP]:❗Packet DeviceId received which could not be classified"<<std::endl;
                        return false;
                }
            }
            else if (arm_model == bravo_control::ArmModel::bravo7) {    
                switch (packet.deviceID) {
                    case 0x01: joint_feedback.data[6] = data; joint_feedback.joints_received[6] = true; break;
                    case 0x02: joint_feedback.data[5] = data; joint_feedback.joints_received[5] = true; break;
                    case 0x03: joint_feedback.data[4] = data; joint_feedback.joints_received[4] = true; break;
                    case 0x04: joint_feedback.data[3] = data; joint_feedback.joints_received[3] = true; break;
                    case 0x05: joint_feedback.data[2] = data; joint_feedback.joints_received[2] = true; break;
                    case 0x06: joint_feedback.data[1] = data; joint_feedback.joints_received[1] = true; break;
                    case 0x07: joint_feedback.data[0] = data; joint_feedback.joints_received[0] = true; break;
                    default:
                        //std::cout <<"[bravo7_UDP]:❗Packet DeviceId received which could not be classified" <<std::endl;
                        return false;
                }
            }
            else{
                std::cerr << "[bravo_UDP]:❌ FATAL Unknown arm model configured." << std::endl;
                std::exit(EXIT_FAILURE);
            }

            // Check if all joints received
            if (std::all_of(joint_feedback.joints_received.begin(), joint_feedback.joints_received.end() - 1, [](bool received){ return received; })) {
                joint_feedback.received = true;
                joint_feedback.last_msg = std::chrono::high_resolution_clock::now();
                std::fill(joint_feedback.joints_received.begin(), joint_feedback.joints_received.end(), false);
                return true; 
            }
            else{
                return false;
            }
        }

    template <typename T> 
        void bravo_udp<T>::processControlModePacket(Packet& packet) {
            switch (packet.data[0]) {
                case 0x00: control_mode_state = control_mode_states::standby;             break;
                case 0x01: control_mode_state = control_mode_states::disable;             break;
                case 0x02: control_mode_state = control_mode_states::joint_position_mode; break; 
                case 0x03: control_mode_state = control_mode_states::joint_velocity_mode; break; //! this can also apply to local twist ee
                case 0x04: control_mode_state = control_mode_states::joint_current_mode;  break;
                case 0x0B: control_mode_state = control_mode_states::joint_torque_mode;   break;
                default:
                    std::cout <<"[bravo_UDP]:❗ Unknown control mode "<< packet.data[0] <<std::endl;
                    break;
            }
        }
}


