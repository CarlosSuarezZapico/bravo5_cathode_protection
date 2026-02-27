/**
 *    @file  bravo_diagnosis.cpp
 *    @brief Publish in the logger the error messages received from the Bravo7 UDP
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  22-Jun-2025
 *    Modification 22-Jun-2025
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */

#include "bravo_cathode_protection/bravo_cpp/bravo_io_v2/bravo_udp_v2.h"

namespace bravo_control{ 
    
    template <typename T> 
        void bravo_udp<T>::processDiagnosticByte(uint8_t byte, int byteIndex) {
            static const std::unordered_map<int, std::unordered_map<uint8_t, std::string>> diagnosticMessages = {
                {0, {
                    {0x80, "FLASH FAILED READ"},
                    {0x40, "HARDWARE OVER HUMIDITY: The humidity levels detected are over acceptable factory humidity levels"},
                    {0x20, "HARDWARE OVER TEMPERATURE: Joint temperature is over acceptable temperature levels."},
                    {0x10, "COMMS SERIAL ERROR: Serial communication errors detected. This may be due to noise, or half duplex communication collisions"},
                    {0x08, "COMMS CRC ERROR: Communication decoding errors detected. This may be due to noise, or half duplex communication collisions"},
                    {0x04, "MOTOR DRIVER FAULT: The motor driver is drawing too much current, or the voltage supply is too low."},
                    {0x02, "ENCODER POSITION ERROR: Errors found in the joints position encoder. Absolute position may be incorrect."},
                    {0x01, "ENCODER NOT DETECTED: Joints position encoder is not detected."},
                }},
                {1, {
                    {0x80, "DEVICE AXIS CONFLICT: Detected an incorrect setup in a devices kinematic chain. Device ids must be in the correct order."},
                    {0x40, "MOTOR NOT CONNECTED: Detected that the motor is not connected."},
                    {0x20, "MOTOR OVER CURRENT: The motor is drawing too much current."},
                    {0x10, "INNER ENCODER POSITION ERROR: Errors found in the inner encoder. Commutation of the Joint may be affected."},
                    {0x08, "DEVICE ID CONFLICT: Detected multiple devices with the same device id."},
                    {0x04, "HARDWARE OVER PRESSURE: Pressure levels detected are over the factory levels."},
                    {0x02, "MOTOR DRIVER OVER CURRENT AND UNDER VOLTAGE: Motor driver is drawing too much current, or the voltage supply is too low"},
                    {0x01, "MOTOR DRIVER OVER TEMPERATURE: The motor driver temperature is too high."},
                }},
                {2, {
                    {0x10, "JAW ZERO REQUIRED: The jaws must be calibrated."},
                    {0x08, "JOINT SERVICE DUE: One or more of the joints have passed the factory-set service interval."},
                    {0x04, "READ PROTECTION ENABLED: Updates can no longer be performed. Contact Support."},
                    {0x02, "ENCODER FAULT: The joint may have developed backlash."},
                }},
                {3, {
                    {0x80, "ENCODER POSITION INVALID [Factory Only]"},
                    {0x10, "LOW SUPPLY VOLTAGE: Check the power supply is providing sufficient current and/or voltage."},
                    {0x08, "INVALID FIRMWARE: Invalid 708 or 703 firmware is loaded on the bravo for the compute module to use."},
                    {0x02, "CANBUS ERROR: Errors found whilst communicating on the CANBUS."},
                    {0x01, "POSITION REPORT NOT RECEIVED: The compute module is unable to get information from the joints."},
                }},
            };

            auto itByteMap = diagnosticMessages.find(byteIndex);
            if (itByteMap != diagnosticMessages.end()) {
                for (const auto& [bitMask, message] : itByteMap->second) {
                    if (byte & bitMask) {
                        BRAVO_LOG_INFO(logger_, "[bravo_UDP]: ", message);
                    }
                }
            }
        }

    template <typename T> 
        void bravo_udp<T>::bravoDiagnoseWarnings(Packet& packet, bool diagnosisEnabled) {
            if (!diagnosisEnabled) return;            
            if (packet.packetID == 0x68) {
                for (int byteIndex = 0; byteIndex < 4; ++byteIndex) {
                    processDiagnosticByte(packet.data[byteIndex], byteIndex);
                }
            }
            
        }
}

