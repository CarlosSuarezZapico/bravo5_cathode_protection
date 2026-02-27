/**
 *    @file  utils.h
 *    @brief methods, structures used....
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  9-Jan-2024
 *    Modification 9-Jan-2024
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#ifndef _UTILS_
#define _UTILS_
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <chrono>
#include <vector>


namespace bravo_utils
{
    template <typename T> 
        struct recv_feedback{    
            T data;                  // Main information of the variable (joint pos, joint torques....)  
            bool received = false;  // Structure used to know if topic received and
            std::chrono::high_resolution_clock::time_point last_msg;  // elapsed time since last msg received 
        }; 

    template <typename T> 
        struct cmd_data{    
            T data;                  // Main information of the variable (joint pos, joint torques....)  
            std::chrono::high_resolution_clock::time_point last_update;  // elapsed time since last msg received 
        }; 

    template <typename T> T VAL_SAT( T value, T maxValue, T minValue) {
        // Check for invalid min/max range
        if (minValue > maxValue) {
            assert(minValue <= maxValue && "VAL_SAT error: minValue > maxValue!");
            // Optionally swap in release mode
            T temp = minValue;
            minValue = maxValue;
            maxValue = temp;
        }
        //! SAFETY CRITICAL STEP: NANs ARE VERY DANGEROUS
        // Check for NaN (Not a Number) 
        if (std::isnan(value)) {
            // Log, throw, or fallback to safe value
            // For robots, returning 0 or minValue might be safest
            return static_cast<T>(0);  // or choose a safe default like minValue
        }
            if (value >= maxValue){
                return maxValue;
            }
            else if (value <= minValue){
                return minValue;
            }
            else{
                return value;
            }
    }

    void GetEulerAngles(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll);

    template <typename T, typename Total, std::size_t N>
        class Moving_Average{
            public:
                Moving_Average& operator()(T sample)
                {
                    total_ += sample;
                    if (num_samples_ < N)
                        samples_[num_samples_++] = sample;
                    else
                    {
                        T& oldest = samples_[num_samples_++ % N];
                        total_ -= oldest;
                        oldest = sample;
                    }
                    return *this;
                }
                operator double() const { return total_ / std::min(num_samples_, N); }

            private:
                T samples_[N];
                size_t num_samples_{0};
                Total total_{0};
    };

}

#endif 