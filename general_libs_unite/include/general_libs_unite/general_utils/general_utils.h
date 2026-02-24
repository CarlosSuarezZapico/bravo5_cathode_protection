/**
 *    @file  general_utils.h
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
#ifndef _GENERAL_UTILS_
#define _GENERAL_UTILS_
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <concepts>

using namespace std;
using namespace Eigen;

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

template<typename T>
    concept FloatingPID =  std::same_as<T, float> ||   std::same_as<T, double>;

template<FloatingPID T>
    class PID {
        public:
            // ✔ Mandatory constructor (only 5 required parameters)
            PID(T kp, T ki, T kd, T min_out, T max_out)
            : kp_(kp),
                ki_(ki),
                kd_(kd),
                min_out_(min_out),
                max_out_(max_out),
                min_integral_(std::numeric_limits<T>::lowest()),
                max_integral_(std::numeric_limits<T>::max()),
                integral_(T(0)),
                prev_error_(T(0)),
                first_run_(true)
            {}

            // ✔ Optional constructor with integral limits
            PID(T kp, T ki, T kd, T min_out, T max_out, T min_integral, T max_integral)
            : kp_(kp),
                ki_(ki),
                kd_(kd),
                min_out_(min_out),
                max_out_(max_out),
                min_integral_(min_integral),
                max_integral_(max_integral),
                integral_(T(0)),
                prev_error_(T(0)),
                first_run_(true)
            {}

            T compute(T error, T dt)
            {
                if (dt <= std::numeric_limits<T>::epsilon())
                dt = static_cast<T>(1e-6);
                // Integral term with clamping
                integral_ += error * dt;
                integral_ = std::clamp(integral_, min_integral_, max_integral_);
                // Derivative term (avoid spike on first call)
                T derivative = T(0);
                if (!first_run_) {
                    derivative = (error - prev_error_) / dt;
                } else {
                    first_run_ = false;
                }
                prev_error_ = error;
                T output = kp_ * error + ki_ * integral_ + kd_ * derivative;
                return std::clamp(output, min_out_, max_out_);
            }

            void reset()
            {
                integral_ = T(0);
                prev_error_ = T(0);
                first_run_ = true;
            }

            void setGains(T kp, T ki, T kd)
            {
                kp_ = kp;
                ki_ = ki;
                kd_ = kd;
            }

            void setOutputLimits(T min_out, T max_out)
            {
                min_out_ = min_out;
                max_out_ = max_out;
            }

            void setIntegralLimits(T min_integral, T max_integral)
            {
                min_integral_ = min_integral;
                max_integral_ = max_integral;
            }

        private:
            T kp_, ki_, kd_;
            T min_out_, max_out_;
            T min_integral_, max_integral_;
            T integral_;
            T prev_error_;
            bool first_run_;
};


namespace general_utils
{
    template <typename T>
        T VAL_SAT( T value, T maxValue, T minValue) {
        // Check for invalid min/max range
        if (minValue > maxValue) {
            assert(minValue <= maxValue && "VAL_SAT error: minValue > maxValue!");
            // Optionally swap in release mode
            T temp = minValue;
            minValue = maxValue;
            maxValue = temp;
        }
        //! SAFETY CRITICAL STEP: NANs ARE VERY DANGEROUS
        // Check for NaN 
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

    void GetEulerAngles(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll){
        const double w2 = q.w()*q.w();
        const double x2 = q.x()*q.x();
        const double y2 = q.y()*q.y();
        const double z2 = q.z()*q.z();
        const double unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
        const double abcd = q.w()*q.x() + q.y()*q.z();
        const double eps = 1e-7;    // TODO: pick from your math lib instead of hardcoding.
        const double pi = 3.14159265358979323846;   // TODO: pick from your math lib instead of hardcoding.
        if (abcd > (0.5-eps)*unitLength)
        {
            yaw = 2 * atan2(q.y(), q.w());
            pitch = pi;
            roll = 0;
        }
        else if (abcd < (-0.5+eps)*unitLength)
        {
            yaw = -2 * atan2(q.y(), q.w());
            pitch = -pi;
            roll = 0;
        }
        else
        {
            const double adbc = q.w()*q.z() - q.x()*q.y();
            const double acbd = q.w()*q.y() - q.x()*q.z();
            yaw = atan2(2*adbc, 1 - 2*(z2+x2));
            pitch = asin(2*abcd/unitLength);
            roll = atan2(2*acbd, 1 - 2*(y2+x2));
        }
    }

    template <typename T, typename Total, size_t N>
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