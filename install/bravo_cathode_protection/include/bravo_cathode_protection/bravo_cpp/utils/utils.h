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
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <chrono>
#include <vector>
#include <algorithm>
#include <concepts>
#include <limits>
#include <string>
#include <stdexcept>
#include <yaml-cpp/yaml.h>


namespace bravo_utils
{
    template<typename T>
        concept FloatingPID =  std::same_as<T, float> || std::same_as<T, double>;

    template<FloatingPID T>
        class PID {
            public:
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
                    if (dt <= std::numeric_limits<T>::epsilon()) {
                        dt = static_cast<T>(1e-6);
                    }
                    integral_ += error * dt;
                    integral_ = std::clamp(integral_, min_integral_, max_integral_);
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

    struct FrictionConfig
    {
        Eigen::Matrix<double,4,6> friction_mat = Eigen::Matrix<double,4,6>::Zero();
        double friction_saturation = 30.0;
    };

    static FrictionConfig load_friction_config_yaml(const std::string& path)
    {
        if (!std::ifstream(path).good()) {
            throw std::runtime_error("Could not open friction config file: " + path);
        }
        const YAML::Node node = YAML::LoadFile(path);
        const YAML::Node friction = node["friction"];
        if (!friction || !friction.IsMap()) {
            throw std::runtime_error("YAML key 'friction' must be a map");
        }
        FrictionConfig cfg;
        for (int joint = 0; joint < 4; ++joint) {
            const std::string joint_key = "joint" + std::to_string(joint + 1);
            const YAML::Node joint_node = friction[joint_key];
            if (!joint_node || !joint_node.IsMap()) {
                throw std::runtime_error("YAML key 'friction." + joint_key + "' must be a map");
            }

            cfg.friction_mat(joint, 0) = joint_node["g0"].as<double>();
            cfg.friction_mat(joint, 1) = joint_node["g1"].as<double>();
            cfg.friction_mat(joint, 2) = joint_node["g2"].as<double>();
            cfg.friction_mat(joint, 3) = joint_node["g3"].as<double>();
            cfg.friction_mat(joint, 4) = joint_node["g4"].as<double>();
            cfg.friction_mat(joint, 5) = joint_node["g5"].as<double>();
            cfg.friction_saturation = joint_node["s"].as<double>();
        }
        return cfg;
    }

    struct RuntimeConfig
    {
        std::string ip_address = "10.43.0.146";
        int udp_port = 6789;
        Eigen::Vector4d home = (Eigen::Vector4d() << 3.14, 2.857, 1.362, 0.0).finished();
        Eigen::Vector3d gravity_vector = (Eigen::Vector3d() << 0.0, 0.0, -9.81).finished();
        double max_current_mA = 2000.0;
        double max_current_mA_go_home = 1000.0;
        double max_manipulability = 6.0;
        double max_ratio_force_ellipsoid = 0.3;
        double max_speed_teleop = 0.25;
        double loop_frequency = 250.0;
        std::string tool_link = "contact_point";
    };

    static Eigen::Vector4d yaml_vec4(const YAML::Node& node, const std::string& key)
    {
        const YAML::Node value = node[key];
        if (!value || !value.IsSequence() || value.size() != 4) {
            throw std::runtime_error("YAML key '" + key + "' must be a sequence of 4 numbers");
        }
        return Eigen::Vector4d(
            value[0].as<double>(),
            value[1].as<double>(),
            value[2].as<double>(),
            value[3].as<double>());
    }

    static Eigen::Vector3d yaml_vec3(const YAML::Node& node, const std::string& key)
    {
        const YAML::Node value = node[key];
        if (!value || !value.IsSequence() || value.size() != 3) {
            throw std::runtime_error("YAML key '" + key + "' must be a sequence of 3 numbers");
        }
        return Eigen::Vector3d(
            value[0].as<double>(),
            value[1].as<double>(),
            value[2].as<double>());
    }

    static RuntimeConfig load_runtime_config_yaml(const std::string& path)
    {
        if (!std::ifstream(path).good()) {
            throw std::runtime_error("Could not open runtime config file: " + path);
        }
        const YAML::Node node = YAML::LoadFile(path);
        RuntimeConfig cfg;
        cfg.ip_address = node["IP"].as<std::string>();
        cfg.udp_port = node["port"].as<int>();
        cfg.home = yaml_vec4(node, "HOME");
        cfg.gravity_vector = yaml_vec3(node, "GRAVITY_VECTOR");
        cfg.max_current_mA = node["MAX_CURRENT_mA"].as<double>();
        cfg.max_current_mA_go_home = node["MAX_CURRENT_mA_GO_HOME"].as<double>();
        cfg.max_manipulability = node["MAX_MANIPULABILITY"].as<double>();
        cfg.max_ratio_force_ellipsoid = node["MAX_RATIO_FORCE_ELLIPSOID"].as<double>();
        cfg.max_speed_teleop = node["MAX_SPEED_TELEOP"].as<double>();
        cfg.loop_frequency = node["LOOP_FREQUENCY"].as<double>();
        cfg.tool_link = node["tool_link"].as<std::string>();
        return cfg;
    }
}

#endif 
