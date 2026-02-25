#ifndef ESTIMATOR_HPP
#define ESTIMATOR_HPP

#include "bravo7_version_2/bravo_cpp/force_estimator/model_handler.hpp"

#include "yaml-cpp/yaml.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
//#include "ament_index_cpp/get_package_share_directory.hpp"

#include <geometry_msgs/msg/wrench.hpp>

#include <optional>

class Estimator
{
    public:
        Estimator(const std::string& urdf_file);
        void initialize(std::optional<Eigen::VectorXd> initial_q = std::nullopt,
                        std::optional<Eigen::VectorXd> initial_dq = std::nullopt,
                        std::optional<Eigen::VectorXd> initial_ddq = std::nullopt,
                        std::optional<Eigen::VectorXd> initial_current = std::nullopt);
        Eigen::VectorXd get_position();
        Eigen::VectorXd get_velocity();
        Eigen::VectorXd get_acceleration();
        Eigen::VectorXd get_current();
        Eigen::VectorXd get_estimated_current();
        Eigen::VectorXd get_torque();
        Eigen::VectorXd get_estimated_torque();
        geometry_msgs::msg::Wrench get_frame_force_torque(const std::string& frame_name);
        geometry_msgs::msg::Wrench get_estimated_frame_force_torque(const std::string& frame_name);
        void update(const Eigen::VectorXd& q, const Eigen::VectorXd& current);
    private:
        double smoothen(const double current, const double previous, const double alpha);
        Eigen::Vector2d point_to_circle(const double point);
        Eigen::Vector2d point_velocity_to_circle_velocity(const double point, const double point_velocity);
        Eigen::Vector2d point_acceleration_to_circle_acceleration(const double point, const double point_velocity, const double point_acceleration);
        double circle_to_point(const Eigen::Vector2d circle);
        double circle_velocity_to_point_velocity(const Eigen::Vector2d circle_velocity, const double point);
        double circle_acceleration_to_point_acceleration(const Eigen::Vector2d circle_acceleration, const double point);
        double saturate_within_one_circle(const double q);
        
        std::shared_ptr<ModelHandler> model_handler_;
        int control_axis_;
        std::vector<double> Kt_, Gr_, Fc_, Fs_, Fd_;
        std::vector<bool> continuous_;
        double Tq_, Tv_, Ta_, Tc_;
        Eigen::VectorXd previous_q_, previous_dq_, previous_ddq_, previous_current_, previous_estimated_current_;
        std::chrono::high_resolution_clock::time_point previous_time_;
};

#endif // ESTIMATOR_HPP