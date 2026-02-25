#include "bravo7_version_2/bravo_cpp/force_estimator/estimator.hpp"


Estimator::Estimator(const std::string& urdf_file)
{
    model_handler_ = std::make_shared<ModelHandler>(urdf_file);
    control_axis_ = model_handler_ -> get_nq();

    previous_q_.resize(control_axis_);
    previous_q_.setZero();

    previous_dq_.resize(control_axis_);
    previous_dq_.setZero();

    previous_ddq_.resize(control_axis_);
    previous_ddq_.setZero();

    previous_current_.resize(control_axis_);
    previous_current_.setZero();

    previous_estimated_current_.resize(control_axis_);
    previous_estimated_current_.setZero();

    std::string share_directory = ament_index_cpp::get_package_share_directory("bravo7_version_2");
    std::string yaml_file = share_directory + "/config/bravo7_dynamic_params.yaml";
    std::string yaml_file2 = "/home/carlos/reach_bravo_7_ws/src/reach-bravo-7/bravo7_version_2/config/bravo7_dynamic_params.yaml"; //!TO BE CHANGED

    YAML::Node parameters = YAML::LoadFile(yaml_file2);
    continuous_ = parameters["continuous"].as<std::vector<bool>>();
    Kt_ = parameters["Kt"].as<std::vector<double>>();
    Gr_ = parameters["Gr"].as<std::vector<double>>();
    Fc_ = parameters["Fc"].as<std::vector<double>>();
    Fs_ = parameters["Fs"].as<std::vector<double>>();
    Fd_ = parameters["Fd"].as<std::vector<double>>();
    Tq_ = parameters["Tq"].as<double>();
    Tv_ = parameters["Tv"].as<double>();
    Ta_ = parameters["Ta"].as<double>();
    Tc_ = parameters["Tc"].as<double>();

    previous_time_ = std::chrono::high_resolution_clock::now();
}

void Estimator::initialize(std::optional<Eigen::VectorXd> initial_q,
                           std::optional<Eigen::VectorXd> initial_dq,
                           std::optional<Eigen::VectorXd> initial_ddq,
                           std::optional<Eigen::VectorXd> initial_current)
{
    if (initial_q.has_value())
    {
        previous_q_ = initial_q.value();
    }
    else
    {
        previous_q_.setZero();
    }
    
    if (initial_dq.has_value())
    {
        previous_dq_ = initial_dq.value();
    }
    else
    {
        previous_dq_.setZero();
    }

    if (initial_ddq.has_value())
    {
        previous_ddq_ = initial_ddq.value();
    }
    else
    {
        previous_ddq_.setZero();
    }

    if (initial_current.has_value())
    {
        previous_current_ = initial_current.value();
    }
    else
    {
        previous_current_.setZero();
    }

    previous_time_ = std::chrono::high_resolution_clock::now();
}

Eigen::VectorXd Estimator::get_position()
{
    return previous_q_;
}

Eigen::VectorXd Estimator::get_velocity()
{
    return previous_dq_;
}

Eigen::VectorXd Estimator::get_acceleration()
{
    return previous_ddq_;
}

Eigen::VectorXd Estimator::get_current()
{
    return previous_current_;
}

Eigen::VectorXd Estimator::get_estimated_current()
{
    return previous_estimated_current_;
}

Eigen::VectorXd Estimator::get_torque()
{
    Eigen::VectorXd torque(control_axis_);
    Eigen::VectorXd current = get_current();
    for (int i = 0; i < control_axis_; i++)
    {
        torque(i) = current(i) / 1000.0 * Kt_[i] * Gr_[i];
    }
    return torque;
}

Eigen::VectorXd Estimator::get_estimated_torque()
{
    Eigen::VectorXd estimated_torque(control_axis_);
    Eigen::VectorXd estimated_current = get_estimated_current();
    for (int i = 0; i < control_axis_; i++)
    {
        estimated_torque(i) = estimated_current(i) / 1000.0 * Kt_[i] * Gr_[i];
    }
    return estimated_torque;
}

geometry_msgs::msg::Wrench Estimator::get_frame_force_torque(const std::string& frame_name)
{
    Eigen::VectorXd current_q = get_position();
    Eigen::VectorXd current_torque = get_torque();
    Eigen::MatrixXd J = model_handler_ -> get_geometric_jacobian(current_q, frame_name, pinocchio::LOCAL);
    Eigen::MatrixXd dls_JT = model_handler_ -> get_dls_jacobian(J.transpose(), 1e-2);
    Eigen::VectorXd FT = dls_JT * current_torque;
    geometry_msgs::msg::Wrench wrench;
    wrench.force.x = FT(0);
    wrench.force.y = FT(1);
    wrench.force.z = FT(2);
    wrench.torque.x = FT(3);
    wrench.torque.y = FT(4);
    wrench.torque.z = FT(5);
    return wrench;
}

geometry_msgs::msg::Wrench Estimator::get_estimated_frame_force_torque(const std::string& frame_name)
{
    Eigen::VectorXd current_q = get_position();
    Eigen::VectorXd current_estimated_torque = get_estimated_torque();
    Eigen::MatrixXd J = model_handler_ -> get_geometric_jacobian(current_q, frame_name, pinocchio::LOCAL);
    Eigen::MatrixXd dls_JT = model_handler_ -> get_dls_jacobian(J.transpose(), 1e-2);
    Eigen::VectorXd FT = dls_JT * current_estimated_torque;
    geometry_msgs::msg::Wrench wrench;
    wrench.force.x = FT(0);
    wrench.force.y = FT(1);
    wrench.force.z = FT(2);
    wrench.torque.x = FT(3);
    wrench.torque.y = FT(4);
    wrench.torque.z = FT(5);
    return wrench;
}

void Estimator::update(const Eigen::VectorXd& q, const Eigen::VectorXd& current)
{
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration<double>(now - previous_time_).count();

    Eigen::VectorXd smoothed_q(control_axis_);
    Eigen::VectorXd current_v(control_axis_);
    Eigen::VectorXd smoothed_v(control_axis_);
    Eigen::VectorXd current_a(control_axis_);
    Eigen::VectorXd smoothed_a(control_axis_);
    Eigen::VectorXd smoothed_current(control_axis_);

    for (int i = 0; i < control_axis_; i++)
    {
        if (continuous_[i])
        {
            Eigen::Vector2d current_circle = point_to_circle(q(i));
            Eigen::Vector2d previous_circle = point_to_circle(previous_q_(i));
            Eigen::Vector2d smoothed_circle;
            smoothed_circle << smoothen(current_circle(0), previous_circle(0), Tq_), smoothen(current_circle(1), previous_circle(1), Tq_);
            smoothed_q(i) = circle_to_point(smoothed_circle);
            smoothed_q(i) = saturate_within_one_circle(smoothed_q(i));

            Eigen::Vector2d current_circle_velocity = 1 / dt * (smoothed_circle - previous_circle);
            Eigen::Vector2d previous_circle_velocity = point_velocity_to_circle_velocity(previous_q_(i), previous_dq_(i));
            Eigen::Vector2d smoothed_circle_velocity;
            smoothed_circle_velocity << smoothen(current_circle_velocity(0), previous_circle_velocity(0), Tv_), smoothen(current_circle_velocity(1), previous_circle_velocity(1), Tv_);
            smoothed_v(i) = circle_velocity_to_point_velocity(smoothed_circle_velocity, smoothed_q(i));

            Eigen::Vector2d current_circle_acceleration = 1 / dt * (smoothed_circle_velocity - previous_circle_velocity);
            Eigen::Vector2d previous_circle_acceleration = point_acceleration_to_circle_acceleration(previous_q_(i), previous_dq_(i), previous_ddq_(i));
            Eigen::Vector2d smoothed_circle_acceleration;
            smoothed_circle_acceleration << smoothen(current_circle_acceleration(0), previous_circle_acceleration(0), Ta_), smoothen(current_circle_acceleration(1), previous_circle_acceleration(1), Ta_);
            smoothed_a(i) = circle_acceleration_to_point_acceleration(smoothed_circle_acceleration, smoothed_q(i));
        }
        else
        {
            smoothed_q(i) = smoothen(q(i), previous_q_(i), Tq_);
            current_v(i) = 1 / dt * (smoothed_q(i) - previous_q_(i));
            smoothed_v(i) = smoothen(current_v(i), previous_dq_(i), Tv_);
            current_a(i) = 1 / dt * (smoothed_v(i) - previous_dq_(i));
            smoothed_a(i) = smoothen(current_a(i), previous_ddq_(i), Ta_);
        }
        smoothed_current(i) = smoothen(current(i), previous_current_(i), Tc_);
    }

    previous_q_ = smoothed_q;
    previous_dq_ = smoothed_v;
    previous_ddq_ = smoothed_a;
    previous_current_ = smoothed_current;
    previous_time_ = now;

    Eigen::VectorXd frictionless_tau = model_handler_ -> get_frictionless_tau(previous_q_, previous_dq_, previous_ddq_);
    Eigen::VectorXd G = model_handler_ -> get_CG(previous_q_, Eigen::VectorXd::Zero(control_axis_));
    Eigen::VectorXd tau(control_axis_);
    Eigen::VectorXd I(control_axis_);

    for (int i = 0; i < control_axis_; i++)
    {
        tau(i) = frictionless_tau(i) + Fc_[i] * previous_dq_.array().sign()(i) + Fd_[i] * previous_dq_(i);
        double G_I = G(i) / Gr_[i] / Kt_[i] * 1000.0;

        if ((std::abs(previous_dq_(i)) < 0.01) && (std::abs(previous_current_(i) - G_I) < Fs_[i]))
        {
            I(i) = previous_current_(i);
        }
        else if ((std::abs(previous_dq_(i)) < 0.01) && (std::abs(previous_current_(i) - G_I) > Fs_[i]))
        {
            if ((previous_current_(i) - G_I) > 0)
            {
                I(i) = Fs_[i] + G_I;
            }
            else
            {
                I(i) = -Fs_[i] + G_I;
            }
        }
        else
        {
            I(i) = 1000.00 * 1 / Gr_[i] * 1 / Kt_[i] * tau(i);
        }
    }

    previous_estimated_current_ = I;
    
}

double Estimator::smoothen(const double current, const double previous, const double alpha)
{
    return alpha * current + (1 - alpha) * previous; 
}

Eigen::Vector2d Estimator::point_to_circle(const double point)
{
    Eigen::Vector2d circle;
    circle << std::sin(point), std::cos(point);
    return circle;
}

Eigen::Vector2d Estimator::point_velocity_to_circle_velocity(const double point, const double point_velocity)
{
    Eigen::Vector2d circle_velocity;
    circle_velocity << point_velocity * std::cos(point), -point_velocity * std::sin(point);
    return circle_velocity;    
}

Eigen::Vector2d Estimator::point_acceleration_to_circle_acceleration(const double point, const double point_velocity, const double point_acceleration)
{
    Eigen::Vector2d circle_acceleration;
    circle_acceleration << point_acceleration * std::cos(point) - point_velocity * point_velocity * std::sin(point), -point_acceleration * std::sin(point) - point_velocity * point_velocity * std::cos(point);
    return circle_acceleration;    
}

double Estimator::circle_to_point(const Eigen::Vector2d circle)
{
    double point = std::atan2(circle(0), circle(1));
    return point;    
}

double Estimator::circle_velocity_to_point_velocity(const Eigen::Vector2d circle_velocity, const double point)
{
    Eigen::Vector2d circle = point_to_circle(point);
    double point_velocity = circle(1) * circle_velocity(0) - circle(0) * circle_velocity(1);
    return point_velocity;    
}

double Estimator::circle_acceleration_to_point_acceleration(const Eigen::Vector2d circle_acceleration, const double point)
{
    Eigen::Vector2d circle = point_to_circle(point);
    double point_acceleration = circle(1) * circle_acceleration(0) - circle(0) * circle_acceleration(1);
    return point_acceleration;    
}

double Estimator::saturate_within_one_circle(const double q)
{
    double circle = q;
    while (circle < 0)
    {
        circle += 2 * M_PI;
    }
    while (circle > 2 * M_PI)
    {
        circle -= 2 * M_PI;
    }
    return circle;
}