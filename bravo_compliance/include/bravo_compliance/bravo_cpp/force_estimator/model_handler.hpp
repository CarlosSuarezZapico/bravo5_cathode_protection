#ifndef MODEL_HANDLER_HPP
#define MODEL_HANDLER_HPP

#include <string>
#include <cmath>

#include "Eigen/Dense"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/regressor.hpp"
#include "pinocchio/spatial/se3.hpp"

struct ModelOptions
{
    Eigen::VectorXd velocity_limit;
    Eigen::VectorXd upper_joint_limit;
    Eigen::VectorXd lower_joint_limit;
    Eigen::Matrix<bool, -1, 1> continuous;

    ModelOptions(const int nq = 1)
    {
        velocity_limit = Eigen::VectorXd::Ones(nq);
        upper_joint_limit = M_PI * Eigen::VectorXd::Ones(nq);
        lower_joint_limit = M_PI * Eigen::VectorXd::Ones(nq);
        continuous = Eigen::Matrix<bool, -1, 1>::Zero(nq);
    }
};


class ModelHandler
{
    public:
        ModelHandler(const std::string& urdf_file);
        void set_model_options(const ModelOptions& model_options);
        void set_model_gravity(const Eigen::Vector3d& gravity);
        void show_model_options();
        void show_model_joints();
        Eigen::VectorXd get_joint_error(const Eigen::VectorXd& current_q, const Eigen::VectorXd& desired_q);
        Eigen::VectorXd scale_joint_velocity(const Eigen::VectorXd& dq);
        Eigen::VectorXd integrate_configuration(const Eigen::VectorXd& current_q, const Eigen::VectorXd& velocity);
        int get_nq();
        int get_nv();
        int get_joint_id(const std::string& joint_name);
        std::string get_joint_name(const int index);
        Eigen::MatrixXd get_geometric_jacobian(const Eigen::VectorXd& current_q, const std::string& frame_name, const pinocchio::ReferenceFrame reference_frame);
        Eigen::MatrixXd get_dls_jacobian(const Eigen::MatrixXd& J, const double lambda);
        pinocchio::SE3 get_frame_pose(const Eigen::VectorXd& current_q, const std::string& frame_name);
        double get_condition_number(const Eigen::VectorXd& current_q, const std::string& frame_name, const pinocchio::ReferenceFrame reference_frame);
        Eigen::MatrixXd get_M(const Eigen::VectorXd& current_q);
        Eigen::VectorXd get_CG(const Eigen::VectorXd& current_q, const Eigen::VectorXd& current_dq);
        Eigen::VectorXd get_frictionless_tau(const Eigen::VectorXd& current_q, const Eigen::VectorXd& current_dq, const Eigen::VectorXd& current_ddq);

    private:  
        pinocchio::Model model_;
        pinocchio::Data data_;

        std::shared_ptr<ModelOptions> model_options_;
};

#endif // MODEL_HANDLER_HPP