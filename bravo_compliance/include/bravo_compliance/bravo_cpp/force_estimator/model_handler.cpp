#include "bravo7_version_2/bravo_cpp/force_estimator/model_handler.hpp"

ModelHandler::ModelHandler(const std::string& urdf_file)
{
    pinocchio::urdf::buildModel(urdf_file, model_);
    data_ = pinocchio::Data(model_);

    model_options_ = std::make_shared<ModelOptions>(model_.nq);
}

void ModelHandler::set_model_options(const ModelOptions& model_options)
{
    if (model_options.continuous.size() != model_.nq || model_options.velocity_limit.size() != model_.nq ||
        model_options.upper_joint_limit.size() != model_.nq || model_options.lower_joint_limit.size() != model_.nq)
    {
        std::cerr << "[set_model_options]: Invalid model options: vector size. Failed to set the options." << std::endl;
        return;
    }

    if ((model_options.lower_joint_limit.array() > model_options.upper_joint_limit.array()).all())
    {
        std::cerr << "[set_model_options]: Invalid model options: lower joint limit is higher than upper joint limit. Failed to set the options." << std::endl;
        return;
    }

    if ((model_options.velocity_limit.array() <= 0).all())
    {
        std::cerr << "[set_model_options]: Invalid model options: non-positive max velocity. Failed to set the options." << std::endl;
        return;
    }

    model_options_ = std::make_shared<ModelOptions>(model_options);
    std::cout << "[set_model_options]: Model options set successfully." << std::endl;
}

void ModelHandler::set_model_gravity(const Eigen::Vector3d& gravity)
{
    model_.gravity.linear() = gravity;
}

void ModelHandler::show_model_options()
{
    std::cout << "Current Model Options:" << std::endl;
    for (int i = 0; i < model_.nq; i++)
    {
        std::cout << i + 1 << ". [" << model_.names[i + 1] << "]"
                  << ", velocity limit: ±" << model_options_ -> velocity_limit(i)
                  << ", joint limit: (" << model_options_ -> lower_joint_limit(i)
                  << ", " << model_options_ -> upper_joint_limit(i) << ")"
                  << ", continuous: " << model_options_ -> continuous(i)
                  << std::endl;
    }
}

void ModelHandler::show_model_joints()
{
    std::cout << "Model has " << model_.joints.size() - 1 << "joints." << std::endl;

    for (int j = 1; j < model_.njoints; ++j)
    {
        std::cout << j << " [" << model_.names[j] << "]"
                  << " nq=" << model_.joints[j].nq()
                  << " nv=" << model_.joints[j].nv()
                  << " idx_qs=" << model_.idx_qs[j]
                  << " type: " << model_.joints[j].shortname()
                  << std::endl;
    }
}

Eigen::VectorXd ModelHandler::get_joint_error(const Eigen::VectorXd& current_q, const Eigen::VectorXd& desired_q)
{
    if (current_q.size() != model_.nq)
    {
        std::cerr << "[get_joint_error]: Invalid vector 'current_q' size." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (desired_q.size() != model_.nq)
    {
        std::cerr << "[get_joint_error]: Invalid vector 'desired_q' size." << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
    if ((desired_q.array() > model_options_ -> upper_joint_limit.array()).all() ||
        (desired_q.array() < model_options_ -> lower_joint_limit.array()).all())
    {
        std::cerr << "[get_joint_error]: 'desired_q' is out of joint limits." << std::endl;
        return Eigen::VectorXd::Zero(model_.nq);
    }

    Eigen::VectorXd error(model_.nq);
    for (int i = 0; i < model_.nq; i++)
    {
        if (model_options_ -> continuous(i))
        {
            error(i) = desired_q(i) - current_q(i);
            while(error(i) > M_PI)
            {
                error(i) -= 2 * M_PI;
            }
            while(error(i) < -M_PI)
            {
                error(i) += 2 * M_PI;
            }
        }
        else
        {
            error(i) = desired_q(i) - current_q(i);
        }
    }
    return error;
}

Eigen::VectorXd ModelHandler::scale_joint_velocity(const Eigen::VectorXd& dq)
{
    double scaling_factor = 1;

    for (int i = 0; i < dq.size(); i++)
    {
        scaling_factor = std::min({scaling_factor, model_options_ -> velocity_limit(i) / std::abs(dq(i))});
    }

    return scaling_factor * dq;
}

Eigen::VectorXd ModelHandler::integrate_configuration(const Eigen::VectorXd& current_q, const Eigen::VectorXd& velocity)
{
    if (current_q.size() != model_.nq)
    {
        std::cerr << "[integrate_configuration]: Invalid vector 'current_q' size." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (velocity.size() != model_.nq)
    {
        std::cerr << "[integrate_configuration]: Invalid vector 'velocity' size." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    Eigen::VectorXd new_q = current_q + velocity;
    
    for (int i = 0; i < model_.nq; i++)
    {
        if (model_options_ -> continuous(i))
        {
            while (new_q(i) > 2 * M_PI)
            {
                new_q(i) -= 2 * M_PI;
            }
            while (new_q(i) < 0)
            {
                new_q(i) += 2 * M_PI;
            }
        }
        else
        {
            if (new_q(i) > model_options_ -> upper_joint_limit(i))
            {
                new_q(i) = model_options_ -> upper_joint_limit(i);
            }
            else if (new_q(i) < model_options_ -> lower_joint_limit(i))
            {
                new_q(i) = model_options_ -> lower_joint_limit(i);
            }
        }
    }

    return new_q;
}

int ModelHandler::get_nq()
{
    return model_.nq;
}

int ModelHandler::get_nv()
{
    return model_.nv;
}

int ModelHandler::get_joint_id(const std::string& joint_name)
{
    int index = model_.getJointId(joint_name);

    if (index > model_.nq)
    {
        return -1;
    }

    return index - 1;
}

std::string ModelHandler::get_joint_name(const int index)
{
    return model_.names[index + 1];
}

Eigen::MatrixXd ModelHandler::get_geometric_jacobian(const Eigen::VectorXd& current_q, const std::string& frame_name, const pinocchio::ReferenceFrame reference_frame)
{
    int frame_index = model_.getFrameId(frame_name);

    if (frame_index == model_.nframes)
    {
        std::cout << "The frame \'" << frame_name <<"\' does not exist." << std::endl;
        return Eigen::MatrixXd::Zero(6, model_.nv);
    }

    pinocchio::forwardKinematics(model_, data_, current_q);
    pinocchio::updateFramePlacements(model_, data_);

    Eigen::MatrixXd J(6, model_.nv);
    J.setZero();

    pinocchio::computeFrameJacobian(model_, data_, current_q, frame_index, reference_frame, J);

    return J;
}

pinocchio::SE3 ModelHandler::get_frame_pose(const Eigen::VectorXd& current_q, const std::string& frame_name)
{
    int frame_index = model_.getFrameId(frame_name);

    if (frame_index == model_.nframes)
    {
        std::cout << "The frame \'" << frame_name <<"\' does not exist." << std::endl;
        pinocchio::SE3 garbage;
        return garbage;
    }

    pinocchio::forwardKinematics(model_, data_, current_q);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::SE3 pose = data_.oMf[frame_index];
    return pose;
}

Eigen::MatrixXd ModelHandler::get_dls_jacobian(const Eigen::MatrixXd& J, const double lambda)
{
    Eigen::MatrixXd JJt = J * J.transpose();
    Eigen::MatrixXd damped = JJt + (lambda * lambda) * Eigen::MatrixXd::Identity(J.rows(), J.rows());
    return J.transpose() * (damped.inverse());
}

double ModelHandler::get_condition_number(const Eigen::VectorXd& current_q, const std::string& frame_name, const pinocchio::ReferenceFrame reference_frame)
{
    Eigen::MatrixXd J = get_geometric_jacobian(current_q, frame_name, reference_frame);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd singular_values = svd.singularValues();

    double sigma_max = singular_values(0);
    double sigma_min = singular_values(singular_values.size() - 1);

    return sigma_max / sigma_min;
}

Eigen::MatrixXd ModelHandler::get_M(const Eigen::VectorXd& current_q)
{
    pinocchio::forwardKinematics(model_, data_, current_q);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::crba(model_, data_, current_q);
    Eigen::MatrixXd M = data_.M;
    M.triangularView<Eigen::StrictlyLower>() = M.transpose();
    return M;
}

Eigen::VectorXd ModelHandler::get_CG(const Eigen::VectorXd& current_q, const Eigen::VectorXd& current_dq)
{
    pinocchio::forwardKinematics(model_, data_, current_q);
    pinocchio::updateFramePlacements(model_, data_);
    Eigen::VectorXd CG = pinocchio::nonLinearEffects(model_, data_, current_q, current_dq);
    return CG;
}


Eigen::VectorXd ModelHandler::get_frictionless_tau(const Eigen::VectorXd& current_q, const Eigen::VectorXd& current_dq, const Eigen::VectorXd& current_ddq)
{
    pinocchio::forwardKinematics(model_, data_, current_q);
    pinocchio::updateFramePlacements(model_, data_);
    Eigen::VectorXd tau = pinocchio::rnea(model_, data_, current_q, current_dq, current_ddq);
    return tau;
}
