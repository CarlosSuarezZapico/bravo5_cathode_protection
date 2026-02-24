/**
 *    @file   kinodynamics_manipulator.h
 *    @brief  Class handler for general purpose serial manipulator
 *            using eigen and pinocchio libraries
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Project UNITE
 *    Created      24-Nov-2023
 *    Modification 24-Nov-2025
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */

#ifndef _KINODYNAMICS_MANIPULATOR_
#define _KINODYNAMICS_MANIPULATOR_

#include <vector>
#include <string>
#include <tuple>
#include <cmath>
#include <iostream>
#include <chrono>

#include <Eigen/Dense>

#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
//#include "pinocchio/algorithm/non-linear-effects.hpp"

template <typename T_data>
class kinodynamics_manipulator {
private:
    using Scalar = T_data;
    using Model  = pinocchio::ModelTpl<Scalar>;
    using Data   = pinocchio::DataTpl<Scalar>;

    using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vector6 = Eigen::Matrix<Scalar, 6, 1>;

private:
    std::vector<std::string> joint_names;
    std::string ee_frame;

    Model model_manipulator;
    Data  data_manipulator;

public:
    kinodynamics_manipulator(const std::string &urdf_filename, const std::string &toolLink)
        : model_manipulator()
        , data_manipulator(model_manipulator) // temporary; will be reset after model build
    {
        // Build typed model
        pinocchio::urdf::buildModel(urdf_filename, model_manipulator);
        data_manipulator = Data(model_manipulator);

        ee_frame = toolLink;
        joint_names = model_manipulator.names;
    }

    MatrixX fixedJacobian(const VectorX& q, const std::string& frame_name) {
        const pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(frame_name);
        MatrixX J(6, model_manipulator.nv);
        J.setZero();
        pinocchio::computeFrameJacobian(model_manipulator, data_manipulator, q, frame_id,
                                        pinocchio::LOCAL_WORLD_ALIGNED, J);
        return J;
    }

    MatrixX fixedJacobianPosition(const VectorX& q, const std::string& frame_name) {
        MatrixX J = fixedJacobian(q, frame_name);
        return J.topRows(3);
    }

    MatrixX fixedJacobian(const VectorX& q) {
        return fixedJacobian(q, ee_frame);
    }

    MatrixX fixedJacobianDerivative(const VectorX& q, const VectorX& qdot) {
        const pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(ee_frame);

        MatrixX dJ(6, model_manipulator.nv);
        dJ.setZero();

        pinocchio::computeForwardKinematicsDerivatives(
            model_manipulator, data_manipulator, q, qdot, VectorX::Zero(model_manipulator.nv));

        pinocchio::getFrameJacobianTimeVariation(
            model_manipulator, data_manipulator, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, dJ);

        return dJ;
    }

    MatrixX localJacobian(const VectorX& q, const std::string& frame_name) {
        const pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(frame_name);
        MatrixX J(6, model_manipulator.nv);
        J.setZero();
        pinocchio::computeFrameJacobian(model_manipulator, data_manipulator, q, frame_id,
                                        pinocchio::LOCAL, J);
        return J;
    }

    MatrixX localJacobianPosition(const VectorX& q, const std::string& frame_name) {
        MatrixX J = localJacobian(q, frame_name);
        return J.topRows(3);
    }

    MatrixX localJacobian(const VectorX& q) {
        return localJacobian(q, ee_frame);
    }

    MatrixX localJacobianDerivative(const VectorX& q, const VectorX& qdot) {
        const pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(ee_frame);

        MatrixX dJ(6, model_manipulator.nv);
        dJ.setZero();

        pinocchio::computeForwardKinematicsDerivatives(
            model_manipulator, data_manipulator, q, qdot, VectorX::Zero(model_manipulator.nv));

        pinocchio::getFrameJacobianTimeVariation(
            model_manipulator, data_manipulator, frame_id, pinocchio::LOCAL, dJ);

        return dJ;
    }

    void change_gravity_vector(const Vector3& gravity_vector) {
        model_manipulator.gravity.linear() = gravity_vector;
    }

    MatrixX mass_joint_space(const VectorX& q) {
        pinocchio::crba(model_manipulator, data_manipulator, q);
        // Pinocchio may leave only upper triangle filled; make it symmetric if desired:
		data_manipulator.M.template triangularView<Eigen::StrictlyLower>() =
			data_manipulator.M.transpose().template triangularView<Eigen::StrictlyLower>();
        return data_manipulator.M;
    }

    VectorX nle(const VectorX& q, const VectorX& v) {
        return pinocchio::nonLinearEffects(model_manipulator, data_manipulator, q, v);
    }

    /**
     * @brief Forward kinematics for a joint (oMi)
     */
    std::tuple<Vector3, Matrix3> FK(const VectorX& q, int JOINT_ID) {
        pinocchio::forwardKinematics(model_manipulator, data_manipulator, q);

        Vector3 x = data_manipulator.oMi[JOINT_ID].translation();
        Matrix3 R = data_manipulator.oMi[JOINT_ID].rotation();
        return std::make_tuple(x, R);
    }

    /**
     * @brief Forward kinematics of end-effector frame (oMf)
     */
    std::tuple<Vector3, Matrix3> FK_ee(const VectorX& q) {
        const pinocchio::FrameIndex frame_id = model_manipulator.getFrameId(ee_frame);

        pinocchio::forwardKinematics(model_manipulator, data_manipulator, q);
        pinocchio::updateFramePlacements(model_manipulator, data_manipulator);

        Vector3 x = data_manipulator.oMf[frame_id].translation();
        Matrix3 R = data_manipulator.oMf[frame_id].rotation();
        return std::make_tuple(x, R);
    }

    Eigen::Vector3d FK_ee_pos(const Eigen::VectorXd& q){
        const auto frame_id = model_manipulator.getFrameId(ee_frame);
        pinocchio::forwardKinematics(model_manipulator, data_manipulator, q);
        pinocchio::updateFramePlacements(model_manipulator, data_manipulator);
        return data_manipulator.oMf[frame_id].translation().template cast<double>();
    }

    /**
     * @brief Calculate SE(3) error between two joint configurations
     */
    Vector6 compute_SE3_error_fixed_axis(const VectorX& q_ref, const VectorX& q_curr) {
        Vector3 p_ref, p_curr;
        Matrix3 R_ref, R_curr;

        std::tie(p_ref, R_ref) = FK_ee(q_ref);
        std::tie(p_curr, R_curr) = FK_ee(q_curr);

        const Vector3 pos_error = p_ref - p_curr;
        const Matrix3 R_err = R_ref * R_curr.transpose();

        Vector3 orientation_error;
        orientation_error <<
            Scalar(0.5) * (R_err(2,1) - R_err(1,2)),
            Scalar(0.5) * (R_err(0,2) - R_err(2,0)),
            Scalar(0.5) * (R_err(1,0) - R_err(0,1));

        Vector6 SE3_error;
        SE3_error.template head<3>() = pos_error;
        SE3_error.template tail<3>() = orientation_error;
        return SE3_error;
    }

    Scalar cond_arm(const MatrixX& J) {
        Eigen::JacobiSVD<MatrixX> svd(J);
        const auto& s = svd.singularValues();
        return s(0) / s(s.size() - 1);
    }

    VectorX invDynamics(const VectorX& q, const VectorX& v, const VectorX& a) {
        return pinocchio::rnea(model_manipulator, data_manipulator, q, v, a);
    }

    VectorX compute_I_ff_stribeck_smooth(const VectorX& qd,
                                         const VectorX& I_coulomb,
                                         const VectorX& I_stiction,
                                         Scalar v_stiction,
                                         Scalar k,
                                         Scalar vel_eps) {
        VectorX out(qd.size());
        for (int i = 0; i < qd.size(); ++i) {
            const Scalar v = qd[i];
            const Scalar blend = Scalar(1) / (Scalar(1) + std::pow(std::abs(v) / v_stiction, k));
            const Scalar sgn_smooth = std::tanh(v / vel_eps);
            out[i] = (I_coulomb[i] + (I_stiction[i] - I_coulomb[i]) * blend) * sgn_smooth;
        }
        return out;
    }

    VectorX compute_I_ff_stribeck_smooth(const VectorX& qd,
                                         const VectorX& I_coulomb,
                                         const VectorX& I_stiction,
                                         const VectorX& v_stiction,
                                         const VectorX& k,
                                         const VectorX& vel_eps) {
        VectorX out(qd.size());
        for (int i = 0; i < qd.size(); ++i) {
            const Scalar v = qd[i];
            const Scalar blend = Scalar(1) / (Scalar(1) + std::pow(std::abs(v) / v_stiction[i], k[i]));
            const Scalar sgn_smooth = std::tanh(v / vel_eps[i]);
            out[i] = (I_coulomb[i] + (I_stiction[i] - I_coulomb[i]) * blend) * sgn_smooth;
        }
        return out;
    }

    VectorX compute_I_ff_stribeck(const VectorX& qd,
                                  const VectorX& I_coulomb,
                                  const VectorX& I_stiction,
                                  const VectorX& qd_stiction2) {
        const VectorX sgn = qd.array().sign().matrix();
        const VectorX exp_term = (-((qd.array().abs() / qd_stiction2.array()).square())).exp().matrix();
        VectorX I_ff = (I_coulomb.array() + (I_stiction.array() - I_coulomb.array()) * exp_term.array()).matrix();
        I_ff = I_ff.array() * sgn.array();
        return I_ff;
    }

    double computeFriction(double velocity, double g0, double g1, double g2, double g3, double g4, double g5, double saturation) {
        // Compute tanh terms
        double tanh1v_ = std::tanh(g1 * velocity);
        double tanh2v_ = std::tanh(g2 * velocity);
        double tanh4v_ = std::tanh(g4 * velocity);
        // Compute friction torque following the Python model
        double frictionTorque_ = 0.0;
        frictionTorque_ += g0 * tanh1v_;
        frictionTorque_ -= g0 * tanh2v_;
        frictionTorque_ += g3 * tanh4v_;
        frictionTorque_ += g5 * velocity;
        if (frictionTorque_ >= saturation)
            frictionTorque_ = saturation;
        if (frictionTorque_ <= -saturation)
            frictionTorque_ = -saturation;
        return frictionTorque_;
    }
};

#endif // _KINODYNAMICS_MANIPULATOR_
