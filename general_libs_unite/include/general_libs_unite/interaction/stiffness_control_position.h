/**
 *    @file  stiffness_control_position.h
 *    @brief Stiffnes control 
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created      28-Nov-2025
 *    Modification 28-Nov-2025
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#ifndef _stiffness_control_position_
#define _stiffness_control_position_
#include <chrono>
#include <math.h>
#include <exception>
#include <fstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <nlohmann/json.hpp>

#include "general_libs_unite/general_utils/general_utils.h"
#include "general_libs_unite/serial_manipulator/diff_kinematics.h"
#include "general_libs_unite/serial_manipulator/kinodynamics_manipulator.h"

using namespace general_utils;
using namespace Eigen;

enum class armModel{bravo5, bravo7};

namespace stiffness_control_config {

    struct StiffnessJsonParams
    {
        Eigen::Vector3d pos_stiffness{50.0, 1000.0, 200.0};
        Eigen::Vector3d pos_damping{1.0, 100.0, 20.0};
        Eigen::Vector3d gain_force{0.03, 0.0, 0.0};
        Eigen::Vector3d nominal_vel{0.05, 0.0, 0.0};
        Eigen::Vector3d desired_force{1.5, 0.0, 0.0};
        Eigen::Vector3d maximum_vel{0.25, 0.25, 0.25};
    };

    inline Eigen::Vector3d json_vec3(const nlohmann::json& j, const std::string& key)
    {
        if (!j.contains(key) || !j.at(key).is_array() || j.at(key).size() != 3) {
            throw std::runtime_error("JSON key '" + key + "' must be an array of 3 numbers");
        }
        return Eigen::Vector3d(
            j.at(key).at(0).get<double>(),
            j.at(key).at(1).get<double>(),
            j.at(key).at(2).get<double>()
        );
    }

    inline StiffnessJsonParams load_stiffness_params_json(const std::string& path)
    {
        StiffnessJsonParams p;
        std::ifstream f(path);
        if (!f.is_open()) {
            throw std::runtime_error("Could not open stiffness config file: " + path);
        }
        nlohmann::json j;
        f >> j;
        p.pos_stiffness = json_vec3(j, "pos_stiffness");
        p.pos_damping   = json_vec3(j, "pos_damping");
        p.gain_force    = json_vec3(j, "gain_force");
        p.nominal_vel   = json_vec3(j, "nominal_vel");
        p.desired_force = json_vec3(j, "desired_force");
        p.maximum_vel   = json_vec3(j, "maximum_vel");
        return p;
    }
}

template <FloatingPoint T> 
    class stiffness_control_position{
        private:            
            Eigen::Vector<T, Eigen::Dynamic> joint_position_integration, joint_velocity_integration, joint_penalties;
            Eigen::Matrix<T, 3, 3> stiffness_matrix;
            std::chrono::high_resolution_clock::time_point last_computed_action;
            
            //& POSITION
            Eigen::Matrix<T, 3, 3> pos_stiff_matrix, pos_damp_matrix;
            Eigen::Vector<T, 3> Kposition{700.0, 1000.0, 1000.0};
            Eigen::Vector<T, 3> Dposition{20.0, 200.0, 200.0};

            //& Z-axis interaction params
            Eigen::Vector<T, 3> nominal_velocity{0.05, 0.0, 0.0};
            Eigen::Vector<T, 3> gain_force{0.03, 0.0, 0.0};
            Eigen::Vector<T, 3> desired_force{10.0, 0.0, 0.0};
            Eigen::Vector<T, 3> ref_ee_pos;    //! REQUIRES careful initialization
            bool ref_initialized = false;

            //& SAFETY LIMITS
            Eigen::Vector<T, 3> MAX_TASK_VEL{0.25, 0.25, 0.25}; 
            T MIN_SAMPLING_TIME = 0.05; //seconds
            T MAX_TORQUE = 3.0; //Nm //! MAYBE DELETE

            Eigen::Vector<T, 3>  vel_ee;
            
            //& OUTPUT ACTION
            Eigen::Vector<T, 3> force_action;

            //& DEBUG
            T debug_stiffness = 0.0;
            T debug_duration = 0.0;

            T position_error = 0.0; 

        private:
            void motion_integration(double elapsed, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian);
            
            Eigen::Vector<T, Eigen::Dynamic> robot_ik_diff(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian_Input);

            T compute_Z_compliance_ratio(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian);
      
            void compute_motion(Eigen::Vector<T, 3> stiffness_wrench);

        public:
            stiffness_control_position();

            void set_pos_stiffness(Eigen::Vector<T, 3> Kposition_input);

            void set_desired_force(Eigen::Vector<T, 3> desired_force_input);

            void set_nominal_vel(Eigen::Vector<T, 3> nominal_vel_input);

            void set_gain_force(Eigen::Vector<T, 3> gain_force_input);

            void set_max_vel(Eigen::Vector<T, 3> maximum_velocity_input);

            void set_ref_ee_position(Eigen::Vector<T, 3> position);

            Eigen::Vector<T, 3> get_ref_ee_position();

            void set_pos_damping(Eigen::Vector<T, 3> Dposition_input);

            Eigen::Vector<T, 3> get_vel_ee();

            Eigen::Vector<T, 3> compute_force_action(Eigen::Vector<T, 3> pos_error, Eigen::Vector<T, 3> vel_error, bool compute_motion_ref);

            std::tuple<T, T> compute_X_compliance_ratios(const Eigen::Matrix<T, 3, Eigen::Dynamic> Jacobian_linear);

            void debug_controller();
    };

#endif 
