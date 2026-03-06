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
#ifndef _STIFFNESS_CONTROL_POSITION_
#define _STIFFNESS_CONTROL_POSITION_
#include <chrono>
#include <math.h>
#include <fstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <yaml-cpp/yaml.h>

#include "bravo_cathode_protection/bravo_cpp/utils/utils.h"

enum class armModel{bravo5, bravo7};

namespace stiffness_control_config {

    struct StiffnessParams
    {
        Eigen::Vector3d pos_stiffness{50.0, 1000.0, 200.0};
        Eigen::Vector3d pos_damping{1.0, 100.0, 20.0};
        Eigen::Vector3d gain_force{0.03, 0.0, 0.0};
        Eigen::Vector3d nominal_vel{0.05, 0.0, 0.0};
        Eigen::Vector3d desired_force{1.5, 0.0, 0.0};
        Eigen::Vector3d maximum_vel{0.25, 0.25, 0.25};
    };

    inline Eigen::Vector3d yaml_vec3(const YAML::Node& node, const std::string& key)
    {
        const YAML::Node value = node[key];
        if (!value || !value.IsSequence() || value.size() != 3) {
            throw std::runtime_error("YAML key '" + key + "' must be a sequence of 3 numbers");
        }
        return Eigen::Vector3d(
            value[0].as<double>(),
            value[1].as<double>(),
            value[2].as<double>()
        );
    }

    inline StiffnessParams load_stiffness_params_yaml(const std::string& path)
    {
        if (!std::ifstream(path).good()) {
            throw std::runtime_error("Could not open stiffness config file: " + path);
        }

        const YAML::Node node = YAML::LoadFile(path);
        StiffnessParams p;
        p.pos_stiffness = yaml_vec3(node, "pos_stiffness");
        p.pos_damping   = yaml_vec3(node, "pos_damping");
        p.gain_force    = yaml_vec3(node, "gain_force");
        p.nominal_vel   = yaml_vec3(node, "nominal_vel");
        p.desired_force = yaml_vec3(node, "desired_force");
        p.maximum_vel   = yaml_vec3(node, "maximum_vel");
        return p;
    }
}

template <typename T>
concept FloatingPoint = std::same_as<T, float> || std::same_as<T, double>;

template <FloatingPoint T> 
    class stiffness_control_position{
        private:            
            std::chrono::high_resolution_clock::time_point last_computed_action;
            
            //& POSITION
            Eigen::Vector<T, 3> Kposition{700.0, 1000.0, 1000.0};
            Eigen::Vector<T, 3> Dposition{20.0, 200.0, 200.0};

            //& Z-axis interaction params
            Eigen::Vector<T, 3> nominal_velocity{0.05, 0.0, 0.0};
            Eigen::Vector<T, 3> gain_force{0.03, 0.0, 0.0};
            Eigen::Vector<T, 3> desired_force{10.0, 0.0, 0.0};
            Eigen::Vector<T, 3> ref_ee_pos;    //! REQUIRES careful initialization
            bool ref_initialized = false;
            bool reference_motion_in_local_frame = false; //FOR MOBILE AXIS CONTROL 
            Eigen::Matrix<T, 3, 3> local_to_world_rotation = Eigen::Matrix<T, 3, 3>::Identity();

            //& SAFETY LIMITS
            Eigen::Vector<T, 3> MAX_TASK_VEL{0.25, 0.25, 0.25}; 
            T MIN_SAMPLING_TIME = 0.05; //seconds

            Eigen::Vector<T, 3>  vel_ee;


        private:
            void motion_integration(double elapsed, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian);
            
            Eigen::Vector<T, Eigen::Dynamic> robot_ik_diff(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jacobian_Input);
      
            void compute_motion(Eigen::Vector<T, 3> stiffness_wrench);

        public:
            stiffness_control_position();

            void set_pos_stiffness(Eigen::Vector<T, 3> Kposition_input);

            void set_desired_force(Eigen::Vector<T, 3> desired_force_input);

            void set_nominal_vel(Eigen::Vector<T, 3> nominal_vel_input);

            void set_gain_force(Eigen::Vector<T, 3> gain_force_input);

            void set_max_vel(Eigen::Vector<T, 3> maximum_velocity_input);

            void set_ref_ee_position(Eigen::Vector<T, 3> position);

            void set_reference_integration_local(bool use_local_frame);

            void set_reference_integration_rotation(const Eigen::Matrix<T, 3, 3>& rotation_local_to_world);

            Eigen::Vector<T, 3> get_ref_ee_position();

            void set_pos_damping(Eigen::Vector<T, 3> Dposition_input);

            Eigen::Vector<T, 3> get_vel_ee();

            Eigen::Vector<T, 3> compute_force_action(Eigen::Vector<T, 3> pos_error, Eigen::Vector<T, 3> vel_error, bool compute_motion_ref);

            std::tuple<T, T> compute_X_compliance_ratios(const Eigen::Matrix<T, 3, Eigen::Dynamic> Jacobian_linear);
    };

#endif 
