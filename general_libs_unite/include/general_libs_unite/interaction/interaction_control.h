/**
 *    @file  interaction_control.h
 *    @brief library for force control
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  25-Sep-2023
 *    Modification 9-Sep-2024
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#ifndef _INTERACTION_CONTROL_
#define _INTERACTION_CONTROL_

#include "general_libs_unite/general_utils/general_utils.h"
#include <chrono>
#include <type_traits>

using namespace std;
using namespace Eigen;
using namespace general_utils;

template <typename T, typename Total, size_t N>
class Moving_Average
{
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

class interaction_control{

	public:	    
		double admittance_vel    = 0.0;
		double max_linear_speed  = 0.25;
		double max_angular_speed = 0.43;

		//ADMITTANCE_CONTROL
		double Mass =3.5; //3
		double Damp = 350; //250
		double velPreZ = 0.0;
		std::chrono::high_resolution_clock::time_point last_command_admittance; 
		       
		interaction_control(){
			admittance_vel = 0.0;
        	last_command_admittance     = chrono::high_resolution_clock::now();
		}
		Matrix<double, 6,1> safety_speed_saturation(Vector<double, 6> twist_ee){
			Vector<double, 6> twist_ee_saturated;
			twist_ee_saturated(0) = general_utils::VAL_SAT<double>(twist_ee[0], max_linear_speed,  -max_linear_speed);
			twist_ee_saturated(1) = general_utils::VAL_SAT<double>(twist_ee[1], max_linear_speed,  -max_linear_speed);
			twist_ee_saturated(2) = general_utils::VAL_SAT<double>(twist_ee[2], max_linear_speed,  -max_linear_speed);
			twist_ee_saturated(3) = general_utils::VAL_SAT<double>(twist_ee[3], max_angular_speed, -max_angular_speed);
			twist_ee_saturated(4) = general_utils::VAL_SAT<double>(twist_ee[4], max_angular_speed, -max_angular_speed);
			twist_ee_saturated(5) = general_utils::VAL_SAT<double>(twist_ee[5], max_angular_speed, -max_angular_speed);
			return twist_ee_saturated;	
		}
 		double md_admittance_force_control(double Mass, double Damp, double desired_Force, double dt, double &velPre, double current_Force){
			double forceZ_error = -current_Force + desired_Force;
			double vel_output = (1/Mass)*(forceZ_error - Damp*(velPre))*dt + velPre;
			velPre = vel_output;
			return vel_output;	
		}
	
		void Z_md_admittance_force_control_loop(double sampling_time, double desired_Force, double current_Force){
			std::chrono::duration<double> elapsed= std::chrono::high_resolution_clock::now()-last_command_admittance;
			if (elapsed.count() > sampling_time) {
					admittance_vel = interaction_control::md_admittance_force_control(Mass, Damp, desired_Force, sampling_time, velPreZ, current_Force);
					last_command_admittance = std::chrono::high_resolution_clock::now();
			}
		}
};

template <typename T_data>
concept FloatVariable = std::same_as<T_data, float> || std::same_as<T_data, double>;

template <FloatVariable T_data>  class force_control{

	private:	    
		T_data admittance_vel    = 0.0;
		//ADMITTANCE_CONTROL
		T_data Mass =2; //3
		T_data Damp = 250; //250
		T_data velPreZ = 0.0;
		std::chrono::high_resolution_clock::time_point last_command_admittance; 

	public:     
		force_control(){
			admittance_vel = 0.0;
        	last_command_admittance     = chrono::high_resolution_clock::now();
		}

		T_data safety_speed_saturation(T_data vel_input, T_data max_vel){
			return general_utils::VAL_SAT<T_data>(vel_input, max_vel,  -max_vel);
		}

 		T_data md_admittance_force_control(T_data desired_Force, T_data dt, T_data current_Force){
			T_data forceZ_error = -current_Force + desired_Force;
			T_data vel_output = (1/Mass)*(forceZ_error - Damp*(velPreZ))*dt + velPreZ;
			velPreZ = vel_output;
			return vel_output;	
		}

		void set_admittance_parameters(T_data mass_param, T_data damp_param){
			Mass = mass_param;
			Damp = damp_param;
		}

		void reset_velPre(){
			velPreZ = 0.0;
		}

		T_data get_admittance_vel(){
			return admittance_vel;
		}
	
		void md_admittance_force_control_loop(T_data sampling_time, T_data desired_Force, T_data current_Force){
			std::chrono::duration<double> elapsed= std::chrono::high_resolution_clock::now()-last_command_admittance;
			if (elapsed.count() > sampling_time) {
					admittance_vel = force_control::md_admittance_force_control(desired_Force, sampling_time, current_Force);
					last_command_admittance = std::chrono::high_resolution_clock::now();
			}
		}
};


#endif
