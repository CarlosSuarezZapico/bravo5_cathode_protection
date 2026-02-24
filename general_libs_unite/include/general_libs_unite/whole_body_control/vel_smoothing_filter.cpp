/**
 *    @file  vel_smoothing_filter.cpp
 *    @brief Velocity smoothing filter for clumsy actuated DOFs
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  7-May-2024
 *    Modification 7-May-2024
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */

#include "general_libs_unite/whole_body_control/vel_smoothing_filter.h"

template <FloatingPoint T> 
    vel_smoothing_filter<T>::vel_smoothing_filter(T max_acc_value, T max_vel_value){           
        max_acc = max_acc_value; //! NOT SURE ABOUT THIS
        max_vel = max_vel_value;
        last_compute_time = std::chrono::high_resolution_clock::now();
    }

template <FloatingPoint T> 
    T vel_smoothing_filter<T>::compute(T vel_input){     
        std::chrono::duration<double> elapsed_filter= std::chrono::high_resolution_clock::now()-last_compute_time;
        double elapsed = elapsed_filter.count();
        T out_vel = general_utils::VAL_SAT<double>(vel_input, max_vel, -max_vel);
        out_vel = general_utils::VAL_SAT<double>( out_vel, pre_vel + max_acc*elapsed,  pre_vel - max_acc*elapsed);
        pre_vel = out_vel;
        last_compute_time =  std::chrono::high_resolution_clock::now();
        return out_vel;
    }

template <FloatingPoint T> 
    void vel_smoothing_filter<T>::reset_filter(){
        pre_vel = 0.0;
        last_compute_time = std::chrono::high_resolution_clock::now();
    }

template class vel_smoothing_filter<double>;