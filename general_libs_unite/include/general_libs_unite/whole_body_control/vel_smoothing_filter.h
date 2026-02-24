/**
 *    @file  vel_smoothing_filter.h
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
#ifndef _VEL_SMOOTHING_FILTER_
#define _VEL_SMOOTHING_FILTER_
#include <chrono>
#include <math.h>
#include <exception>
#include <type_traits>

#include "general_libs_unite/general_utils/general_utils.h"

using namespace std;
using namespace general_utils;
using namespace Eigen;

template <typename T>
concept FloatingPoint = std::same_as<T, float> || std::same_as<T, double>;

template <FloatingPoint T> 
    class vel_smoothing_filter{
        private:                
            T   pre_vel = 0.0;
            T   max_acc = 0.0;
            T   max_vel = 0.0;
            std::chrono::high_resolution_clock::time_point last_compute_time;
        public:
            vel_smoothing_filter(T max_acc_value = 0.0, T max_vel_value = 0.0);            
            T compute(T vel_input);
            void reset_filter();

    };

#endif 