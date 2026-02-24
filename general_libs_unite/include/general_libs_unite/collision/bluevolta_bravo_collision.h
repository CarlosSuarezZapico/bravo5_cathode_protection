/**
 *    @file  bluevolta_bravo_collision.h
 *    @brief Library for collision checking on Bluevolta and Bravo
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Created  24-Sep-2024
 *    Modification 24-Sep-2024
 *    Project: UNITE
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ===============================================================
 */
#ifndef _BLUEVOLTA_BRAVO_COLLISION_
#define _BLUEVOLTA_BRAVO_COLLISION_

#include <iostream>
#include <math.h>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono> 

#include <pinocchio/algorithm/kinematics.hpp>
#include "uvm_falcon_bravo/collision/collisions.h"


using namespace std;
using namespace Eigen;

class bluevolta_bravo_collision{
private:

    Matrix4d T_footprint_AABB_base_bias   = (Matrix4d() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.25, 0, 0, 0, 1).finished();// 0.50912/2 in z
    Matrix4d T_col_base_capsule_bias      = (Matrix4d() << cos(M_PI/2), -sin(M_PI/2), 0, 0, sin(M_PI/2), cos(M_PI/2), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1).finished(); //0.23/2 in z
    Matrix4d T_col_link1_capsule_bias     = (Matrix4d() << 1, 0, 0, 0.18, 0, 1, 0, 0, 0, 0, 1, 0.13, 0, 0, 0, 1).finished();//0.55/2
    Matrix4d T_col_link2_capsule_bias     = (Matrix4d() << 1, 0, 0, 0.18, 0, 1, 0, 0, 0, 0, 1, 0.01, 0, 0, 0, 1).finished();//0.50/2
    Matrix4d T_col_link3_capsule_bias     = (Matrix4d() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1).finished();//? 22cm
    Matrix4d T_col_link4_capsule_bias     = (Matrix4d() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1).finished();//? 17cm
    Matrix4d T_col_link5_capsule_bias     = (Matrix4d() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1).finished();//? ?? 20 cm
    Matrix4d T_col_linktool_capsule_bias  = (Matrix4d() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1).finished();//? ?? 20 cm
    //Matrix4d T_footprint_armbase= (Matrix4d() << cos(-M_PI/2), -sin(-M_PI/2), 0, 0.16, sin(-M_PI/2), cos(-M_PI/2), 0, 0, 0, 0, 1, 0.50762, 0, 0, 0, 1).finished();
    Matrix4d T_footprint_armbase = (Matrix4d() << cos(-M_PI), -sin(-M_PI), 0, 0.16, sin(-M_PI), cos(-M_PI), 0, 0, 0, 0, 1, 0.50912, 0, 0, 0, 1).finished();

public:
    //& Pinocchion handler
		Model model_manipulator;
		Data data_manipulator;
  
  bluevolta_bravo_collision(Model model, Data data) : model_manipulator(model), data_manipulator(data){}
  
  Matrix<double,4,4> AH2(int n, Matrix<double, 6,1> joint);

  tuple<Matrix4d, Matrix4d, Matrix4d, Matrix4d, Matrix4d, Matrix4d, Matrix4d> fKine_collision_links(Matrix<double, 6, 1> joint);

  tuple<visualization_msgs::MarkerArray, double, bool> self_collision( Matrix<double, 6, 1> arm_joint_states);

  bool self_collision_check( Matrix<double, 6, 1> arm_joint_states);

  bool self_collision_check_pinocchio(Eigen::VectorXd q);

  visualization_msgs::MarkerArray visualize_self_collision(AABBbox base_AABB, capsule_swept_spheres link0_capsule, capsule_swept_spheres link1_capsule,\
          capsule_swept_spheres link2_capsule, sphere link3_sphere, sphere link4_sphere, sphere tool_sphere);
  
  }; 
#endif 
