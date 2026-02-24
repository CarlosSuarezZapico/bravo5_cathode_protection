#ifndef _COLLISION_PRIMITIVES_
#define _COLLISION_PRIMITIVES_

#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class sphere{
    public:
        double radius;
        Eigen::Vector<double, 3> position;

    sphere(double radius_, Eigen::Vector<double, 3> position_in)
    {
       radius = radius_;
       position = position_in;
    }
    void set_position( Eigen::Vector<double, 3> position_in)
    { 
        position = position_in;
    }
    visualization_msgs::msg::Marker rviz_visual_msg(std::string frame, int id)
    {
        visualization_msgs::msg::Marker sphere;           
        sphere.header.frame_id = frame;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.scale.x = radius*2;
        sphere.scale.y = radius*2;
        sphere.scale.z = radius*2;
        sphere.color.a = 0.6;
        sphere.color.r = 1.0;
        sphere.color.g = 0.0;
        sphere.color.b = 0.0;
        sphere.pose.position.x = position(0);
        sphere.pose.position.y = position(1);
        sphere.pose.position.z = position(2);
        sphere.id = id;
        return sphere;
    }
    //virtual ~sphere(); // destructor
};

class capsule{
    public:
        double radius;
        double length;
        Eigen::Matrix<double, 4, 4> transform;
};

class AABBbox{
    public:
        double x_length;
        double y_length;
        double z_length;
        Eigen::Vector<double, 3> position;

    AABBbox(double x_length_,double y_length_,double z_length_, Eigen::Vector<double, 3> position_in)
    {
       x_length = x_length_;
       y_length = y_length_;
       z_length = z_length_;
       position = position_in;
    }
    void set_position( Eigen::Vector<double, 3> position_in)
    { 
        position = position_in;
    }
    visualization_msgs::msg::Marker rviz_visual_msg(std::string frame, int id)
    {
        visualization_msgs::msg::Marker box;           
        box.header.frame_id = frame;
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.action = visualization_msgs::msg::Marker::ADD;
        box.scale.x = x_length;
        box.scale.y = y_length;
        box.scale.z = z_length;
        box.color.a = 0.5;
        box.color.r = 0.0;
        box.color.g = 0.458;
        box.color.b = 0.616;
        box.pose.position.x = position(0);
        box.pose.position.y = position(1);
        box.pose.position.z = position(2);
        box.id = id;
        return box;
    }
};

class OBBbox{
    public:
        double x_length;
        double y_length;
        double z_length;
        Eigen::Matrix<double, 4, 4> transform;
};

class capsule_swept_spheres{
    public:
        double radius;
        double length;
        Eigen::Matrix<double, 4, 4> transform;
        int resolution;
        int number_spheres;
        std::vector<double> spheres_location;
    capsule_swept_spheres(double radius_, double length_, int resolution_,  Eigen::Matrix<double, 4, 4> transform_in)
    {
        resolution = resolution_;
        transform = transform_in;
        radius = radius_;
        length = length_;
        number_spheres = (int) resolution*(length/radius);
        double delta = (length) / (number_spheres - 1);
        for(int i=0; i < number_spheres-1; ++i)
        {
            spheres_location.push_back(delta * i);
        }
        spheres_location.push_back(length);
    }
    void set_transform(Eigen::Matrix<double, 4, 4> transform_in)
    { 
        transform = transform_in;
    }
    visualization_msgs::msg::MarkerArray rviz_visual_msg(std::string frame, int n)
    {
        visualization_msgs::msg::MarkerArray capsule;
        for (int i=0; i<number_spheres; i++)
        {
          double relative_distance = -length/2 + spheres_location[i];
          Eigen::Matrix<double, 4, 1> position_in_origin, aux;
          aux << relative_distance, 0, 0, 1;    
          position_in_origin = transform * aux;

          visualization_msgs::msg::Marker sphere;           
          sphere.header.frame_id = frame;
          sphere.type = visualization_msgs::msg::Marker::SPHERE;
          sphere.action = visualization_msgs::msg::Marker::ADD;
          sphere.scale.x = radius*2;
          sphere.scale.y = radius*2;
          sphere.scale.z = radius*2;
          sphere.color.a = 0.6;
          sphere.color.r = 0.0;
          sphere.color.g = 1.0;
          sphere.color.b = 1.0;
          sphere.pose.position.x = position_in_origin(0,0);
          sphere.pose.position.y = position_in_origin(1,0);
          sphere.pose.position.z = position_in_origin(2,0);
          sphere.id = n+i;
          capsule.markers.push_back(sphere);
        }  
        return capsule;
    }
   //virtual ~capsule_swept_spheres(); // destructor
};

struct grid_struct{            
  float map_resolution;   //meters per cell
  int map_height;   
  int map_width;
  int *map_data;
  bool map_received;
} ;


tuple<bool, double> sphere_sphere_cpp(sphere sphere1, sphere sphere2);

double SqDistPointAABB(Eigen::Vector<double,3> point_position, AABBbox box);
tuple<bool, double> sphere_AABB_cpp(sphere sphere1, AABBbox box1);

bool OBB_OBB_cpp(Eigen::Matrix<double,4,4> OBB1_transform, double OBB1_x, double OBB1_y, double OBB1_z, Eigen::Matrix<double,4,4> OBB2_transform, double OBB2_x, double OBB2_y, double OBB2_z);
bool OBB_OBB_cpp(OBBbox box1, OBBbox box2);

tuple<bool, double> capsule_AABB_sweept_sphere_volumes(capsule_swept_spheres capsule, AABBbox box1);
tuple<bool, double> sphere_sweept_sphere_volumes(capsule_swept_spheres capsule, sphere sphere1);
bool grid_check_collision_cpu(vector<int> grid, float resolution, int height, int witdh, double base_height, double base_witdh, double dist_x, double dist_y);
bool grid_check_collision_cpu2(int* grid, float resolution, int height, int witdh, double base_height, double base_witdh, double dist_x, double dist_y);
bool debug_grid_check_collision_cpu(int *grid, float resolution, int height, int witdh, double base_height, double base_witdh, double dist_x, double dist_y);
bool debug_grid_check_collision_cpu2(int *grid, float resolution, int height, int witdh, double base_height, double base_witdh, double dist_x, double dist_y);
bool grid_collision_rbkairos(grid_struct occupancy_grid, double base_height, double base_witdh, double dist_x, double dist_y);
bool grid_collision_rbkairos(grid_struct occupancy_grid, double base_height, double base_witdh);
bool debug_grid_collision_rbkairos(grid_struct occupancy_grid, double base_height, double base_witdh, double dist_x, double dist_y);

#endif 