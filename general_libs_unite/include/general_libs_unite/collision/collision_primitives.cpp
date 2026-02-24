#include "general_libs_unite/collision/collision_primitives.h"

using namespace std;
using namespace Eigen;


tuple<bool, double> sphere_sphere_cpp(sphere sphere1, sphere sphere2)
{
  //returning multiple values in cpp method: https://www.educative.io/answers/how-to-return-multiple-values-from-a-function-in-cpp17
  double distance = sqrt(pow((sphere1.position(0)-sphere2.position(0)),2) + pow((sphere1.position(1)-sphere2.position(1)),2) + pow((sphere1.position(2)-sphere2.position(2)),2));
  double distance_from_collision = distance - (sphere1.radius + sphere2.radius);
  bool OnCollision = true;
  if (distance_from_collision < 0.0){
    OnCollision = true;
  }
  else{
    OnCollision = false;
  }
  return {OnCollision, distance_from_collision};
}

double SqDistPointAABB(Eigen::Vector<double,3> point_position, AABBbox box)
{
  Eigen::Matrix<double,4,4> AABB_transform;
  AABB_transform << 1 ,0 ,0 ,box.position(0),  0 ,1 ,0 ,box.position(1),  0 ,0 ,1 ,box.position(2),    0 ,0 ,0 ,1;
  Eigen::Matrix<double,3,1> b_min, b_max;
  Eigen::Matrix<double,4,1> b_min_1;
  Eigen::Matrix<double,4,1> dim_box, dim_box2;
  dim_box << -box.x_length/2, -box.y_length/2, -box.z_length/2, 1;
  dim_box2 << box.x_length/2, box.y_length/2, box.z_length/2, 1;
  b_min_1 = AABB_transform * dim_box;
  Eigen::Matrix<double,4,1> b_max_1;
  b_max_1 = AABB_transform * dim_box2;
  b_min <<b_min_1(0,0), b_min_1(1,0), b_min_1(2,0);
  b_max <<b_max_1(0,0), b_max_1(1,0), b_max_1(2,0);

  double sqDist =0.0;
  for (int i =0; i<3; i++)
  {
    //for each axis count any excess distance outside box extents
    double v = point_position(i);
    if (v < b_min(i)){
      sqDist = sqDist + pow((b_min(i) - v), 2);
    }
    if (v > b_max(i)){
      sqDist = sqDist + pow((b_max(i) - v), 2);
    }
  }
  return sqDist;
}

tuple<bool, double> sphere_AABB_cpp(sphere sphere1, AABBbox box1)
{
  double square_distance_from_collision= SqDistPointAABB(sphere1.position, box1);
  double distance_from_collision = square_distance_from_collision - sphere1.radius*sphere1.radius;
  bool OnCollision = true;
  if (distance_from_collision < 0){
    OnCollision = true;
  }
  else{
    OnCollision = false;
  }
  return {OnCollision, distance_from_collision};
}


bool OBB_OBB_cpp(Eigen::Matrix<double,4,4> OBB1_transform, double OBB1_x, double OBB1_y, double OBB1_z, Eigen::Matrix<double,4,4> OBB2_transform, double OBB2_x, double OBB2_y, double OBB2_z)
{
  Eigen::Vector<float,3> A0(OBB1_transform(0,0), OBB1_transform(0,1), OBB1_transform(0,2)); 
  Eigen::Vector<float,3> A1(OBB1_transform(1,0), OBB1_transform(1,1), OBB1_transform(1,2));
  Eigen::Vector<float,3> A2(OBB1_transform(2,0), OBB1_transform(2,1), OBB1_transform(2,2));

  Eigen::Vector<float,3> B0(OBB2_transform(0,0), OBB2_transform(0,1), OBB2_transform(0,2)); 
  Eigen::Vector<float,3> B1(OBB2_transform(1,0), OBB2_transform(1,1), OBB2_transform(1,2));
  Eigen::Vector<float,3> B2(OBB2_transform(2,0), OBB2_transform(2,1), OBB2_transform(2,2));

  Eigen::Vector<float,3> positionA(OBB1_transform(0,3), OBB1_transform(1,3), OBB1_transform(2,3));
  Eigen::Vector<float,3> positionB(OBB2_transform(0,3), OBB2_transform(1,3), OBB2_transform(2,3));
  
  Eigen::Vector<float,3> D = positionB - positionA;

  double a0 = OBB1_x/2;
  double a1 = OBB1_y/2;
  double a2 = OBB1_z/2;

  double b0 = OBB2_x/2;
  double b1 = OBB2_y/2;
  double b2 = OBB2_z/2;

  Eigen::Matrix<double,4,4> Rot_Conversion_A_B = OBB1_transform.inverse() * OBB2_transform; 

  //A0---1
  double R0 = a0;
  double R1 = abs(Rot_Conversion_A_B(0,0)) * b0 + abs(Rot_Conversion_A_B(0,1)) * b1 + abs(Rot_Conversion_A_B(0,2)) * b2;
  double R = abs(A0.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //A1---2
  R0 = a1;
  R1 = abs(Rot_Conversion_A_B(1,0)) * b0 + abs(Rot_Conversion_A_B(1,1)) * b1 + abs(Rot_Conversion_A_B(1,2)) * b2;
  R = abs(A1.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //A2---3
  R0 = a2;
  R1 = abs(Rot_Conversion_A_B(2,0)) * b0 + abs(Rot_Conversion_A_B(2,1)) * b1 + abs(Rot_Conversion_A_B(2,2)) * b2;
  R = abs(A2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //B0---4
  R1 = b0;
  R1 = abs(Rot_Conversion_A_B(0,0)) * a0 + abs(Rot_Conversion_A_B(1,0)) * a1 + abs(Rot_Conversion_A_B(2,0)) * a2;
  R = abs(B0.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //B1---5
  R1 = b1;
  R0 = abs(Rot_Conversion_A_B(0,1)) * a0 + abs(Rot_Conversion_A_B(1,1)) * a1 + abs(Rot_Conversion_A_B(2,1)) * a2;
  R = abs(B1.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //B2---6
  R1 = b2;
  R0 = abs(Rot_Conversion_A_B(0,2)) * a0 + abs(Rot_Conversion_A_B(1,2)) * a1 + abs(Rot_Conversion_A_B(2,2)) * a2;
  R = abs(B2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //A0xB0---7 
  R0 = abs(Rot_Conversion_A_B(2,0)) * a1 + abs(Rot_Conversion_A_B(1,0)) * a2;
  R1 = b1 * abs(Rot_Conversion_A_B(0,2)) + b2 * abs(Rot_Conversion_A_B(0,1));
  Eigen::Vector<float,3> v1 = Rot_Conversion_A_B(1,0) * A2;
  Eigen::Vector<float,3> v2 = Rot_Conversion_A_B(2,0) * A1; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //A0xB1---8 
  R0 = abs(Rot_Conversion_A_B(2,1)) * a1 + abs(Rot_Conversion_A_B(1,1)) * a2;
  R1 = b0 * abs(Rot_Conversion_A_B(0,2)) + b2 * abs(Rot_Conversion_A_B(0,0));
  v1 = Rot_Conversion_A_B(1,1) * A2;
  v2 = Rot_Conversion_A_B(2,1) * A1; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //A0xB2---9 
  R0 = abs(Rot_Conversion_A_B(2,2)) * a1 + abs(Rot_Conversion_A_B(1,2)) * a2;
  R1 = b0 * abs(Rot_Conversion_A_B(0,1)) + b1 * abs(Rot_Conversion_A_B(0,0));
  v1 = Rot_Conversion_A_B(1,2) * A2;
  v2 = Rot_Conversion_A_B(2,2) * A1; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }  
        
  //A1xB0---10 
  R0 = abs(Rot_Conversion_A_B(2,0)) * a0 + abs(Rot_Conversion_A_B(0,0)) * a2;
  R1 = b1 * abs(Rot_Conversion_A_B(1,2)) + b2 * abs(Rot_Conversion_A_B(1,1));
  v1 = Rot_Conversion_A_B(2,0) * A0;
  v2 = Rot_Conversion_A_B(0,0) * A2; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }  

  //A1xB1---11 
  R0 = abs(Rot_Conversion_A_B(2,1)) * a0 + abs(Rot_Conversion_A_B(0,1)) * a2;
  R1 = b0 * abs(Rot_Conversion_A_B(1,2)) + b2 * abs(Rot_Conversion_A_B(1,0));
  v1 = Rot_Conversion_A_B(2,1) * A0;
  v2 = Rot_Conversion_A_B(0,1) * A2; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }  

  //A1xB2---12
  R0 = abs(Rot_Conversion_A_B(2,2)) * a0 + abs(Rot_Conversion_A_B(0,2)) * a2;
  R1 = b0 * abs(Rot_Conversion_A_B(1,1)) + b1 * abs(Rot_Conversion_A_B(1,0));
  v1 = Rot_Conversion_A_B(2,2) * A0;
  v2 = Rot_Conversion_A_B(0,2) * A2; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }   

  //A2xB0---13
  R0 = abs(Rot_Conversion_A_B(1,0)) * a0 + abs(Rot_Conversion_A_B(0,0)) * a1;
  R1 = b1 * abs(Rot_Conversion_A_B(2,2)) + b2 * abs(Rot_Conversion_A_B(2,1));
  v1 = Rot_Conversion_A_B(0,0) * A1;
  v2 = Rot_Conversion_A_B(1,0) * A0; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }   

  //A2xB1---14
  R0 = abs(Rot_Conversion_A_B(1,1)) * a0 + abs(Rot_Conversion_A_B(0,1)) * a1;
  R1 = b0 * abs(Rot_Conversion_A_B(2,2)) + b2 * abs(Rot_Conversion_A_B(2,0));
  v1 = Rot_Conversion_A_B(0,1) * A1;
  v2 = Rot_Conversion_A_B(1,1) * A0; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }  

  //A2xB2---15
  R0 = abs(Rot_Conversion_A_B(1,2)) * a0 + abs(Rot_Conversion_A_B(0,2)) * a1;
  R1 = b0 * abs(Rot_Conversion_A_B(2,1)) + b1 * abs(Rot_Conversion_A_B(2,0));
  v1 = Rot_Conversion_A_B(0,2) * A1;
  v2 = Rot_Conversion_A_B(1,2) * A0; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }      

  return true;

}

bool OBB_OBB_cpp(OBBbox box1, OBBbox box2)
{
  Eigen::Vector<float,3> A0(box1.transform(0,0), box1.transform(0,1), box1.transform(0,2)); 
  Eigen::Vector<float,3> A1(box1.transform(1,0), box1.transform(1,1), box1.transform(1,2));
  Eigen::Vector<float,3> A2(box1.transform(2,0), box1.transform(2,1), box1.transform(2,2));

  Eigen::Vector<float,3> B0(box2.transform(0,0), box2.transform(0,1), box2.transform(0,2)); 
  Eigen::Vector<float,3> B1(box2.transform(1,0), box2.transform(1,1), box2.transform(1,2));
  Eigen::Vector<float,3> B2(box2.transform(2,0), box2.transform(2,1), box2.transform(2,2));

  Eigen::Vector<float,3> positionA(box1.transform(0,3), box1.transform(1,3), box1.transform(2,3));
  Eigen::Vector<float,3> positionB(box2.transform(0,3), box2.transform(1,3), box2.transform(2,3));
  
  Eigen::Vector<float,3> D = positionB - positionA;

  double a0 = box1.x_length/2;
  double a1 = box1.y_length/2;
  double a2 = box1.z_length/2;

  double b0 = box2.x_length/2;
  double b1 = box2.y_length/2;
  double b2 = box2.z_length/2;

  Eigen::Matrix<double,4,4> Rot_Conversion_A_B = box1.transform.inverse() * box2.transform; 

  //A0---1
  double R0 = a0;
  double R1 = abs(Rot_Conversion_A_B(0,0)) * b0 + abs(Rot_Conversion_A_B(0,1)) * b1 + abs(Rot_Conversion_A_B(0,2)) * b2;
  double R = abs(A0.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //A1---2
  R0 = a1;
  R1 = abs(Rot_Conversion_A_B(1,0)) * b0 + abs(Rot_Conversion_A_B(1,1)) * b1 + abs(Rot_Conversion_A_B(1,2)) * b2;
  R = abs(A1.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //A2---3
  R0 = a2;
  R1 = abs(Rot_Conversion_A_B(2,0)) * b0 + abs(Rot_Conversion_A_B(2,1)) * b1 + abs(Rot_Conversion_A_B(2,2)) * b2;
  R = abs(A2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //B0---4
  R1 = b0;
  R1 = abs(Rot_Conversion_A_B(0,0)) * a0 + abs(Rot_Conversion_A_B(1,0)) * a1 + abs(Rot_Conversion_A_B(2,0)) * a2;
  R = abs(B0.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //B1---5
  R1 = b1;
  R0 = abs(Rot_Conversion_A_B(0,1)) * a0 + abs(Rot_Conversion_A_B(1,1)) * a1 + abs(Rot_Conversion_A_B(2,1)) * a2;
  R = abs(B1.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //B2---6
  R1 = b2;
  R0 = abs(Rot_Conversion_A_B(0,2)) * a0 + abs(Rot_Conversion_A_B(1,2)) * a1 + abs(Rot_Conversion_A_B(2,2)) * a2;
  R = abs(B2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //A0xB0---7 
  R0 = abs(Rot_Conversion_A_B(2,0)) * a1 + abs(Rot_Conversion_A_B(1,0)) * a2;
  R1 = b1 * abs(Rot_Conversion_A_B(0,2)) + b2 * abs(Rot_Conversion_A_B(0,1));
  Eigen::Vector<float,3> v1 = Rot_Conversion_A_B(1,0) * A2;
  Eigen::Vector<float,3> v2 = Rot_Conversion_A_B(2,0) * A1; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //A0xB1---8 
  R0 = abs(Rot_Conversion_A_B(2,1)) * a1 + abs(Rot_Conversion_A_B(1,1)) * a2;
  R1 = b0 * abs(Rot_Conversion_A_B(0,2)) + b2 * abs(Rot_Conversion_A_B(0,0));
  v1 = Rot_Conversion_A_B(1,1) * A2;
  v2 = Rot_Conversion_A_B(2,1) * A1; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }

  //A0xB2---9 
  R0 = abs(Rot_Conversion_A_B(2,2)) * a1 + abs(Rot_Conversion_A_B(1,2)) * a2;
  R1 = b0 * abs(Rot_Conversion_A_B(0,1)) + b1 * abs(Rot_Conversion_A_B(0,0));
  v1 = Rot_Conversion_A_B(1,2) * A2;
  v2 = Rot_Conversion_A_B(2,2) * A1; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }  
        
  //A1xB0---10 
  R0 = abs(Rot_Conversion_A_B(2,0)) * a0 + abs(Rot_Conversion_A_B(0,0)) * a2;
  R1 = b1 * abs(Rot_Conversion_A_B(1,2)) + b2 * abs(Rot_Conversion_A_B(1,1));
  v1 = Rot_Conversion_A_B(2,0) * A0;
  v2 = Rot_Conversion_A_B(0,0) * A2; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }  

  //A1xB1---11 
  R0 = abs(Rot_Conversion_A_B(2,1)) * a0 + abs(Rot_Conversion_A_B(0,1)) * a2;
  R1 = b0 * abs(Rot_Conversion_A_B(1,2)) + b2 * abs(Rot_Conversion_A_B(1,0));
  v1 = Rot_Conversion_A_B(2,1) * A0;
  v2 = Rot_Conversion_A_B(0,1) * A2; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }  

  //A1xB2---12
  R0 = abs(Rot_Conversion_A_B(2,2)) * a0 + abs(Rot_Conversion_A_B(0,2)) * a2;
  R1 = b0 * abs(Rot_Conversion_A_B(1,1)) + b1 * abs(Rot_Conversion_A_B(1,0));
  v1 = Rot_Conversion_A_B(2,2) * A0;
  v2 = Rot_Conversion_A_B(0,2) * A2; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }   

  //A2xB0---13
  R0 = abs(Rot_Conversion_A_B(1,0)) * a0 + abs(Rot_Conversion_A_B(0,0)) * a1;
  R1 = b1 * abs(Rot_Conversion_A_B(2,2)) + b2 * abs(Rot_Conversion_A_B(2,1));
  v1 = Rot_Conversion_A_B(0,0) * A1;
  v2 = Rot_Conversion_A_B(1,0) * A0; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }   

  //A2xB1---14
  R0 = abs(Rot_Conversion_A_B(1,1)) * a0 + abs(Rot_Conversion_A_B(0,1)) * a1;
  R1 = b0 * abs(Rot_Conversion_A_B(2,2)) + b2 * abs(Rot_Conversion_A_B(2,0));
  v1 = Rot_Conversion_A_B(0,1) * A1;
  v2 = Rot_Conversion_A_B(1,1) * A0; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }  

  //A2xB2---15
  R0 = abs(Rot_Conversion_A_B(1,2)) * a0 + abs(Rot_Conversion_A_B(0,2)) * a1;
  R1 = b0 * abs(Rot_Conversion_A_B(2,1)) + b1 * abs(Rot_Conversion_A_B(2,0));
  v1 = Rot_Conversion_A_B(0,2) * A1;
  v2 = Rot_Conversion_A_B(1,2) * A0; 
  R = abs(v1.dot(D) - v2.dot(D));
  if (R > (R1 + R0)){
    return false;
  }      

  return true;

}

tuple<bool, double>  capsule_AABB_sweept_sphere_volumes(capsule_swept_spheres capsule1, AABBbox box1)
{
  //Idea do a Swept sphere along the main axis of the capsule
  //Like checking with a certain resolution using sphere_AABB
  double length_capsule = capsule1.length;
  bool OnCollision; 
  Eigen::Matrix<double, 4, 1> position_origin;
  Eigen::Matrix<double, 4, 1> origin;
  origin << -length_capsule/2, 0, 0, 1;
  position_origin = capsule1.transform * origin;
  vector<double> distances_array(capsule1.number_spheres);
  double minimum = 1000000;
  for( int i = 0; i<capsule1.number_spheres; i++)
  {
    double relative_distance = -length_capsule/2 + capsule1.spheres_location[i];
    Eigen::Matrix<double, 4, 1> aux;
    aux << relative_distance, 0, 0, 1;
    origin = capsule1.transform * aux;
    Eigen::Vector<double,3> position_sphere;
    position_sphere << position_origin(0), position_origin(1), position_origin(2);
    sphere sphere_n(capsule1.radius, position_sphere);
    double distance;
    tie(OnCollision, distance) = sphere_AABB_cpp(sphere_n, box1);
    if(OnCollision == true){
      return {OnCollision, 0.0};
    }
    else
    {
      distances_array.push_back(distance);
      if (distance < minimum)
      {
        minimum = distance;
      }
    }

  }
  return {false, minimum};
}

tuple<bool, double> sphere_sweept_sphere_volumes(capsule_swept_spheres capsule1, sphere sphere1)
{
  //Idea do a Swept sphere along the main axis of the capsule
  //Like checking with a certain resolution using sphere_AABB
  double length_capsule = capsule1.length;
  bool OnCollision; 
  Eigen::Matrix<double, 4, 1> position_origin;
  Eigen::Matrix<double, 4, 1> origin;
  origin << -length_capsule/2, 0, 0, 1;
  position_origin = capsule1.transform * origin;
  vector<double> distances_array(capsule1.number_spheres);
  double minimum = 1000000;
  for( int i = 0; i<capsule1.number_spheres; i++)
  {
    double relative_distance = -length_capsule/2 + capsule1.spheres_location[i];
    Eigen::Matrix<double, 4, 1> aux;
    aux << relative_distance, 0, 0, 1;
    origin = capsule1.transform * aux;
    Eigen::Vector<double,3> position_sphere;
    position_sphere << position_origin(0), position_origin(1), position_origin(2);
    sphere sphere_n(capsule1.radius, position_sphere);
    double distance;
    tie(OnCollision, distance) = sphere_sphere_cpp(sphere_n, sphere1);
    if(OnCollision == true){
      return {OnCollision, 0.0};
    }
    else
    {
      distances_array.push_back(distance);
      if (distance < minimum)
      {
        minimum = distance;
      }
    }

  }
  return {false, minimum};
}


bool grid_check_collision_cpu(vector<int> grid, float resolution, int height, int witdh, double base_height, double base_witdh, double dist_x, double dist_y) {

	// returns if there is collision (true) or not (false)
	int dist_x_cells = dist_x/resolution + witdh/2;
	int dist_y_cells = dist_y/resolution + height/2;

  int base_height_cells = base_height/resolution;
  int base_witdh_cells = base_witdh/resolution;

	for (int row = 0; row < base_height_cells; ++row) {
        for (int col = 0; col < base_witdh_cells; ++col) {
			if (grid[dist_x_cells + col + witdh*(dist_y_cells + row)] > 99){
               return true;
			}
		}
	}
	return false;

}

bool grid_check_collision_cpu2(int* grid, float resolution, int height, int witdh, double base_height, double base_witdh, double dist_x, double dist_y) {

	// returns if there is collision (true) or not (false)
	int dist_x_cells = dist_x/resolution + witdh/2;
	int dist_y_cells = dist_y/resolution + height/2;

  int base_height_cells = base_height/resolution;
  int base_witdh_cells = base_witdh/resolution;

	for (int row = 0; row < base_height_cells; ++row) {
        for (int col = 0; col < base_witdh_cells; ++col) {
			if (grid[dist_x_cells + col + witdh*(dist_y_cells + row)] > 99){
               return true;
			}
		}
	}
	return false;

}

bool debug_grid_check_collision_cpu(int *grid, float resolution, int height, int witdh, double base_height, double base_witdh, double dist_x, double dist_y){
  // returns if there is collision (true) or not (false)
	int dist_x_cells = dist_x/resolution + witdh/2;
	int dist_y_cells = dist_y/resolution + height/2;

  int base_height_cells = base_height/resolution;
  int base_witdh_cells = base_witdh/resolution;
  int half_base_height_cells = 0.5*base_height_cells;
  int half_base_witdh_cells = 0.5*base_witdh_cells;

	for (int row = 0; row < base_witdh_cells; ++row) {
        for (int col = 0; col < base_height_cells; ++col) {            
			      if (grid[dist_x_cells + col - half_base_height_cells  + witdh*(dist_y_cells + row - half_base_witdh_cells)] > 99){
                  return true;
			      }
            grid[dist_x_cells + col - half_base_height_cells  + witdh*(dist_y_cells + row - half_base_witdh_cells)] = 30;
		}
	}
	return false;

}

bool debug_grid_collision_rbkairos(grid_struct occupancy_grid, double base_height, double base_witdh, double dist_x, double dist_y){
  // returns if there is collision (true) or not (false)
	int dist_x_cells = round((dist_x-0.08)/occupancy_grid.map_resolution) + occupancy_grid.map_width/2; //0.08 is added because the base is not symmetric in x, we need more space in the back for cables
	int dist_y_cells = round(dist_y/occupancy_grid.map_resolution) + occupancy_grid.map_height/2;

  int base_height_cells = round(base_height/occupancy_grid.map_resolution);
  int base_witdh_cells = round(base_witdh/occupancy_grid.map_resolution);
  int half_base_height_cells = round(0.5*base_height_cells);
  int half_base_witdh_cells = round(0.5*base_witdh_cells);

	for (int row = 0; row < base_witdh_cells; ++row) {
        for (int col = 0; col < base_height_cells; ++col) {            
			      if (occupancy_grid.map_data[dist_x_cells + col - half_base_height_cells  + occupancy_grid.map_width*(dist_y_cells + row - half_base_witdh_cells)] > 99){
                  return true;
			      }
            occupancy_grid.map_data[dist_x_cells + col - half_base_height_cells  + occupancy_grid.map_width*(dist_y_cells + row - half_base_witdh_cells)] = 30;
		}
	}
	return false;
}

bool grid_collision_rbkairos(grid_struct occupancy_grid, double base_height, double base_witdh, double dist_x, double dist_y){
  // returns if there is collision (true) or not (false)
	int dist_x_cells = round((dist_x-0.08)/occupancy_grid.map_resolution) + occupancy_grid.map_width/2; //0.08 is added because the base is not symmetric in x, we need more space in the back for cables
	int dist_y_cells = round(dist_y/occupancy_grid.map_resolution) + occupancy_grid.map_height/2;

  int base_height_cells = round(base_height/occupancy_grid.map_resolution);
  int base_witdh_cells = round(base_witdh/occupancy_grid.map_resolution);
  int half_base_height_cells = round(0.5*base_height_cells);
  int half_base_witdh_cells = round(0.5*base_witdh_cells);

	for (int row = 0; row < base_witdh_cells; ++row) {
        for (int col = 0; col < base_height_cells; ++col) {            
			      if (occupancy_grid.map_data[dist_x_cells + col - half_base_height_cells  + occupancy_grid.map_width*(dist_y_cells + row - half_base_witdh_cells)] > 99){
                  return true;
			      }
		}
	}
	return false;
}

bool grid_collision_rbkairos(grid_struct occupancy_grid, double base_height, double base_witdh){
  double dist_x = 0.0;
  double dist_y = 0.0;
  // returns if there is collision (true) or not (false)
	int dist_x_cells = round((dist_x-0.08)/occupancy_grid.map_resolution) + occupancy_grid.map_width/2; //0.08 is added because the base is not symmetric in x, we need more space in the back for cables
	int dist_y_cells = round(dist_y/occupancy_grid.map_resolution) + occupancy_grid.map_height/2;

  int base_height_cells = round(base_height/occupancy_grid.map_resolution);
  int base_witdh_cells = round(base_witdh/occupancy_grid.map_resolution);
  int half_base_height_cells = round(0.5*base_height_cells);
  int half_base_witdh_cells = round(0.5*base_witdh_cells);

	for (int row = 0; row < base_witdh_cells; ++row) {
        for (int col = 0; col < base_height_cells; ++col) {            
			      if (occupancy_grid.map_data[dist_x_cells + col - half_base_height_cells  + occupancy_grid.map_width*(dist_y_cells + row - half_base_witdh_cells)] > 99){
                  return true;
			      }
		}
	}
	return false;
}
