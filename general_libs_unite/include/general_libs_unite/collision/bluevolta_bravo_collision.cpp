/**
 *    @file  bluevolta_bravo_collision.cpp
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
#include "uvm_falcon_bravo/collision/bluevolta_bravo_collision.h"

Matrix<double,4,4> bluevolta_bravo_collision::AH2(int n, Matrix<double, 6, 1> joint){ // WORKING !!!
    Matrix<double,4,4> T_a;
    T_a << 1,0,0,0,
           0,1,0,0,
           0,0,1,0,
           0,0,0,1;
    T_a(0,3) = a[0,n-1];
    
    Matrix<double,4,4> T_d;
    T_d << 1,0,0,0,
          0,1,0,0,
          0,0,1,0,
          0,0,0,1;
    T_d(2,3) = d[0,n-1];

    Matrix<double,4,4> Rzt;
    Rzt << cos(joint(n-1)), -sin(joint(n-1)), 0, 0,\
           sin(joint(n-1)), cos(joint(n-1)), 0, 0, \
           0, 0, 1, 0,\
           0, 0, 0, 1;
    
    Matrix<double,4,4> Rxa;
    Rxa << 1, 0, 0, 0,\
           0, cos(alph[n-1]), -sin(alph[n-1]), 0, \
           0, sin(alph[n-1]), cos(alph[n-1]), 0,\
           0, 0, 0, 1;
    
    Matrix<double,4,4> A_i;
    A_i = T_d * Rzt * T_a * Rxa;
    return A_i;
}



tuple<Matrix4d, Matrix4d, Matrix4d, Matrix4d, Matrix4d, Matrix4d, Matrix4d> bluevolta_bravo_collision::fKine_collision_links(Matrix<double, 6, 1> joint){  // WORKING !!!

    Matrix4d T_footprint_base_link,T_footprint_Link0,  T_footprint_Link1, T_footprint_Link2, T_footprint_Link3, T_footprint_Link4, T_footprint_Link5, T_footprint_Link6; 
    
    double thetaZ = -M_PI/2;
    Matrix4d T_bias, T_footprint_footprint, T_footprint_wrist;
    T_bias << cos(thetaZ), -sin(thetaZ), 0, 0,\
              sin(thetaZ), cos(thetaZ), 0, 0,\
              0, 0, 1, 0,\
              0, 0, 0, 1;

    Matrix4d A_1, A_2, A_3, A_4, A_5, A_6, T_06;
    A_1 = AH2(1, joint);
    A_2 = AH2(2, joint);
    A_3 = AH2(3, joint);
    A_4 = AH2(4, joint);
    A_5 = AH2(5, joint);
    A_6 = AH2(6, joint);

    T_footprint_base_link = T_footprint_armbase * T_bias; 
    T_footprint_Link0 = T_footprint_base_link * A_1;
    T_footprint_Link1 = T_footprint_Link0* A_2;
    T_footprint_Link2 = T_footprint_Link1* A_3;
    T_footprint_Link3 = T_footprint_Link2* A_4;
    T_footprint_Link4 = T_footprint_Link3* A_5;
    T_footprint_Link5 = T_footprint_Link4* A_6;

    Matrix4d T_footprint_AABB_base, T_footprint_base_capsule, T_footprint_capsule_Link1, T_footprint_capsule_Link2, T_footprint_capsule_Link3, T_footprint_capsule_Link4, T_footprint_capsule_Link5, T_footprint_capsule_Link6;
    //base transform
    T_footprint_AABB_base     = T_footprint_footprint*T_footprint_AABB_base_bias;
    //Base Link (shoulder)
    T_footprint_base_capsule  = T_footprint_Link0 * T_col_base_capsule_bias;
    // First Link 
    T_footprint_capsule_Link1 = T_footprint_Link1 * T_col_link1_capsule_bias;
    // Second Link
    T_footprint_capsule_Link2 = T_footprint_Link2 * T_col_link2_capsule_bias;
    // Third Link
    T_footprint_capsule_Link3 = T_footprint_Link3 * T_col_link3_capsule_bias;
    // Fourth Link
    T_footprint_capsule_Link4 = T_footprint_Link4 * T_col_link4_capsule_bias; 
    // Fifth Link
    T_footprint_capsule_Link5 = T_footprint_Link5 * T_col_link5_capsule_bias; 
    // Sixth Link
    T_footprint_capsule_Link6 = T_footprint_Link6 * T_col_linktool_capsule_bias; 


    return {T_footprint_base_capsule, T_footprint_capsule_Link1, T_footprint_capsule_Link2, T_footprint_capsule_Link3, T_footprint_capsule_Link4, T_footprint_capsule_Link5, T_footprint_capsule_Link6};
}

tuple<visualization_msgs::MarkerArray, double, bool> bluevolta_bravo_collision::self_collision( Matrix<double, 6, 1> arm_joint_states)
  {
    Matrix4d T_footprint_base_capsule, T_footprint_capsule_Link1, T_footprint_capsule_Link2, T_footprint_capsule_Link3, T_footprint_capsule_Link4, T_footprint_capsule_Link5, T_footprint_capsule_Link6;
    tie(T_footprint_base_capsule, T_footprint_capsule_Link1, T_footprint_capsule_Link2, T_footprint_capsule_Link3, T_footprint_capsule_Link4, T_footprint_capsule_Link5, T_footprint_capsule_Link6) = fKine_collision_links(arm_joint_states); 
    Vector3d position_base; position_base << 0, 0, 0.254;
    // BASE
    AABBbox base_AABB(0.77, 0.6, 0.52, position_base);
    // LINK 0
    capsule_swept_spheres link0_capsule(0.07, 0.1, 3, T_footprint_base_capsule);
    // LINK1
    capsule_swept_spheres link1_capsule(0.07, 0.45, 2, T_footprint_capsule_Link1);
    // LINK2
    capsule_swept_spheres link2_capsule(0.065, 0.35, 2, T_footprint_capsule_Link2);
    // LINK3
    Vector3d  positionLink3;
    positionLink3 << T_footprint_capsule_Link3(0,3), T_footprint_capsule_Link3(1,3), T_footprint_capsule_Link3(2,3);
    sphere link3_sphere(0.08, positionLink3);
    // LINK4
    Vector3d  positionLink4;
    positionLink4 << T_footprint_capsule_Link4(0,3), T_footprint_capsule_Link4(1,3), T_footprint_capsule_Link4(2,3);
    sphere link4_sphere(0.08, positionLink4);
    // TOOL
    Vector3d  positionTool;
    positionTool << T_footprint_capsule_Link5(0,3), T_footprint_capsule_Link5(1,3), T_footprint_capsule_Link5(2,3);
    sphere tool_sphere(0.08, positionTool);

    // COLLISION MATRIX
    /*
    1 CHECK  0 NO CHECK 2 NOT IMPORTANT CHECK
    BASE LINK0 LINK1 LINK2 LINK3 LINK4 TOOL
    0      0     1     1     1      1    1   BASE
    0      0     2     0     0      1   a 1   LINK0
    0      0     0     0     2      1    1   LINK1
    0      0     0     0     0      0    1   LINK2
    0      0     0     0     0      0    0   LINK3
    0      0     0     0     0      0    0   LINK4
    0      0     0     0     0      0    0   TOOL
    */
    // 10 CHECKS TO DO 
    //1 BASE--LINK0
    
    //2 BASE--LINK1
    bool collisionBaseLink1, distanceBaseLink1;
    tie(collisionBaseLink1, distanceBaseLink1) = capsule_AABB_sweept_sphere_volumes(link1_capsule, base_AABB);
    //3 BASE--LINK2
    bool collisionBaseLink2, distanceBaseLink2;
    tie(collisionBaseLink2, distanceBaseLink2) = capsule_AABB_sweept_sphere_volumes(link2_capsule, base_AABB);
    //4 BASE--LINK3
    bool collisionBaseLink3, distanceBaseLink3;
    tie(collisionBaseLink3, distanceBaseLink3) = sphere_AABB_cpp(link3_sphere, base_AABB);
    //5 BASE--LINK4
    bool collisionBaseLink4, distanceBaseLink4;
    tie(collisionBaseLink4, distanceBaseLink4) = sphere_AABB_cpp(link4_sphere, base_AABB);
    //6 BASE--TOOL
    //bool collisionBaseTool, distanceBaseTool;
    //tie(collisionBaseTool, distanceBaseTool) = sphere_AABB_cpp(tool_sphere, base_AABB);

    //7 LINK4--LINK0
    bool collisionLink4Link0, distanceLink4Link0;
    tie(collisionLink4Link0, distanceLink4Link0) = sphere_sweept_sphere_volumes(link0_capsule, link4_sphere);
    //8 TOOL--LINK0
    //bool collisionToolLink0, distanceToolLink0;
    //tie(collisionToolLink0, distanceToolLink0) = sphere_sweept_sphere_volumes(link0_capsule, tool_sphere);
    
    //9 LINK3--LINK1
    bool collisionLink3Link1, distanceLink3Link1;
    tie(collisionLink3Link1, distanceLink3Link1) = sphere_sweept_sphere_volumes(link1_capsule, link3_sphere);

    //9 LINK4--LINK1
    bool collisionLink4Link1, distanceLink4Link1;
    tie(collisionLink4Link1, distanceLink4Link1) = sphere_sweept_sphere_volumes(link1_capsule, link4_sphere);
    //10 TOOL--LINK1
    //bool collisionToolLink1, distanceToolLink1;
    //tie(collisionToolLink1, distanceToolLink1) = sphere_sweept_sphere_volumes(link1_capsule, tool_sphere);

    //11 TOOL--LINK2
    //bool collisionToolLink2, distanceToolLink2;
    //tie(collisionToolLink2, distanceToolLink2) = sphere_sweept_sphere_volumes(link2_capsule, tool_sphere);

    double distance = 0.0;
    bool collision = false;
    
    if(collisionBaseLink1 or collisionBaseLink2 or collisionBaseLink3 or collisionBaseLink4 or collisionLink4Link0 or \
       collisionLink3Link1 or collisionLink4Link1){
      collision = true; 
    }
    

    visualization_msgs::MarkerArray collision_markers;
    collision_markers = visualize_self_collision(base_AABB, link0_capsule, link1_capsule, link2_capsule, link3_sphere, link4_sphere, tool_sphere);
      
    return {collision_markers, distance, collision};
  }

visualization_msgs::MarkerArray bluevolta_bravo_collision::visualize_self_collision(AABBbox base_AABB, capsule_swept_spheres link0_capsule, capsule_swept_spheres link1_capsule,\
          capsule_swept_spheres link2_capsule, sphere link3_sphere, sphere link4_sphere, sphere tool_sphere)
  {
    visualization_msgs::MarkerArray collision_markers;
    std::string frame; frame = "real_base_footprint";
    //BASE
      visualization_msgs::Marker box;           
      box.header.frame_id = frame;
      box.type = visualization_msgs::Marker::CUBE;
      box.action = visualization_msgs::Marker::ADD;
      box.scale.x = base_AABB.x_length;
      box.scale.y = base_AABB.y_length;
      box.scale.z = base_AABB.z_length;
      box.color.a = 0.5;
      box.color.r = 0.0;
      box.color.g = 0.458;
      box.color.b = 0.616;
      box.pose.position.x = base_AABB.position(0);
      box.pose.position.y = base_AABB.position(1);
      box.pose.position.z = base_AABB.position(2);
      box.id = 0;
      collision_markers.markers.push_back(box);
    //LINK 0 
     for (int i=0; i<link0_capsule.number_spheres; i++)
        {
          double relative_distance = -link0_capsule.length/2 + link0_capsule.spheres_location[i];
          Matrix<double, 4, 1> position_origin, aux;
          aux << relative_distance, 0, 0, 1;    
          position_origin = link0_capsule.transform * aux;
          visualization_msgs::Marker sphere;           
          sphere.header.frame_id = frame;
          sphere.type = visualization_msgs::Marker::SPHERE;
          sphere.action = visualization_msgs::Marker::ADD;
          sphere.scale.x = link0_capsule.radius*2;
          sphere.scale.y = link0_capsule.radius*2;
          sphere.scale.z = link0_capsule.radius*2;
          sphere.color.a = 0.6;
          sphere.color.r = 0.0;
          sphere.color.g = 1.0;
          sphere.color.b = 1.0;
          sphere.pose.position.x = position_origin(0,0);
          sphere.pose.position.y = position_origin(1,0);
          sphere.pose.position.z = position_origin(2,0);
          sphere.id = i+200;
          collision_markers.markers.push_back(sphere);
      }  
    //LINK 1
     for (int i=0; i<link1_capsule.number_spheres; i++)
        {
          double relative_distance = -link1_capsule.length/2 + link1_capsule.spheres_location[i];
          Matrix<double, 4, 1> position_origin, aux;
          aux << relative_distance, 0, 0, 1;    
          position_origin = link1_capsule.transform * aux;
          visualization_msgs::Marker sphere;           
          sphere.header.frame_id = frame;
          sphere.type = visualization_msgs::Marker::SPHERE;
          sphere.action = visualization_msgs::Marker::ADD;
          sphere.scale.x = link1_capsule.radius*2;
          sphere.scale.y = link1_capsule.radius*2;
          sphere.scale.z = link1_capsule.radius*2;
          sphere.color.a = 0.6;
          sphere.color.r = 0.0;
          sphere.color.g = 1.0;
          sphere.color.b = 1.0;
          sphere.pose.position.x = position_origin(0,0);
          sphere.pose.position.y = position_origin(1,0);
          sphere.pose.position.z = position_origin(2,0);
          sphere.id = i+300;
          collision_markers.markers.push_back(sphere);
      }  
    //LINK 2
     for (int i=0; i<link2_capsule.number_spheres; i++)
        {
          double relative_distance = -link2_capsule.length/2 + link2_capsule.spheres_location[i];
          Matrix<double, 4, 1> position_origin, aux;
          aux << relative_distance, 0, 0, 1;    
          position_origin = link2_capsule.transform * aux;
          visualization_msgs::Marker sphere;           
          sphere.header.frame_id = frame;
          sphere.type = visualization_msgs::Marker::SPHERE;
          sphere.action = visualization_msgs::Marker::ADD;
          sphere.scale.x = link2_capsule.radius*2;
          sphere.scale.y = link2_capsule.radius*2;
          sphere.scale.z = link2_capsule.radius*2;
          sphere.color.a = 0.6;
          sphere.color.r = 0.0;
          sphere.color.g = 1.0;
          sphere.color.b = 1.0;
          sphere.pose.position.x = position_origin(0,0);
          sphere.pose.position.y = position_origin(1,0);
          sphere.pose.position.z = position_origin(2,0);
          sphere.id = i+400;
          collision_markers.markers.push_back(sphere);
      } 
      //LINK3
      visualization_msgs::Marker sphere3;           
      sphere3.header.frame_id = frame;
      sphere3.type = visualization_msgs::Marker::SPHERE;
      sphere3.action = visualization_msgs::Marker::ADD;
      sphere3.scale.x = link3_sphere.radius*2;
      sphere3.scale.y = link3_sphere.radius*2;
      sphere3.scale.z = link3_sphere.radius*2;
      sphere3.color.a = 0.6;
      sphere3.color.r = 1.0;
      sphere3.color.g = 0.0;
      sphere3.color.b = 0.0;
      sphere3.pose.position.x = link3_sphere.position(0);
      sphere3.pose.position.y = link3_sphere.position(1);
      sphere3.pose.position.z = link3_sphere.position(2);
      sphere3.id = 3;
      collision_markers.markers.push_back(sphere3);
      //LINK4
      visualization_msgs::Marker sphere4;           
      sphere4.header.frame_id = frame;
      sphere4.type = visualization_msgs::Marker::SPHERE;
      sphere4.action = visualization_msgs::Marker::ADD;
      sphere4.scale.x = link4_sphere.radius*2;
      sphere4.scale.y = link4_sphere.radius*2;
      sphere4.scale.z = link4_sphere.radius*2;
      sphere4.color.a = 0.6;
      sphere4.color.r = 1.0;
      sphere4.color.g = 0.0;
      sphere4.color.b = 0.0;
      sphere4.pose.position.x = link4_sphere.position(0);
      sphere4.pose.position.y = link4_sphere.position(1);
      sphere4.pose.position.z = link4_sphere.position(2);
      sphere4.id = 4;
      collision_markers.markers.push_back(sphere4);

      //TOOL
      visualization_msgs::Marker sphereTool;           
      sphereTool.header.frame_id = frame;
      sphereTool.type = visualization_msgs::Marker::SPHERE;
      sphereTool.action = visualization_msgs::Marker::ADD;
      sphereTool.scale.x = tool_sphere.radius*2;
      sphereTool.scale.y = tool_sphere.radius*2;
      sphereTool.scale.z = tool_sphere.radius*2;
      sphereTool.color.a = 0.6;
      sphereTool.color.r = 1.0;
      sphereTool.color.g = 0.0;
      sphereTool.color.b = 0.0;
      sphereTool.pose.position.x = tool_sphere.position(0);
      sphereTool.pose.position.y = tool_sphere.position(1);
      sphereTool.pose.position.z = tool_sphere.position(2);
      sphereTool.id = 5;
      collision_markers.markers.push_back(sphereTool);
      
    return collision_markers;
  }
 
  bool bluevolta_bravo_collision::self_collision_check_pinocchio(Eigen::VectorXd q){

    forwardKinematics(model_manipulator, data_manipulator, q);
    Matrix4d T_footprint_base_capsule, T_footprint_capsule_Link1, T_footprint_capsule_Link2, T_footprint_capsule_Link3, T_footprint_capsule_Link4, T_footprint_capsule_Link5, T_footprint_capsule_Link6;
    
    T_footprint_capsule_Link1 = data.oMi[0].homogeneous() * T_col_link1_capsule_bias;
    T_footprint_capsule_Link2 = data.oMi[1].homogeneous() * T_col_link2_capsule_bias;
    T_footprint_capsule_Link3 = data.oMi[2].homogeneous() * T_col_link3_capsule_bias;
    T_footprint_capsule_Link4 = data.oMi[3].homogeneous() * T_col_link4_capsule_bias;
    T_footprint_capsule_Link5 = data.oMi[4].homogeneous() * T_col_link5_capsule_bias;
    T_footprint_capsule_Link6 = data.oMi[5].homogeneous() * T_col_linktool_capsule_bias;
    
    // BASE
    Vector3d position_base; position_base << 0, 0, 0.254;
    AABBbox base_AABB(0.77, 0.6, 0.52, position_base);
    // LINK 0
    capsule_swept_spheres link0_capsule(0.07, 0.1, 3, T_footprint_base_capsule);
    // LINK1
    capsule_swept_spheres link1_capsule(0.07, 0.45, 2, T_footprint_capsule_Link1);
    // LINK2
    capsule_swept_spheres link2_capsule(0.065, 0.35, 2, T_footprint_capsule_Link2);
    // LINK3
    Vector3d  positionLink3;
    positionLink3 << T_footprint_capsule_Link3(0,3), T_footprint_capsule_Link3(1,3), T_footprint_capsule_Link3(2,3);
    sphere link3_sphere(0.08, positionLink3);
    // LINK4
    Vector3d  positionLink4;
    positionLink4 << T_footprint_capsule_Link4(0,3), T_footprint_capsule_Link4(1,3), T_footprint_capsule_Link4(2,3);
    sphere link4_sphere(0.08, positionLink4);
    // TOOL
    Vector3d  positionTool;
    positionTool << T_footprint_capsule_Link5(0,3), T_footprint_capsule_Link5(1,3), T_footprint_capsule_Link5(2,3);
    sphere tool_sphere(0.08, positionTool);

    // COLLISION MATRIX
    /*
    1 CHECK  0 NO CHECK 2 NOT IMPORTANT CHECK
    BASE LINK0 LINK1 LINK2 LINK3 LINK4 TOOL
    0      0     1     1     1      1    1   BASE
    0      0     2     0     0      1    1   LINK0
    0      0     0     0     2      1    1   LINK1
    0      0     0     0     0      0    1   LINK2
    0      0     0     0     0      0    0   LINK3
    0      0     0     0     0      0    0   LINK4
    0      0     0     0     0      0    0   TOOL
    */
    // 10 CHECKS TO DO 
    //1 BASE--LINK0
    
    //2 BASE--LINK1
    bool collisionBaseLink1, distanceBaseLink1;
    tie(collisionBaseLink1, distanceBaseLink1) = capsule_AABB_sweept_sphere_volumes(link1_capsule, base_AABB);
    //3 BASE--LINK2
    bool collisionBaseLink2, distanceBaseLink2;
    tie(collisionBaseLink2, distanceBaseLink2) = capsule_AABB_sweept_sphere_volumes(link2_capsule, base_AABB);
    //4 BASE--LINK3
    bool collisionBaseLink3, distanceBaseLink3;
    tie(collisionBaseLink3, distanceBaseLink3) = sphere_AABB_cpp(link3_sphere, base_AABB);
    //5 BASE--LINK4
    bool collisionBaseLink4, distanceBaseLink4;
    tie(collisionBaseLink4, distanceBaseLink4) = sphere_AABB_cpp(link4_sphere, base_AABB);
    //6 BASE--TOOL
    //bool collisionBaseTool, distanceBaseTool;
    //tie(collisionBaseTool, distanceBaseTool) = sphere_AABB_cpp(tool_sphere, base_AABB);

    //7 LINK4--LINK0
    bool collisionLink4Link0, distanceLink4Link0;
    tie(collisionLink4Link0, distanceLink4Link0) = sphere_sweept_sphere_volumes(link0_capsule, link4_sphere);
    //8 TOOL--LINK0
    //bool collisionToolLink0, distanceToolLink0;
    //tie(collisionToolLink0, distanceToolLink0) = sphere_sweept_sphere_volumes(link0_capsule, tool_sphere);
    
    //9 LINK3--LINK1
    bool collisionLink3Link1, distanceLink3Link1;
    tie(collisionLink3Link1, distanceLink3Link1) = sphere_sweept_sphere_volumes(link1_capsule, link3_sphere);

    //9 LINK4--LINK1
    bool collisionLink4Link1, distanceLink4Link1;
    tie(collisionLink4Link1, distanceLink4Link1) = sphere_sweept_sphere_volumes(link1_capsule, link4_sphere);
    //10 TOOL--LINK1
    //bool collisionToolLink1, distanceToolLink1;
    //tie(collisionToolLink1, distanceToolLink1) = sphere_sweept_sphere_volumes(link1_capsule, tool_sphere);

    //11 TOOL--LINK2
    //bool collisionToolLink2, distanceToolLink2;
    //tie(collisionToolLink2, distanceToolLink2) = sphere_sweept_sphere_volumes(link2_capsule, tool_sphere);

    double distance = 0.0;
    bool collision = false;
    
    if(collisionBaseLink1 or collisionBaseLink2 or collisionBaseLink3 or collisionBaseLink4 or collisionLink4Link0 or \
       collisionLink3Link1 or collisionLink4Link1){
      collision = true; 
    }
    
    return collision;
  }

 bool bluevolta_bravo_collision::self_collision_check( Matrix<double, 6, 1> arm_joint_states)
  {
    Matrix4d T_footprint_base_capsule, T_footprint_capsule_Link1, T_footprint_capsule_Link2, T_footprint_capsule_Link3, T_footprint_capsule_Link4, T_footprint_capsule_Link5, T_footprint_capsule_Link6;
    tie(T_footprint_base_capsule, T_footprint_capsule_Link1, T_footprint_capsule_Link2, T_footprint_capsule_Link3, T_footprint_capsule_Link4, T_footprint_capsule_Link5, T_footprint_capsule_Link6) = fKine_collision_links(arm_joint_states); 
    Vector3d position_base; position_base << 0, 0, 0.254;
    // BASE
    AABBbox base_AABB(0.77, 0.6, 0.52, position_base);
    // LINK 0
    capsule_swept_spheres link0_capsule(0.07, 0.1, 3, T_footprint_base_capsule);
    // LINK1
    capsule_swept_spheres link1_capsule(0.07, 0.45, 2, T_footprint_capsule_Link1);
    // LINK2
    capsule_swept_spheres link2_capsule(0.065, 0.35, 2, T_footprint_capsule_Link2);
    // LINK3
    Vector3d  positionLink3;
    positionLink3 << T_footprint_capsule_Link3(0,3), T_footprint_capsule_Link3(1,3), T_footprint_capsule_Link3(2,3);
    sphere link3_sphere(0.08, positionLink3);
    // LINK4
    Vector3d  positionLink4;
    positionLink4 << T_footprint_capsule_Link4(0,3), T_footprint_capsule_Link4(1,3), T_footprint_capsule_Link4(2,3);
    sphere link4_sphere(0.08, positionLink4);
    // TOOL
    Vector3d  positionTool;
    positionTool << T_footprint_capsule_Link5(0,3), T_footprint_capsule_Link5(1,3), T_footprint_capsule_Link5(2,3);
    sphere tool_sphere(0.08, positionTool);

    // COLLISION MATRIX
    /*
    1 CHECK  0 NO CHECK 2 NOT IMPORTANT CHECK
    BASE LINK0 LINK1 LINK2 LINK3 LINK4 TOOL
    0      0     1     1     1      1    1   BASE
    0      0     2     0     0      1    1   LINK0
    0      0     0     0     2      1    1   LINK1
    0      0     0     0     0      0    1   LINK2
    0      0     0     0     0      0    0   LINK3
    0      0     0     0     0      0    0   LINK4
    0      0     0     0     0      0    0   TOOL
    */
    // 10 CHECKS TO DO 
    //1 BASE--LINK0
    
    //2 BASE--LINK1
    bool collisionBaseLink1, distanceBaseLink1;
    tie(collisionBaseLink1, distanceBaseLink1) = capsule_AABB_sweept_sphere_volumes(link1_capsule, base_AABB);
    //3 BASE--LINK2
    bool collisionBaseLink2, distanceBaseLink2;
    tie(collisionBaseLink2, distanceBaseLink2) = capsule_AABB_sweept_sphere_volumes(link2_capsule, base_AABB);
    //4 BASE--LINK3
    bool collisionBaseLink3, distanceBaseLink3;
    tie(collisionBaseLink3, distanceBaseLink3) = sphere_AABB_cpp(link3_sphere, base_AABB);
    //5 BASE--LINK4
    bool collisionBaseLink4, distanceBaseLink4;
    tie(collisionBaseLink4, distanceBaseLink4) = sphere_AABB_cpp(link4_sphere, base_AABB);
    //6 BASE--TOOL
    //bool collisionBaseTool, distanceBaseTool;
    //tie(collisionBaseTool, distanceBaseTool) = sphere_AABB_cpp(tool_sphere, base_AABB);

    //7 LINK4--LINK0
    bool collisionLink4Link0, distanceLink4Link0;
    tie(collisionLink4Link0, distanceLink4Link0) = sphere_sweept_sphere_volumes(link0_capsule, link4_sphere);
    //8 TOOL--LINK0
    //bool collisionToolLink0, distanceToolLink0;
    //tie(collisionToolLink0, distanceToolLink0) = sphere_sweept_sphere_volumes(link0_capsule, tool_sphere);
    
    //9 LINK3--LINK1
    bool collisionLink3Link1, distanceLink3Link1;
    tie(collisionLink3Link1, distanceLink3Link1) = sphere_sweept_sphere_volumes(link1_capsule, link3_sphere);

    //9 LINK4--LINK1
    bool collisionLink4Link1, distanceLink4Link1;
    tie(collisionLink4Link1, distanceLink4Link1) = sphere_sweept_sphere_volumes(link1_capsule, link4_sphere);
    //10 TOOL--LINK1
    //bool collisionToolLink1, distanceToolLink1;
    //tie(collisionToolLink1, distanceToolLink1) = sphere_sweept_sphere_volumes(link1_capsule, tool_sphere);

    //11 TOOL--LINK2
    //bool collisionToolLink2, distanceToolLink2;
    //tie(collisionToolLink2, distanceToolLink2) = sphere_sweept_sphere_volumes(link2_capsule, tool_sphere);

    double distance = 0.0;
    bool collision = false;
    
    if(collisionBaseLink1 or collisionBaseLink2 or collisionBaseLink3 or collisionBaseLink4 or collisionLink4Link0 or \
       collisionLink3Link1 or collisionLink4Link1){
      collision = true; 
    }
    
    return collision;
  }


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

double SqDistPointAABB(Vector3d point_position, AABBbox box)
{
  Matrix<double,4,4> AABB_transform;
  AABB_transform << 1 ,0 ,0 ,box.position(0),  0 ,1 ,0 ,box.position(1),  0 ,0 ,1 ,box.position(2),    0 ,0 ,0 ,1;
  Matrix<double,3,1> b_min, b_max;
  Matrix<double,4,1> b_min_1;
  Matrix<double,4,1> dim_box, dim_box2;
  dim_box << -box.x_length/2, -box.y_length/2, -box.z_length/2, 1;
  dim_box2 << box.x_length/2, box.y_length/2, box.z_length/2, 1;
  b_min_1 = AABB_transform * dim_box;
  Matrix<double,4,1> b_max_1;
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




bool OBB_OBB_cpp(OBBbox box1, OBBbox box2)
{
  Vector3f A0(box1.transform(0,0), box1.transform(0,1), box1.transform(0,2)); 
  Vector3f A1(box1.transform(1,0), box1.transform(1,1), box1.transform(1,2));
  Vector3f A2(box1.transform(2,0), box1.transform(2,1), box1.transform(2,2));

  Vector3f B0(box2.transform(0,0), box2.transform(0,1), box2.transform(0,2)); 
  Vector3f B1(box2.transform(1,0), box2.transform(1,1), box2.transform(1,2));
  Vector3f B2(box2.transform(2,0), box2.transform(2,1), box2.transform(2,2));

  Vector3f positionA(box1.transform(0,3), box1.transform(1,3), box1.transform(2,3));
  Vector3f positionB(box2.transform(0,3), box2.transform(1,3), box2.transform(2,3));
  
  Vector3f D = positionB - positionA;

  double a0 = box1.x_length/2;
  double a1 = box1.y_length/2;
  double a2 = box1.z_length/2;

  double b0 = box2.x_length/2;
  double b1 = box2.y_length/2;
  double b2 = box2.z_length/2;

  Matrix<double,4,4> Rot_Conversion_A_B = box1.transform.inverse() * box2.transform; 

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
  Vector3f v1 = Rot_Conversion_A_B(1,0) * A2;
  Vector3f v2 = Rot_Conversion_A_B(2,0) * A1; 
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
  Matrix<double, 4, 1> position_origin;
  Matrix<double, 4, 1> origin;
  origin << -length_capsule/2, 0, 0, 1;
  position_origin = capsule1.transform * origin;
  vector<double> distances_array(capsule1.number_spheres);
  double minimum = 1000000;
  for( int i = 0; i<capsule1.number_spheres; i++)
  {
    double relative_distance = -length_capsule/2 + capsule1.spheres_location[i];
    Matrix<double, 4, 1> aux;
    aux << relative_distance, 0, 0, 1;
    origin = capsule1.transform * aux;
    Vector3d position_sphere;
    position_sphere << origin(0), origin(1), origin(2);
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
  Matrix<double, 4, 1> position_origin;
  Matrix<double, 4, 1> origin;
  origin << -length_capsule/2, 0, 0, 1;
  position_origin = capsule1.transform * origin;
  vector<double> distances_array(capsule1.number_spheres);
  double minimum = 1000000;
  for( int i = 0; i<capsule1.number_spheres; i++)
  {
    double relative_distance = -length_capsule/2 + capsule1.spheres_location[i];
    Matrix<double, 4, 1> aux;
    aux << relative_distance, 0, 0, 1;
    origin = capsule1.transform * aux;
    Vector3d position_sphere;
    position_sphere << origin(0), origin(1), origin(2);
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
 
