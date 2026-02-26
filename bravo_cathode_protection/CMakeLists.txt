cmake_minimum_required(VERSION 3.8)
project(bravo_cathode_protection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(general_libs_unite REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(urdf REQUIRED)
find_package(nlohmann_json REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    Eigen3
    visualization_msgs
    general_libs_unite
    urdf
)


include_directories(include
                    ${PINOCCHIO_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    /usr/include/rapidjson)

install(
    DIRECTORY include/
    DESTINATION include)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
  
add_library(bravo_handler_v2 SHARED include/bravo_cathode_protection/bravo_cpp/bravo_handler/bravo_handler_v2.cpp
                                  include/bravo_cathode_protection/bravo_cpp/bravo_io_v2/bravo_udp_v2.cpp
                                  include/bravo_cathode_protection/bravo_cpp/bravo_io_v2/bravo_diagnosis.cpp
                                  include/bravo_cathode_protection/bravo_cpp/bravo_io_v2/bravo_req_paquets.cpp
                                  include/bravo_cathode_protection/bravo_cpp/bravo_io_v2/bravo_process_fdb.cpp
                                  include/bravo_cathode_protection/bravo_cpp/bravo_io_v2/bplprotocol_unite.cpp
                                  include/bravo_cathode_protection/bravo_cpp/utils/utils.cpp
                                  include/bravo_cathode_protection/bravo_cpp/utils/bravo_logger.cpp)
ament_target_dependencies(bravo_handler_v2 rclcpp std_msgs sensor_msgs urdf Eigen3 visualization_msgs general_libs_unite)
ament_export_libraries(bravo_handler_v2)
ament_export_include_directories(include)

install(TARGETS bravo_handler_v2
        EXPORT export_bravo_handler_v2
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

# add_library(model_handler SHARED include/bravo_cathode_protection/bravo_cpp/force_estimator/model_handler.cpp)
# ament_export_libraries(model_handler)
# ament_export_include_directories(include)
# install(TARGETS model_handler
#         EXPORT model_handler
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib
#         RUNTIME DESTINATION bin)

# add_library(estimator SHARED include/bravo_cathode_protection/bravo_cpp/force_estimator/estimator.cpp)
# ament_target_dependencies(estimator geometry_msgs ament_index_cpp)
# ament_export_libraries(estimator)
# ament_export_include_directories(include)
# install(TARGETS estimator
#         EXPORT estimator
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib
#         RUNTIME DESTINATION bin)

# add_executable(joystick_bravo_real_diff  src/unittests/joystick_bravo_real_diff.cpp)
# ament_target_dependencies(joystick_bravo_real_diff rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(joystick_bravo_real_diff pinocchio::pinocchio bravo_handler general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(joint_CurrentFdb_FT_joystick_dataCollection  src/unittests/joint_CurrentFdb_FT_joystick_dataCollection.cpp)
# ament_target_dependencies(joint_CurrentFdb_FT_joystick_dataCollection rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(joint_CurrentFdb_FT_joystick_dataCollection pinocchio::pinocchio bravo_handler general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(joystick_bravo_sim_diff  src/unittests/joystick_bravo_sim_diff.cpp)
# ament_target_dependencies(joystick_bravo_sim_diff rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(joystick_bravo_sim_diff pinocchio::pinocchio bravo_handler general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(joystick_bravo_real_force_control  src/unittests/joystick_bravo_real_force_control.cpp)
# ament_target_dependencies(joystick_bravo_real_force_control rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(joystick_bravo_real_force_control pinocchio::pinocchio bravo_handler general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(force_control_goHome  src/unittests/force_control_goHome.cpp)
# ament_target_dependencies(force_control_goHome rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(force_control_goHome pinocchio::pinocchio bravo_handler general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(force_control_compliance_goHome  src/unittests/force_control_compliance_goHome.cpp)
# ament_target_dependencies(force_control_compliance_goHome rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(force_control_compliance_goHome pinocchio::pinocchio bravo_handler general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(dof_joint_current_control  src/unittests/dof_joint_current_control.cpp)
# ament_target_dependencies(dof_joint_current_control rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(dof_joint_current_control pinocchio::pinocchio bravo5_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(compliance_whole_body_p_gain_spring  src/unittests/compliance/compliance_whole_body_p_gain_spring.cpp)
# ament_target_dependencies(compliance_whole_body_p_gain_spring rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(compliance_whole_body_p_gain_spring pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(compliance_3D_stiffness_control  src/unittests/compliance/compliance_3D_stiffness_control.cpp)
# ament_target_dependencies(compliance_3D_stiffness_control rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(compliance_3D_stiffness_control pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(compliance_diff_kin_loop  src/unittests/compliance/compliance_diff_kin_loop.cpp)
# ament_target_dependencies(compliance_whole_body_p_gain rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(compliance_diff_kin_loop pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(compliance_stiction_diff_kin_loop  src/unittests/compliance/compliance_stiction_diff_kin_loop.cpp)
# ament_target_dependencies(compliance_stiction_diff_kin_loop rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(compliance_stiction_diff_kin_loop pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(compliance_stiction_diff_kin_loop_pd_rnea src/unittests/compliance/compliance_stiction_diff_kin_loop_pd_rnea.cpp)
# ament_target_dependencies(compliance_stiction_diff_kin_loop_pd_rnea rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(compliance_stiction_diff_kin_loop_pd_rnea pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(compliance_stiction_diff_kin_loop_pd src/unittests/compliance/compliance_stiction_diff_kin_loop_pd.cpp)
# ament_target_dependencies(compliance_stiction_diff_kin_loop_pd rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(compliance_stiction_diff_kin_loop_pd pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(diff_kin_teleop src/unittests/joint_position_control/diff_kin_teleop.cpp)
# ament_target_dependencies(diff_kin_teleop rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(diff_kin_teleop pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(diff_kin_loop src/unittests/joint_position_control/diff_kin_loop.cpp)
# ament_target_dependencies(diff_kin_loop rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(diff_kin_loop pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(gravity_compensation src/unittests/compliance/gravity_compensation.cpp)
# ament_target_dependencies(gravity_compensation rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(gravity_compensation pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(gravity_compensation_and_collaborative src/unittests/compliance/gravity_compensation_and_collaborative.cpp)
# ament_target_dependencies(gravity_compensation_and_collaborative rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(gravity_compensation_and_collaborative pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(gravity_compensation_and_stiction_stribeck src/unittests/compliance/gravity_compensation_and_stiction_stribeck.cpp)
# ament_target_dependencies(gravity_compensation_and_stiction_stribeck rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(gravity_compensation_and_stiction_stribeck pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(gravity_compensation_and_stiction_stribeck_smooth src/unittests/compliance/gravity_compensation_and_stiction_stribeck_smooth.cpp)
# ament_target_dependencies(gravity_compensation_and_stiction_stribeck_smooth rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(gravity_compensation_and_stiction_stribeck_smooth pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(goToStore_joint_current src/unittests/compliance/goToStore_joint_current.cpp)
# ament_target_dependencies(goToStore_joint_current rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(goToStore_joint_current pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(stiction_identification  src/unittests/compliance/stiction_identification.cpp)
# ament_target_dependencies(stiction_identification rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(stiction_identification pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(compliance_diff_kin_loop_force_control  src/unittests/compliance_diff_kin_loop_force_control.cpp)
# ament_target_dependencies(compliance_diff_kin_loop_force_control rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(compliance_diff_kin_loop_force_control pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(external_force_estimation  src/unittests/compliance/external_force_estimation.cpp)
# ament_target_dependencies(external_force_estimation rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(external_force_estimation estimator "${cpp_typesupport_target}" yaml-cpp model_handler pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(force_control_goHome_LocalTwist               src/unittests/force_control_goHome_LocalTwist.cpp)
# ament_target_dependencies(force_control_goHome_LocalTwist    rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(force_control_goHome_LocalTwist        pinocchio::pinocchio bravo_handler general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(goToStore  src/unittests/joint_position_control/goToStore.cpp)
# ament_target_dependencies(goToStore rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(goToStore pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(diff_kin_loop  src/unittests/joint_position_control/diff_kin_loop.cpp)
# ament_target_dependencies(diff_kin_loop rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(diff_kin_loop pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(task_stiffness_control  src/unittests/torque_based_interaction/task_stiffness_control.cpp)
# ament_target_dependencies(task_stiffness_control rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(task_stiffness_control pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(task_stiffness_fixed_axis_control  src/unittests/torque_based_interaction/task_stiffness_fixed_axis_control.cpp)
# ament_target_dependencies(task_stiffness_fixed_axis_control rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(task_stiffness_fixed_axis_control pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(task_stiffness_fixed_axis_control_moving_forward  src/unittests/torque_based_interaction/task_stiffness_fixed_axis_control_moving_forward.cpp)
# ament_target_dependencies(task_stiffness_fixed_axis_control_moving_forward rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(task_stiffness_fixed_axis_control_moving_forward pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(task_stiffness_fixed_axis_control_moving_forward_loop  src/unittests/torque_based_interaction/task_stiffness_fixed_axis_control_moving_forward_loop.cpp)
# ament_target_dependencies(task_stiffness_fixed_axis_control_moving_forward_loop rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(task_stiffness_fixed_axis_control_moving_forward_loop pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(task_space_impedance_control_fixed_axis  src/unittests/torque_based_interaction/task_space_impedance_control_fixed_axis.cpp)
# ament_target_dependencies(task_space_impedance_control_fixed_axis rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(task_space_impedance_control_fixed_axis pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(program_cp_teleop_arm  src/unittests/torque_based_interaction/program_cp_teleop_arm.cpp)
# ament_target_dependencies(program_cp_teleop_arm rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(program_cp_teleop_arm pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(program_cp_teleop_arm_cyberov_debug  src/unittests/torque_based_interaction/program_cp_teleop_arm_cyberov_debug.cpp)
# ament_target_dependencies(program_cp_teleop_arm_cyberov_debug rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(program_cp_teleop_arm_cyberov_debug pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(program_cp_teleop_arm_cyberov_debug_2  src/unittests/torque_based_interaction/program_cp_teleop_arm_cyberov_debug_2.cpp)
# ament_target_dependencies(program_cp_teleop_arm_cyberov_debug_2 rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(program_cp_teleop_arm_cyberov_debug_2 pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(program_cp_teleop_arm_cyberov_continuous  src/unittests/torque_based_interaction/program_cp_teleop_arm_cyberov_continuous.cpp)
# ament_target_dependencies(program_cp_teleop_arm_cyberov_continuous rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(program_cp_teleop_arm_cyberov_continuous pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(program_cp_teleop_arm_cyberov_italian  src/unittests/torque_based_interaction/program_cp_teleop_arm_cyberov_italian.cpp)
# ament_target_dependencies(program_cp_teleop_arm_cyberov_italian rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(program_cp_teleop_arm_cyberov_italian pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(go_to_conf  src/unittests/software_checking/go_to_conf.cpp)
# ament_target_dependencies(go_to_conf rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(go_to_conf pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(test_rapidjson src/unittests/software_checking/test_rapidjson.cpp)
# target_include_directories(test_rapidjson PRIVATE ${RAPIDJSON_INCLUDE_DIRS})

# add_executable(test_rapidjson_file src/unittests/software_checking/test_rapidjson_file.cpp)
# target_include_directories(test_rapidjson_file PRIVATE ${RAPIDJSON_INCLUDE_DIRS})

# add_executable(task_stiffness_mobile_axis_control  src/unittests/torque_based_interaction/task_stiffness_mobile_axis_control.cpp)
# ament_target_dependencies(task_stiffness_mobile_axis_control rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(task_stiffness_mobile_axis_control pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(check_airbus  src/unittests/hardware_checking/check_airbus.cpp)
# ament_target_dependencies(check_airbus rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(check_airbus pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(program_cp_underwater            src/demos/program_cp_underwater.cpp)
# ament_target_dependencies(program_cp_underwater rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(program_cp_underwater     pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(program_cp_improved_1            src/demos/program_cp_improved.cpp)
# ament_target_dependencies(program_cp_improved_1 rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(program_cp_improved_1     pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(program_cp_improved_2            src/demos/program_cp_improved_2.cpp)
# ament_target_dependencies(program_cp_improved_2 rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(program_cp_improved_2     pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

add_executable(bravo5_gravity_and_joint_friction_identification            src/demos_v2/bravo5_gravity_and_joint_friction_identification.cpp)
ament_target_dependencies(bravo5_gravity_and_joint_friction_identification rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
target_link_libraries(bravo5_gravity_and_joint_friction_identification     pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(bravo7_gravity_and_joint_friction            src/demos_v2/bravo7_gravity_and_joint_friction.cpp)
# ament_target_dependencies(bravo7_gravity_and_joint_friction rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(bravo7_gravity_and_joint_friction     pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(bravo5_diff_kinematics  src/demos_v2/bravo5_diff_kinematics.cpp)
# ament_target_dependencies(bravo5_diff_kinematics rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(bravo5_diff_kinematics pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(joystick_bravo5_sim_diff  src/unittests/joystick_bravo5_sim_diff.cpp)
# ament_target_dependencies(joystick_bravo5_sim_diff rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(joystick_bravo5_sim_diff pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(bravo_cathode_protection_compliance_2  src/demos_v2/bravo_cathode_protection_compliance_2.cpp)
# ament_target_dependencies(bravo_cathode_protection_compliance_2 rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(bravo_cathode_protection_compliance_2 pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(bravo_cathode_protection_whole_program  src/demos_v2/bravo_cathode_protection_whole_program.cpp)
# ament_target_dependencies(bravo_cathode_protection_whole_program rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(bravo_cathode_protection_whole_program pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(bravo_cathode_protection_whole_program_2  src/demos_v2/bravo_cathode_protection_whole_program_2.cpp)
# ament_target_dependencies(bravo_cathode_protection_whole_program_2 rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(bravo_cathode_protection_whole_program_2 pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(bravo_cathode_protection_whole_program_3  src/demos_v2/bravo_cathode_protection_whole_program_3.cpp)
# ament_target_dependencies(bravo_cathode_protection_whole_program_3 rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(bravo_cathode_protection_whole_program_3 pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

add_executable(bravo5_cp_whole_program_7  src/demos_v2/bravo5_cp_whole_program_7.cpp)
ament_target_dependencies(bravo5_cp_whole_program_7 rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
target_link_libraries(bravo5_cp_whole_program_7 pinocchio::pinocchio nlohmann_json::nlohmann_json bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

add_executable(bravo5_cp_fixed_axis  src/demos_v2/bravo5_cp_fixed_axis.cpp)
ament_target_dependencies(bravo5_cp_fixed_axis rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
target_link_libraries(bravo5_cp_fixed_axis pinocchio::pinocchio nlohmann_json::nlohmann_json bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(bravo7_cp_compliance            src/demos_v2/bravo7_cp_compliance.cpp)
# ament_target_dependencies(bravo7_cp_compliance rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(bravo7_cp_compliance     pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})


# add_executable(gravity_compensation_and_stiction_stribeck_smooth            src/demos/gravity_compensation_and_stiction_stribeck_smooth.cpp)
# ament_target_dependencies(gravity_compensation_and_stiction_stribeck_smooth rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(gravity_compensation_and_stiction_stribeck_smooth     pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(task_stiffness_fixed_axis_control_compliant_Z            src/demos/task_stiffness_fixed_axis_control_compliant_Z.cpp)
# ament_target_dependencies(task_stiffness_fixed_axis_control_compliant_Z rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(task_stiffness_fixed_axis_control_compliant_Z     pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})

# add_executable(task_stiffness_fixed_axis_control_compliant_all_axis            src/demos/task_stiffness_fixed_axis_control_compliant_all_axis.cpp)
# ament_target_dependencies(task_stiffness_fixed_axis_control_compliant_all_axis rclcpp std_msgs sensor_msgs general_libs_unite ${TEST_SOURCES} ${dependencies})
# target_link_libraries(task_stiffness_fixed_axis_control_compliant_all_axis     pinocchio::pinocchio bravo_handler_v2 general_libs_unite::airbus_joystick general_libs_unite::general_utils ${PINOCCHIO_LIBRARIES})


install(TARGETS
  #joystick_bravo_real_diff
  #joystick_bravo_sim_diff
  # diff_kin_loop
  #dof_joint_current_control
  #joint_CurrentFdb_FT_joystick_dataCollection
  # joystick_bravo_real_force_control
  #force_control_goHome
  #force_control_compliance_goHome
  # force_control_goHome_LocalTwist
  # compliance_whole_body_p_gain_spring
  #goToStore_joint_current
  # task_stiffness_fixed_axis_control
  # task_stiffness_mobile_axis_control
  #task_stiffness_fixed_axis_control_moving_forward
  #task_stiffness_fixed_axis_control_moving_forward_loop
  #task_space_impedance_control_fixed_axis
  #program_cp_teleop_arm
  # program_cp_teleop_arm_cyberov_debug
  # program_cp_teleop_arm_cyberov_debug_2
  # program_cp_teleop_arm_cyberov_continuous
  # program_cp_teleop_arm_cyberov_italian
  # test_rapidjson
  # test_rapidjson_file
  # go_to_conf
  # task_stiffness_control
  # compliance_3D_stiffness_control
  # compliance_diff_kin_loop 
  #stiction_identification
  #compliance_diff_kin_loop_force_control
  # compliance_stiction_diff_kin_loop_pd_rnea
  # # compliance_stiction_diff_kin_loop_pd
  # # compliance_stiction_diff_kin_loop
  # gravity_compensation
  #diff_kin_teleop
  #diff_kin_loop
  # gravity_compensation_and_collaborative
  # gravity_compensation_and_stiction_stribeck
  # gravity_compensation_and_stiction_stribeck_smooth
  # external_force_estimation
  #check_airbus
  # goToStore
  #program_cp_underwater
  bravo5_gravity_and_joint_friction_identification
  #bravo7_gravity_and_joint_friction
  # joystick_bravo5_sim_diff
  # bravo5_diff_kinematics
  # bravo_cathode_protection_compliance_2
  # bravo_cathode_protection_whole_program
  # bravo_cathode_protection_whole_program_2
  # bravo_cathode_protection_whole_program_3
  bravo5_cp_whole_program_7
  bravo5_cp_fixed_axis
  #bravo7_cp_compliance
  #gravity_compensation_and_stiction_stribeck_smooth
  # program_cp_improved_1
  #program_cp_improved_2
  # task_stiffness_fixed_axis_control_compliant_Z
  # task_stiffness_fixed_axis_control_compliant_all_axis
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
