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
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)


set(dependencies
    rclcpp
    sensor_msgs
    Eigen3
)


include_directories(include
                    ${PINOCCHIO_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

install(
    DIRECTORY include/
    DESTINATION include)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )

install(
    DIRECTORY urdf meshes
    DESTINATION share/${PROJECT_NAME}
  )
  
install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME})

add_library(airbus_joystick SHARED include/bravo_cathode_protection/bravo_cpp/joysticks/airbus_joystick.cpp)
ament_target_dependencies(airbus_joystick rclcpp sensor_msgs)
target_include_directories(airbus_joystick PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install the library
install(TARGETS airbus_joystick
        EXPORT export_airbus_joystick
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


add_library(bravo_handler_v2 SHARED include/bravo_cathode_protection/bravo_cpp/bravo_handler/bravo_handler_v2.cpp
                                  include/bravo_cathode_protection/bravo_cpp/bravo_io_v2/bravo_udp_v2.cpp
                                  include/bravo_cathode_protection/bravo_cpp/bravo_io_v2/bravo_diagnosis.cpp
                                  include/bravo_cathode_protection/bravo_cpp/bravo_io_v2/bravo_req_paquets.cpp
                                  include/bravo_cathode_protection/bravo_cpp/bravo_io_v2/bravo_process_fdb.cpp
                                  include/bravo_cathode_protection/bravo_cpp/bravo_io_v2/bplprotocol_unite.cpp
                                  include/bravo_cathode_protection/bravo_cpp/utils/utils.cpp
                                  include/bravo_cathode_protection/bravo_cpp/utils/bravo_logger.cpp)
ament_target_dependencies(bravo_handler_v2 Eigen3 )
ament_export_libraries(bravo_handler_v2)
ament_export_include_directories(include)

install(TARGETS bravo_handler_v2
        EXPORT export_bravo_handler_v2
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

add_library(kinodynamics_manipulator SHARED include/bravo_cathode_protection/bravo_cpp/serial_manipulator/kinodynamics_manipulator.cpp)
target_link_libraries(kinodynamics_manipulator )
target_include_directories(kinodynamics_manipulator PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)

# Install the library
install(TARGETS kinodynamics_manipulator
        EXPORT export_kinodynamics_manipulator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(stiffness_control_position SHARED include/bravo_cathode_protection/bravo_cpp/interaction/stiffness_control_position.cpp)
target_link_libraries(stiffness_control_position)
target_include_directories(stiffness_control_position PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)

# Install the library
install(TARGETS stiffness_control_position
        EXPORT  export_stiffness_control_position
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_executable(bravo5_gravity_and_joint_friction_identification            src/bravo5_gravity_and_joint_friction_identification.cpp)
ament_target_dependencies(bravo5_gravity_and_joint_friction_identification ${dependencies})
target_link_libraries(bravo5_gravity_and_joint_friction_identification     pinocchio::pinocchio bravo_handler_v2 airbus_joystick ${PINOCCHIO_LIBRARIES})

add_executable(bravo5_cp_fixed_axis_switch  src/bravo5_cp_fixed_axis_switch.cpp)
ament_target_dependencies(bravo5_cp_fixed_axis_switch ${dependencies})
target_link_libraries(bravo5_cp_fixed_axis_switch pinocchio::pinocchio nlohmann_json::nlohmann_json bravo_handler_v2 airbus_joystick stiffness_control_position ${PINOCCHIO_LIBRARIES})

add_executable(bravo5_cp_mobile_axis_switch  src/bravo5_cp_mobile_axis_switch.cpp)
ament_target_dependencies(bravo5_cp_mobile_axis_switch ${dependencies})
target_link_libraries(bravo5_cp_mobile_axis_switch pinocchio::pinocchio nlohmann_json::nlohmann_json bravo_handler_v2 airbus_joystick stiffness_control_position ${PINOCCHIO_LIBRARIES})

install(TARGETS
  bravo5_gravity_and_joint_friction_identification
  bravo5_cp_fixed_axis_switch
  bravo5_cp_mobile_axis_switch
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
