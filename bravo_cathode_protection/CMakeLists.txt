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
# Workaround for CMake 3.22 FindBoost not mapping python310+ component headers.
# pinocchio/eigenpy can request python310 on ROS Humble, triggering a warning.
if(CMAKE_VERSION VERSION_LESS "3.24")
  foreach(_py_comp 310 311 312 313)
    set("_Boost_PYTHON${_py_comp}_HEADERS" "boost/python.hpp")
  endforeach()
endif()
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


set(ros_dependencies
    rclcpp
    sensor_msgs
    Eigen3
)

install(
    DIRECTORY include/
    DESTINATION include)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY rviz
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
ament_target_dependencies(bravo_handler_v2 Eigen3)
target_include_directories(bravo_handler_v2 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(bravo_handler_v2 PUBLIC pinocchio::pinocchio)
ament_export_libraries(bravo_handler_v2)
ament_export_include_directories(include)

install(TARGETS bravo_handler_v2
        EXPORT export_bravo_handler_v2
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

add_library(kinodynamics_manipulator SHARED include/bravo_cathode_protection/bravo_cpp/serial_manipulator/kinodynamics_manipulator.cpp)
ament_target_dependencies(kinodynamics_manipulator Eigen3)
target_link_libraries(kinodynamics_manipulator PUBLIC pinocchio::pinocchio)
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
ament_target_dependencies(stiffness_control_position Eigen3)
target_link_libraries(stiffness_control_position PUBLIC yaml-cpp)
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
ament_target_dependencies(bravo5_gravity_and_joint_friction_identification ${ros_dependencies})
target_link_libraries(bravo5_gravity_and_joint_friction_identification     yaml-cpp bravo_handler_v2 airbus_joystick)

add_executable(bravo5_cp_fixed_axis  src/bravo5_cp_fixed_axis.cpp)
ament_target_dependencies(bravo5_cp_fixed_axis ${ros_dependencies})
target_link_libraries(bravo5_cp_fixed_axis yaml-cpp bravo_handler_v2 airbus_joystick stiffness_control_position)

add_executable(bravo5_cp_mobile_axis_hybrid  src/bravo5_cp_mobile_axis_hybrid.cpp)
ament_target_dependencies(bravo5_cp_mobile_axis_hybrid ${ros_dependencies})
target_link_libraries(bravo5_cp_mobile_axis_hybrid yaml-cpp bravo_handler_v2 airbus_joystick stiffness_control_position)

add_executable(bravo5_cp_mobile_axis_hybrid2  src/bravo5_cp_mobile_axis_hybrid2.cpp)
ament_target_dependencies(bravo5_cp_mobile_axis_hybrid2 ${ros_dependencies})
target_link_libraries(bravo5_cp_mobile_axis_hybrid2 yaml-cpp bravo_handler_v2 airbus_joystick stiffness_control_position)

add_executable(bravo5_cp_mobile_axis  src/bravo5_cp_mobile_axis.cpp)
ament_target_dependencies(bravo5_cp_mobile_axis ${ros_dependencies})
target_link_libraries(bravo5_cp_mobile_axis yaml-cpp bravo_handler_v2 airbus_joystick stiffness_control_position)

install(TARGETS
  bravo5_gravity_and_joint_friction_identification
  bravo5_cp_fixed_axis
  bravo5_cp_mobile_axis
  bravo5_cp_mobile_axis_hybrid
  bravo5_cp_mobile_axis_hybrid2
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(
  export_airbus_joystick
  export_bravo_handler_v2
  export_kinodynamics_manipulator
  export_stiffness_control_position
  HAS_LIBRARY_TARGET
)
ament_export_dependencies(rclcpp sensor_msgs Eigen3 pinocchio yaml-cpp)


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
