cmake_minimum_required(VERSION 3.8)
project(general_libs_unite)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
)

include_directories(include
                    ${EIGEN3_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${PINOCCHIO_INCLUDE_DIRS})

install(
        DIRECTORY include/
        DESTINATION include)

# Library sources
add_library(general_utils SHARED include/general_libs_unite/general_utils/general_utils.cpp)
ament_target_dependencies(general_utils rclcpp)
ament_export_libraries(general_utils)
ament_export_include_directories(include)
target_include_directories(general_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install the library
install(TARGETS general_utils
        EXPORT export_general_utils
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(collision_primitives SHARED include/general_libs_unite/collision/collision_primitives.cpp)
ament_target_dependencies(collision_primitives rclcpp visualization_msgs)
target_include_directories(collision_primitives PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install the library
install(TARGETS collision_primitives
        EXPORT export_collision_primitives
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(airbus_joystick SHARED include/general_libs_unite/joysticks/airbus_joystick.cpp)
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

add_library(null_space_projection SHARED include/general_libs_unite/whole_body_control/null_space_projection.cpp)
ament_target_dependencies(null_space_projection rclcpp visualization_msgs)
target_link_libraries(null_space_projection pinocchio::pinocchio ${PINOCCHIO_LIBRARIES})
target_include_directories(null_space_projection PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)

# Install the library
install(TARGETS null_space_projection
        EXPORT export_null_space_projection
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(vel_smoothing_filter SHARED include/general_libs_unite/whole_body_control/vel_smoothing_filter.cpp)
target_link_libraries(vel_smoothing_filter )
target_include_directories(vel_smoothing_filter PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)

# Install the library
install(TARGETS vel_smoothing_filter
        EXPORT export_vel_smoothing_filter
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(kinodynamics_manipulator SHARED include/general_libs_unite/serial_manipulator/kinodynamics_manipulator.cpp)
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

add_library(diff_kinematics SHARED include/general_libs_unite/serial_manipulator/diff_kinematics.cpp)
target_link_libraries(diff_kinematics)
target_include_directories(diff_kinematics PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)

# Install the library
install(TARGETS diff_kinematics
        EXPORT export_diff_kinematics
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(interaction_control SHARED include/general_libs_unite/interaction/interaction_control.cpp)
target_link_libraries(interaction_control)
target_include_directories(interaction_control PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)

# Install the library
install(TARGETS interaction_control
        EXPORT export_interaction_control
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(stiffness_control SHARED include/general_libs_unite/interaction/stiffness_control.cpp)
target_link_libraries(stiffness_control)
target_include_directories(stiffness_control PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)

# Install the library
install(TARGETS stiffness_control
        EXPORT export_stiffness_control
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(stiffness_control_position SHARED include/general_libs_unite/interaction/stiffness_control_position.cpp)
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

# Export the library
ament_export_targets(export_airbus_joystick export_null_space_projection export_general_utils export_collision_primitives export_stiffness_control_position
                     export_kinodynamics_manipulator export_vel_smoothing_filter export_diff_kinematics export_interaction_control export_stiffness_control HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs)
ament_package()

