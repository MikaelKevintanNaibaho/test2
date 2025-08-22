cmake_minimum_required(VERSION 3.8)
project(krsri2025)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# find dependecies
set (HW_IF_INCLUDE_DEPENDS
  pluginlib
  hardware_interface
  rclcpp_lifecycle
)

set (REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)

set (CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  controller_interface
  realtime_tools
  rclcpp_lifecycle
  trajectory_msgs
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0 REQUIRED)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(visualization_msgs REQUIRED)

# compile
add_executable(send_trajectory
  trajectory_generator/send_trajectory.cpp
  trajectory_generator/foot_trajectory.cpp
)

target_link_libraries(send_trajectory PUBLIC
  ${trajectory_msgs_TARGETS}
  kdl_parser::kdl_parser
  rclcpp::rclcpp
  ${visualization_msgs_TARGETS}
)

target_include_directories(send_trajectory PRIVATE
  ${PROJECT_SOURCE_DIR}/trajectory_generator/include
)

add_library(
  krsri2025
  SHARED
  hardware/krsri2025_hardware.cpp
  controller/krsri_controller.cpp
)

target_compile_features(krsri2025 PUBLIC cxx_std_17)
target_include_directories(krsri2025 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/krsri2025>
)

target_link_libraries(krsri2025 PUBLIC
  ${trajectory_msgs_TARGETS}
  hardware_interface::hardware_interface
  controller_interface::controller_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
  rclcpp_lifecycle::rclcpp_lifecycle
)

# export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface krsri2025.xml)

# export controller plugins
pluginlib_export_plugin_description_file(controller_interface krsri2025.xml)


# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/krsri2025
)

install(
  DIRECTORY description/launch description/meshes description/ros2_control description/urdf description/rviz
  DESTINATION share/krsri2025
)

install(
  FILES
    config/controller.yaml
    config/trajectory_params.yaml
  DESTINATION share/krsri2025/config
)

install(TARGETS send_trajectory
  RUNTIME DESTINATION lib/krsri2025
)

install (TARGETS krsri2025
  EXPORT export_krsri2025
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

# exports
ament_export_targets(export_krsri2025 HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})

ament_package()
