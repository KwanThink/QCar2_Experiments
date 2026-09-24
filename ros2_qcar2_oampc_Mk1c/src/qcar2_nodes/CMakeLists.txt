cmake_minimum_required(VERSION 3.8)
project(qcar2_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(qcar2_interfaces REQUIRED)

find_package(tf2_ros REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

if(NOT DEFINED ENV{GUROBI_HOME} OR "$ENV{GUROBI_HOME}" STREQUAL "")
  message(FATAL_ERROR "GUROBI_HOME must point to the installed Gurobi directory.")
endif()

set(GUROBI_HOME "$ENV{GUROBI_HOME}")
find_path(GUROBI_INCLUDE_DIR
  NAMES gurobi_c++.h
  HINTS "${GUROBI_HOME}/include"
  NO_DEFAULT_PATH
)
find_library(GUROBI_CXX_LIBRARY
  NAMES gurobi_c++
  HINTS "${GUROBI_HOME}/lib"
  NO_DEFAULT_PATH
)
find_library(GUROBI_C_LIBRARY
  NAMES gurobi130 gurobi120 gurobi110 gurobi100 gurobi95 gurobi
  HINTS "${GUROBI_HOME}/lib"
  NO_DEFAULT_PATH
)

if(NOT GUROBI_INCLUDE_DIR OR NOT GUROBI_CXX_LIBRARY OR NOT GUROBI_C_LIBRARY)
  message(FATAL_ERROR "Gurobi C++ headers or libraries were not found under GUROBI_HOME=${GUROBI_HOME}.")
endif()

# Need to define _NVIDIA macro since the Quanser SDK has different
# functionalities depending on which platform it's being used.
add_compile_definitions(_NVIDIA)

include_directories(include)

# The header 'cv_bridge/cv_bridge.hpp' was added in version 3.3.0. For older
# cv_bridge versions, we have to use the header 'cv_bridge/cv_bridge.h'.
if(${cv_bridge_VERSION} VERSION_GREATER_EQUAL "3.3.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCV_BRDIGE_HAS_HPP")
endif()

#add_executable(qcar2_driver_interface src/qcar2_driver_interface.cpp)
#target_link_libraries(qcar2_driver_interface -lquanser_communications)
#ament_target_dependencies(qcar2_driver_interface rclcpp std_msgs geometry_msgs sensor_msgs control_msgs)

add_executable(qcar2_odometry src/qcar2_odometry.cpp)
ament_target_dependencies(qcar2_odometry
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(qcar2_hardware src/qcar2_hardware.cpp)
target_link_libraries(qcar2_hardware -lhil -lquanser_devices -lquanser_runtime -lquanser_common)
ament_target_dependencies(qcar2_hardware rclcpp std_msgs geometry_msgs sensor_msgs control_msgs qcar2_interfaces)

add_executable(rgbd src/rgbd.cpp)
# Orin has OpenCV 4 installed, and put it under the /usr/include/opencv4 directory
# So we need to add this directory to the include directory list for the node to
# compile properly.
target_include_directories(rgbd PUBLIC /usr/include/opencv4)
target_link_libraries(rgbd cv_bridge::cv_bridge image_transport::image_transport opencv_imgcodecs -lquanser_media -lquanser_runtime -lquanser_common)
ament_target_dependencies(rgbd rclcpp cv_bridge image_transport OpenCV std_msgs sensor_msgs)

add_executable(lidar src/lidar.cpp)
target_link_libraries(lidar -lquanser_devices -lquanser_communications -lquanser_runtime -lquanser_common)
ament_target_dependencies(lidar rclcpp sensor_msgs)

add_executable(csi src/csi.cpp)
# Orin has OpenCV 4 installed, and put it under the /usr/include/opencv4 directory
# So we need to add this directory to the include directory list for the node to
# compile properly.
target_include_directories(csi PUBLIC /usr/include/opencv4)
target_link_libraries(csi cv_bridge::cv_bridge image_transport::image_transport opencv_imgcodecs -lquanser_media -lquanser_runtime -lquanser_common)
ament_target_dependencies(csi rclcpp cv_bridge image_transport OpenCV std_msgs sensor_msgs)

# TF
# qcar2 -> lidar
add_executable(fixed_lidar_frame src/fixed_lidar_frame.cpp)
ament_target_dependencies(fixed_lidar_frame geometry_msgs rclcpp tf2 tf2_ros)

# OAMPC autonomous tracking and obstacle-avoidance controller.
add_executable(qcar2_oampc_controller
  src/qcar2_oampc_controller.cpp
  src/oampc_solver.cpp
  src/obstacle_map.cpp
  src/obstacle_activation.cpp
  src/qcar2_traj.cpp
  src/result_logger.cpp
)
target_include_directories(qcar2_oampc_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GUROBI_INCLUDE_DIR}
)
ament_target_dependencies(qcar2_oampc_controller
  ament_index_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  qcar2_interfaces
)
target_link_libraries(qcar2_oampc_controller
  ${GUROBI_CXX_LIBRARY}
  ${GUROBI_C_LIBRARY}
  yaml-cpp
  m
)
set_target_properties(qcar2_oampc_controller PROPERTIES
  BUILD_RPATH "${GUROBI_HOME}/lib"
  INSTALL_RPATH "${GUROBI_HOME}/lib"
)

# Build the launch-time occupancy-map preprocessing executable.
add_executable(generate_obstacle_map src/generate_obstacle_map.cpp)
target_include_directories(generate_obstacle_map PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(generate_obstacle_map ${OpenCV_LIBS} yaml-cpp)

# Install nodes
#install(TARGETS qcar2_driver_interface
#  DESTINATION lib/${PROJECT_NAME})
install(TARGETS qcar2_hardware
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS rgbd
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS lidar
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS csi
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS qcar2_oampc_controller
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS generate_obstacle_map
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS qcar2_odometry
  DESTINATION lib/${PROJECT_NAME})

# Install TF
install(TARGETS fixed_lidar_frame
  DESTINATION lib/${PROJECT_NAME})

# Install result plotting scripts used by the C++ logger.
install(
  PROGRAMS scripts/plot_oampc_results.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install headers used by the OAMPC node.
install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}/
)

# Install configuration needed for the localization and OAMPC nodes.
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
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
