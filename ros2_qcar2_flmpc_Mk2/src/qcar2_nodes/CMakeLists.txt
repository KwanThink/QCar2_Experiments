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

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(qcar2_interfaces REQUIRED)

find_package(tf2_ros REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

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

# FLMPC autonomous tracking controller.
add_executable(qcar2_flmpc_controller
  src/qcar2_flmpc_controller.cpp
  src/qcar2_traj.cpp
  src/result_logger.cpp
)
target_include_directories(qcar2_flmpc_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(qcar2_flmpc_controller
  ament_index_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  qcar2_interfaces
)

set(QCAR2_ACADOS_SOLVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/acados_solver/qcar2_flmpc_solver)
set(QCAR2_ACADOS_SOLVER_HEADER ${QCAR2_ACADOS_SOLVER_DIR}/acados_solver_qcar2_flmpc.h)
set(QCAR2_ACADOS_SOLVER_LIB ${QCAR2_ACADOS_SOLVER_DIR}/libacados_ocp_solver_qcar2_flmpc.so)

if(EXISTS ${QCAR2_ACADOS_SOLVER_HEADER} AND EXISTS ${QCAR2_ACADOS_SOLVER_LIB})
  message(STATUS "Building qcar2_flmpc_controller with generated acados solver")
  target_compile_definitions(qcar2_flmpc_controller PRIVATE QCAR2_HAS_ACADOS_SOLVER)
  target_include_directories(qcar2_flmpc_controller PRIVATE
    ${QCAR2_ACADOS_SOLVER_DIR}
    $ENV{ACADOS_SOURCE_DIR}/include
    $ENV{ACADOS_SOURCE_DIR}/external/blasfeo/include
    $ENV{ACADOS_SOURCE_DIR}/external/hpipm/include
    /home/nvidia/acados/include
    /home/nvidia/acados/external/blasfeo/include
    /home/nvidia/acados/external/hpipm/include
  )
  target_link_directories(qcar2_flmpc_controller PRIVATE
    $ENV{ACADOS_SOURCE_DIR}/lib
    /home/nvidia/acados/lib
  )
  target_link_libraries(qcar2_flmpc_controller
    ${QCAR2_ACADOS_SOLVER_LIB}
    acados
    hpipm
    blasfeo
    m
  )
  set_target_properties(qcar2_flmpc_controller PROPERTIES
    BUILD_RPATH "${QCAR2_ACADOS_SOLVER_DIR};/home/nvidia/acados/lib"
    INSTALL_RPATH "$ORIGIN;/home/nvidia/acados/lib"
  )
  install(FILES
    ${QCAR2_ACADOS_SOLVER_LIB}
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "Generated acados solver was not found in acados_solver/qcar2_flmpc_solver. qcar2_flmpc_controller will build with a safe zero-command solver stub.")
endif()


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

install(TARGETS qcar2_flmpc_controller
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS qcar2_odometry
  DESTINATION lib/${PROJECT_NAME})

# Install TF
install(TARGETS fixed_lidar_frame
  DESTINATION lib/${PROJECT_NAME})

# Install result plotting scripts used by the C++ logger.
install(
  PROGRAMS scripts/plot_flmpc_results.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install headers and generated-solver folder used by the FLMPC node.
install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY acados_solver
  DESTINATION share/${PROJECT_NAME}/
)

# Install configuration needed for the localization and FLMPC nodes.
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
