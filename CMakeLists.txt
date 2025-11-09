cmake_minimum_required(VERSION 3.16)
project(dwal_planner)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_common QUIET)
find_package(nav2_util QUIET)
find_package(Boost REQUIRED COMPONENTS system)

# ---------------------------
# Messages & Services
# ---------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClusterGroup.msg"
  "msg/Path.msg"
  "msg/PathCluster.msg"
  "msg/Pose2D32.msg"
  "msg/SampledCluster.msg"
  "msg/SampledPath.msg"
  "msg/ThinCluster.msg"
  "srv/ToggleClusters.srv"
  "srv/ToggleSlice.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs trajectory_msgs visualization_msgs
)

# Global headers
include_directories(include)

# ---------------------------
# Library: dwal_cluster
# ---------------------------
add_library(dwal_cluster
  src/libdwal_cluster.cpp
)

ament_target_dependencies(dwal_cluster
  rclcpp geometry_msgs sensor_msgs std_msgs trajectory_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs
  nav2_costmap_2d
)

target_include_directories(dwal_cluster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

add_dependencies(dwal_cluster ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(dwal_cluster ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---------------------------
# Executable: dwal_clustering
# ---------------------------
add_executable(dwal_clustering src/clustering.cpp)

ament_target_dependencies(dwal_clustering
  rclcpp geometry_msgs sensor_msgs std_msgs trajectory_msgs visualization_msgs
  tf2 tf2_ros nav2_costmap_2d
)

# generated headers path for this target
target_include_directories(dwal_clustering PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

add_dependencies(dwal_clustering ${PROJECT_NAME}__rosidl_generator_cpp)
target_link_libraries(dwal_clustering dwal_cluster)
rosidl_target_interfaces(dwal_clustering ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---------------------------
# Executable: dwal_generator
# ---------------------------
add_executable(dwal_generator src/trajectory_generator.cpp)

ament_target_dependencies(dwal_generator
  rclcpp geometry_msgs sensor_msgs std_msgs trajectory_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs
  nav2_costmap_2d
)

target_include_directories(dwal_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

add_dependencies(dwal_generator ${PROJECT_NAME}__rosidl_generator_cpp)
target_link_libraries(dwal_generator dwal_cluster)
rosidl_target_interfaces(dwal_generator ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---------------------------
# Install
# ---------------------------
install(TARGETS
  dwal_cluster
  dwal_clustering
  dwal_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Launch & params
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# ---------------------------
# Export
# ---------------------------
ament_export_dependencies(
  rclcpp geometry_msgs sensor_msgs std_msgs trajectory_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs nav2_costmap_2d
)
ament_export_include_directories(include)

ament_package()
