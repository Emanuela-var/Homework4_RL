cmake_minimum_required(VERSION 3.5)
project(rl_fra2mo_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Trova le dipendenze necessarie
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # Necessario per `aruco_transform_node`
find_package(rclpy REQUIRED)  # Aggiungi rclpy per supportare i nodi Python

# Aggiungi gli eseguibili C++
add_executable(dynamic_tf_publisher src/odom_bl_tf.cpp)
ament_target_dependencies(dynamic_tf_publisher rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(aruco_transform_node src/aruco_pose.cpp)
ament_target_dependencies(aruco_transform_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Installa gli eseguibili C++
install(TARGETS
  dynamic_tf_publisher
  aruco_transform_node
  DESTINATION lib/${PROJECT_NAME}
)

# Installa directory necessarie
install(DIRECTORY launch urdf models meshes worlds config src maps scripts
  DESTINATION share/${PROJECT_NAME}
)

# Installa gli script Python
install(PROGRAMS
  scripts/reach_goal.py
  scripts/follow_waypoints.py
  scripts/aruco_waypoint.py
  scripts/vision_based_navigation.py  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
