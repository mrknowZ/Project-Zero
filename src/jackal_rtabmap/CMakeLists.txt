cmake_minimum_required(VERSION 3.8)
project(jackal_rtabmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install Python Executables as ROS2 nodes
install(PROGRAMS
  jackal_rtabmap/slope_inclinometer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME slope_inclinometer_node
)

install(PROGRAMS
  jackal_rtabmap/frontier_explorer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME frontier_explorer_node
)

install(PROGRAMS
  jackal_rtabmap/semantic_object_mapper_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME semantic_object_mapper_node
)

install(PROGRAMS
  jackal_rtabmap/web_telemetry_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME web_telemetry_bridge_node
)

install(PROGRAMS
  jackal_rtabmap/slope_traversability_costmap_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME slope_traversability_costmap_node
)

install(DIRECTORY launch config www
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
