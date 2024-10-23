cmake_minimum_required(VERSION 3.8)
project(simple_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  src
  ${OpenCV_INCLUDE_DIRS}
)

# Export dependencies
ament_export_dependencies(rclcpp nav_msgs geometry_msgs OpenCV)

# Declare libraries
add_library(custom_path_finding src/custom_path_finding.cpp)
add_library(obstacle_distance_map src/obstacle_distance_map.cpp)

# Link libraries and dependencies
ament_target_dependencies(custom_path_finding rclcpp nav_msgs geometry_msgs)
ament_target_dependencies(obstacle_distance_map rclcpp nav_msgs geometry_msgs)

target_link_libraries(custom_path_finding ${OpenCV_LIBS})
target_link_libraries(obstacle_distance_map ${OpenCV_LIBS})

# Declare executables
add_executable(map_handler src/map_handler.cpp)
add_executable(path_planner src/path_planner.cpp)

# Specify dependencies
ament_target_dependencies(map_handler rclcpp nav_msgs geometry_msgs)
ament_target_dependencies(path_planner rclcpp nav_msgs geometry_msgs)

# Link libraries to executables
target_link_libraries(map_handler obstacle_distance_map)
target_link_libraries(path_planner custom_path_finding ${OpenCV_LIBS})

# Install targets
install(TARGETS
  map_handler
  path_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install header files if needed
install(DIRECTORY src/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# Remove or comment out the following lines since 'config/' doesn't exist
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

ament_package()
