cmake_minimum_required(VERSION 3.8)
project(point_cloud_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directorieS(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(pcd_import src/0_table_scene.cpp)
target_link_libraries(pcd_import  ${PCL_LIBRARIES})

add_executable(plane_cloud src/1_planner_cloud.cpp)
target_link_libraries(plane_cloud  ${PCL_LIBRARIES})

add_executable(circular_cloud src/2_circular_cloud.cpp)
target_link_libraries(circular_cloud ${PCL_LIBRARIES})

add_executable(obstacle_detection src/3_obstacle_detection.cpp)

ament_target_dependencies(obstacle_detection 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  geometry_msgs
  pcl_conversions
)

target_link_libraries(obstacle_detection ${PCL_LIBRARIES})

install(TARGETS
  pcd_import plane_cloud circular_cloud obstacle_detection
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
