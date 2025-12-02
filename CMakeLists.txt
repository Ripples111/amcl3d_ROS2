project(amcl3d)
cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED) # ROS 2中没有octomap_ros，需要使用octomap包
find_package(pcl_conversions REQUIRED)# ROS 2中没有pcl_ros，需要使用pcl包
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap REQUIRED)

###########
## Build ##
###########

include_directories(
	include
  ${rclcpp_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

add_library(my_library
    src/Grid3d.cpp
    src/ParticleFilter.cpp
    src/PointCloudTools.cpp
)

add_executable(amcl3d
  src/main.cpp
  src/Node.cpp
  src/Parameters.cpp
  )

ament_target_dependencies(amcl3d
  rclcpp
  visualization_msgs
  sensor_msgs
  nav_msgs
  octomap
)

add_executable(starter
  src/starter.cpp
)

ament_target_dependencies(starter
  rclcpp
  std_msgs
)

target_link_libraries(amcl3d
my_library
${rclcpp_LIBRARIES} 
${PCL_LIBRARIES} 
${tf2_ros_LIBRARIES}
${sensor_msgs_LIBRARIES}
${nav_msgs_LIBRARIES}
${octomap_LIBRARIES} 
)

#############
## Testing ##
#############

# Install targets
install(TARGETS amcl3d
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS starter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

# Export include directory
ament_export_include_directories(include)

# Export libraries
ament_export_libraries(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install package manifest
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# If this package has a test setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize the package
ament_package()
