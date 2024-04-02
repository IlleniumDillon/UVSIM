cmake_minimum_required(VERSION 3.5)
project(caterpillar_robot)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include/caterpillar_robot)
link_directories(${gazebo_dev_LIBRARY_DIRS})

file(GLOB SOURCEFILES src/set_joint_state_node.cpp)
file(GLOB PLUGINFILE src/gazebo_ros_servojoint.cpp)

add_executable(set_joint_state_node ${SOURCEFILES})
ament_target_dependencies(
  set_joint_state_node
  "rclcpp"
  "std_msgs"
  "trajectory_msgs"
)
install(TARGETS
  set_joint_state_node
  DESTINATION lib/${PROJECT_NAME})

# gazebo_ros_joint_pose_trajectory
add_library(gazebo_ros_servojoint SHARED
  src/gazebo_ros_servojoint.cpp
)
target_include_directories(gazebo_ros_servojoint PUBLIC include)
ament_target_dependencies(gazebo_ros_servojoint
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "trajectory_msgs"
)
ament_export_libraries(gazebo_ros_servojoint)
if(ENABLE_PROFILER)
  target_include_directories(gazebo_ros_servojoint PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(gazebo_ros_servojoint ${ignition-common3_LIBRARIES})
endif()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
  gazebo_ros_servojoint
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch urdf worlds meshes config map rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
