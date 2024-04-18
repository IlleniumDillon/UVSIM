cmake_minimum_required(VERSION 3.5)
project(sa_task_sokoban)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
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
find_package(simbridge REQUIRED)
find_package(simplanner REQUIRED)
find_package(simallocator REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sokoban_solver REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS}) 
include_directories(include/sa_task_sokoban)

file(GLOB SOURCEFILE src/*.cpp)

add_executable(sa_socoban_node src/sa_socoban_node.cpp src/Sokoban.cpp)
ament_target_dependencies(
  sa_socoban_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "simbridge"
  "simplanner"
  "simallocator"
  "sensor_msgs"
  "OpenCV"
  "sokoban_solver"
)

install(TARGETS
  sa_socoban_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(sa_task_publish src/sa_publish_task.cpp)
ament_target_dependencies(
  sa_task_publish
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "simbridge"
  "simplanner"
  "simallocator"
  "sensor_msgs"
)

install(TARGETS
  sa_task_publish
  DESTINATION lib/${PROJECT_NAME})

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
