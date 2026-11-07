cmake_minimum_required(VERSION 3.8)
project(team_loc)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(isaac_ros_visual_slam_interfaces REQUIRED)

# Pure logic (no ROS): unit-tested on its own.
add_library(team_loc_core STATIC src/course_config.cpp src/fuser_core.cpp)
target_include_directories(team_loc_core PUBLIC include)
target_link_libraries(team_loc_core Eigen3::Eigen)
set_target_properties(team_loc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(loc_fuser src/loc_fuser_node.cpp)
target_link_libraries(loc_fuser team_loc_core)
ament_target_dependencies(loc_fuser rclcpp nav_msgs geometry_msgs diagnostic_msgs tf2 tf2_ros
  tf2_eigen px4_msgs isaac_ros_visual_slam_interfaces)

install(TARGETS loc_fuser DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_team_loc test/test_pose_math.cpp test/test_course_config.cpp
    test/test_fuser_core.cpp
    test/test_filter.cpp)
  target_link_libraries(test_team_loc team_loc_core)
  target_compile_definitions(test_team_loc PRIVATE
    COURSE_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../planner/config/course_v3.yaml")
endif()

ament_package()
