cmake_minimum_required(VERSION 3.8)
project(my_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(odom_to_tf src/odom_to_tf.cpp)
ament_target_dependencies(odom_to_tf rclcpp nav_msgs geometry_msgs tf2_ros)

install(TARGETS
  odom_to_tf
  DESTINATION lib/${PROJECT_NAME})

add_executable(cmd_vel_republisher src/cmd_vel_republisher.cpp)
ament_target_dependencies(cmd_vel_republisher rclcpp geometry_msgs)

install(TARGETS
  cmd_vel_republisher
  DESTINATION lib/${PROJECT_NAME})

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

# Install launch, config, description, world files
install(DIRECTORY
  launch config description worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
