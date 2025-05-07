cmake_minimum_required(VERSION 3.8)
project(robot_follow_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
#~ find_package(tf_transformations REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Define a CMake variable named dependencies that lists all
# ROS 2 packages and other dependencies the project requires.
set(dependencies
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  tf2_msgs
  tf_transformations
  actionlib
)
 
# Add the specified directories to the list of paths that the compiler
# uses to search for header files. This is important for C++
# projects where you have custom header files that are not located
# in the standard system include paths.
include_directories(
  include
)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY ${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules for import
ament_python_install_package(${PROJECT_NAME})


# Add this section to install Python scripts
install(
  PROGRAMS
  ${PROJECT_NAME}/follow.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
  ${PROJECT_NAME}/republish_tf.py
  DESTINATION lib/${PROJECT_NAME}
)






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

# Used to export include directories of a package so that they can be easily
# included by other packages that depend on this package.
ament_export_include_directories(include)

ament_package()
