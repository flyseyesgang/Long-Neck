cmake_minimum_required(VERSION 3.8)
project(ur3_cartesian_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(ur3_cartesian_motion_node
  src/ur3_cartesian_motion_node.cpp
)
ament_target_dependencies(ur3_cartesian_motion_node
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

install(TARGETS ur3_cartesian_motion_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skip copyright/CPPlint checks until you add headers & licenses
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(ur3_cartesian_motion)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(moveit_core REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)

# add_executable(ur3_cartesian_motion_node
#   src/ur3_cartesian_motion_node.cpp
# )
# ament_target_dependencies(ur3_cartesian_motion_node
#   rclcpp
#   geometry_msgs
#   tf2_geometry_msgs
#   moveit_core
#   moveit_ros_planning_interface
# )

# install(TARGETS ur3_cartesian_motion_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
