cmake_minimum_required(VERSION 3.8)
project(Visual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(main src/main.cpp)
ament_target_dependencies(main
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  OpenCV
  std_msgs
)

add_executable(Imageoverlay src/Imageoverlay.cpp)
ament_target_dependencies(Imageoverlay
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  OpenCV
  std_msgs
)

add_executable(OpenCv src/OpenCv.cpp)
ament_target_dependencies(OpenCv
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  OpenCV
  std_msgs
)

add_executable(PCL src/PCL.cpp)
ament_target_dependencies(PCL
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  OpenCV
  std_msgs
)

add_executable(Recognition src/Recognition.cpp)
ament_target_dependencies(Recognition
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  OpenCV
  std_msgs
)

install(TARGETS
  main
  Imageoverlay
  OpenCv
  PCL
  Recognition
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_lint_auto_find_test_dependencies TRUE)
endif()

ament_package()
