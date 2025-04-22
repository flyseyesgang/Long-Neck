cmake_minimum_required(VERSION 3.8)
project(Visual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json REQUIRED)       # for reading JSON
find_package(ament_index_cpp REQUIRED)     # optional

include_directories(include ${OpenCV_INCLUDE_DIRS})

# common libs
set(COMMON 
  rclcpp sensor_msgs std_msgs geometry_msgs vision_msgs
  cv_bridge image_transport OpenCV tf2_ros nlohmann_json
)

# camera_node
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node ${COMMON})

# YOLO visualizer
add_executable(yolo_visualiser src/yolo_visualiser.cpp)
ament_target_dependencies(yolo_visualiser ${COMMON})

# detector (stand‐alone Hough)
add_executable(detector src/detector.cpp)
ament_target_dependencies(detector OpenCV)

# location_node (lid finder + optional Hough)
add_executable(location_node src/location.cpp)
ament_target_dependencies(location_node ${COMMON})

# Cartesian solver
add_executable(cartesian_node src/Cartesian.cpp)
ament_target_dependencies(cartesian_node ${COMMON})

# Brand overlay
add_executable(brand_node src/Imageoverlay.cpp)
ament_target_dependencies(brand_node ${COMMON})

# Recognition / decision
add_executable(recognition_node src/Recognition.cpp)
ament_target_dependencies(recognition_node ${COMMON})

# demo / GUI
add_executable(main_demo src/main.cpp)
ament_target_dependencies(main_demo ${COMMON})

add_executable(fusion_node src/fusion.cpp)
ament_target_dependencies(fusion_node rclcpp sensor_msgs vision_msgs cv_bridge OpenCV nlohmann_json)


install(
  TARGETS
    camera_node yolo_visualiser detector location_node
    cartesian_node brand_node recognition_node main_demo
    fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
