cmake_minimum_required(VERSION 3.8)
project(visual)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)
find_package(ament_index_cpp REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# camera node
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node
  rclcpp sensor_msgs cv_bridge OpenCV)

# esky detector
add_executable(esky_detector src/EskyDetector.cpp)
ament_target_dependencies(esky_detector
  rclcpp sensor_msgs cv_bridge OpenCV)

### ONNX Runtime (absolute path) ####
set(ONNXRUNTIME_ROOT "$ENV{HOME}/ros2_ws/onnxruntime")
include_directories(${ONNXRUNTIME_ROOT}/include)
link_directories(${ONNXRUNTIME_ROOT}/lib)


add_executable(bottle_detector_ort src/BottleDetector.cpp)
ament_target_dependencies(bottle_detector_ort
  rclcpp sensor_msgs cv_bridge OpenCV
)
target_link_libraries(bottle_detector_ort
  onnxruntime               # resolves to libonnxruntime.so
)

# cartesian estimator
add_executable(cartesian src/Cartesian.cpp)
ament_target_dependencies(cartesian
  rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

# brand recognizer
add_executable(brand_recognition src/BrandRecognition.cpp)
ament_target_dependencies(brand_recognition
  rclcpp sensor_msgs vision_msgs cv_bridge OpenCV)

# fusion node
add_executable(fusion_node src/main.cpp)
ament_target_dependencies(fusion_node
  rclcpp sensor_msgs std_msgs geometry_msgs vision_msgs cv_bridge OpenCV)

add_executable(crop_node src/CropNode.cpp)
ament_target_dependencies(crop_node
  rclcpp sensor_msgs vision_msgs cv_bridge OpenCV)

# Install the ONNX model you’ve placed under src/RS2/visual/model
install(
  DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  camera_node
  crop_node
  esky_detector
  bottle_detector_ort
  cartesian
  brand_recognition
  fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
