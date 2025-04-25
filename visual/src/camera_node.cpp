// src/camera_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <chrono>

class CameraNode : public rclcpp::Node {
public:
  CameraNode(): Node("camera_node") {
    // 1) Declare the file-mode parameter
    std::string image_file = declare_parameter<std::string>("image_file", "");

    // 2) Create the publisher (reliable QoS by default)
    pub_ = create_publisher<sensor_msgs::msg::Image>("webcam/image_raw", 10);

    // 3) If file-mode, load & publish the frame multiple times, then exit
    if (!image_file.empty()) {
      RCLCPP_INFO(get_logger(), "File-mode: loading image '%s'", image_file.c_str());
      cv::Mat frame = cv::imread(image_file, cv::IMREAD_COLOR);
      if (frame.empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to load image: %s", image_file.c_str());
        rclcpp::shutdown();
        return;
      }
      // convert once
      auto msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), "bgr8", frame
      ).toImageMsg();

      // publish it 5 times at 200ms intervals
      for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
        msg->header.stamp = now();
        pub_->publish(*msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
      RCLCPP_INFO(get_logger(), "Published %d frames, shutting down.", 5);
      rclcpp::shutdown();
      return;
    }

    // 4) Otherwise fall back to live camera mode
    int cam_index = declare_parameter<int>("camera_index", 0);
    double fps    = declare_parameter<double>("fps", 30.0);

    if (!cap_.open(cam_index)) {
      RCLCPP_FATAL(get_logger(), "Cannot open camera index %d", cam_index);
      rclcpp::shutdown();
      return;
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(int(1000.0 / fps)),
      std::bind(&CameraNode::grab_frame, this)
    );
  }

private:
  void grab_frame() {
    cv::Mat frame;
    if (cap_.read(frame)) {
      auto msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), "bgr8", frame
      ).toImageMsg();
      msg->header.stamp = now();
      pub_->publish(*msg);
    }
  }

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraNode>();
  if (rclcpp::ok()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
