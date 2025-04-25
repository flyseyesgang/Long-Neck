// src/Cartesian.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class Cartesian : public rclcpp::Node {
public:
  Cartesian(): Node("cartesian") {
    declare_parameter<double>("lid_diameter_mm", 28.0);
    declare_parameter<std::vector<double>>(
      "camera_intrinsics",
      std::vector<double>{600.0,0.0,320.0, 0.0,600.0,240.0, 0.0,0.0,1.0});
    auto cam = get_parameter("camera_intrinsics").as_double_array();
    K_ = cv::Mat(3,3,CV_64F, (void*)cam.data()).clone();
    lid_diam_ = get_parameter("lid_diameter_mm").as_double();

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "yolo_detections", 10,
      std::bind(&Cartesian::cb, this, std::placeholders::_1));
    pub_ = create_publisher<geometry_msgs::msg::PoseArray>("lid_poses", 10);
  }

private:
  void cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat gray; cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT,
                     1.0, 30.0, 100.0, 30.0, 10, 40);

    geometry_msgs::msg::PoseArray pa;
    pa.header = msg->header;
    double f = K_.at<double>(0,0);

    for (auto &c : circles) {
      double u = c[0], v = c[1], r = c[2];
      double Z = (lid_diam_ * f) / (2.0 * r);
      double X = (u - K_.at<double>(0,2)) * Z / f;
      double Y = (v - K_.at<double>(1,2)) * Z / f;

      geometry_msgs::msg::Pose p;
      p.position.x = X / 1000.0;
      p.position.y = Y / 1000.0;
      p.position.z = Z / 1000.0;
      pa.poses.push_back(p);
    }

    pub_->publish(pa);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;
  cv::Mat K_;
  double lid_diam_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Cartesian>());
  rclcpp::shutdown();
  return 0;
}
