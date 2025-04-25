// src/EskyDetector.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class EskyDetector : public rclcpp::Node {
public:
  EskyDetector(): Node("esky_detector") {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "webcam/image_raw", 10,
      std::bind(&EskyDetector::imageCallback, this, std::placeholders::_1));
    pub_mask_ = create_publisher<sensor_msgs::msg::Image>("esky_mask", 10);
    pub_ok_   = create_publisher<std_msgs::msg::Bool>("esky_ok", 10);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    auto cv_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat gray, blurred, thresh, mask = cv::Mat::zeros(cv_img.size(), CV_8UC1);

    cv::cvtColor(cv_img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5,5), 0);
    cv::adaptiveThreshold(blurred, thresh, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV, 11, 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double bestArea = 0;
    std::vector<cv::Point> bestPoly;
    for (auto &c : contours) {
      std::vector<cv::Point> poly;
      cv::approxPolyDP(c, poly, 0.02 * cv::arcLength(c, true), true);
      if (poly.size() == 4) {
        double area = cv::contourArea(poly);
        if (area > bestArea) {
          bestArea = area;
          bestPoly = poly;
        }
      }
    }

    bool ok = !bestPoly.empty();
    std_msgs::msg::Bool ok_msg; ok_msg.data = ok;
    pub_ok_->publish(ok_msg);

    if (ok) {
      std::vector<std::vector<cv::Point>> draw{bestPoly};
      cv::fillPoly(mask, draw, cv::Scalar(255));
    }

    auto out = cv_bridge::CvImage(msg->header, "mono8", mask).toImageMsg();
    pub_mask_->publish(*out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_mask_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ok_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EskyDetector>());
  rclcpp::shutdown();
  return 0;
}
