#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class ImageProcessor : public rclcpp::Node {
public:
  ImageProcessor() : Node("image_processor") {
    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image", 10, std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat processed_image;
    // Insert OpenCV code here to process frame
    cv::imshow("View", processed_image);
    cv::waitKey(10);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageProcessor>());
  rclcpp::shutdown();
  return 0;
}
