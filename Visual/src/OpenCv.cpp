#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono> // Correct use of the include directive
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class OpenCv : public rclcpp::Node {
public:
    OpenCv() : Node("OpenCv"), count_(0) {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("random_image", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&OpenCv::timer_callback, this));
    }

private:
    void timer_callback() {
        // Create a new 640x480 image
        cv::Mat my_image(cv::Size(640, 480), CV_8UC3);

        // Generate an image where each pixel is a random color
        cv::randu(my_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
        cv::Point text_position(15, 40);

        // Declare the size and color of the font
        int font_size = 1;
        cv::Scalar font_color(255, 255, 255); // White color

        // Declare the font weight (thickness)
        int font_weight = 2;

        // Put the text on the image
        cv::putText(my_image, "ROS2 + OpenCV", text_position, cv::FONT_HERSHEY_COMPLEX, font_size, font_color, font_weight);

        // Convert a CvImage into a ROS image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image).toImageMsg();

        // Publish the image to the topic defined in the publisher
        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Image %ld published", count_++);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OpenCv>(); // Correct class name used here
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}