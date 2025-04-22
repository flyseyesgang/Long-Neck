#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node {
public:
  CameraNode(): Node("camera_node") {
    int cam_index = declare_parameter("camera_index", 0);
    double fps    = declare_parameter("fps", 30.0);

    if(!cap_.open(cam_index)) {
      RCLCPP_FATAL(get_logger(),"Cannot open camera");
      rclcpp::shutdown();
    }
    pub_ = create_publisher<sensor_msgs::msg::Image>("webcam/image_raw", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(int(1000.0/fps)),
      std::bind(&CameraNode::grab_frame, this)
    );
  }

private:
  void grab_frame(){
    cv::Mat frame;
    if(cap_.read(frame)){
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

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
