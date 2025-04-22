#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CartesianNode : public rclcpp::Node
{
public:
  CartesianNode()
  : Node("cartesian_node")
  {
    fx_     = declare_parameter<double>("fx", 920.0);
    fy_     = declare_parameter<double>("fy", 920.0);
    cx_     = declare_parameter<double>("cx", 640.0);
    cy_     = declare_parameter<double>("cy", 360.0);
    cap_mm_ = declare_parameter<double>("cap_mm", 28.0);

    sub_img_ = create_subscription<sensor_msgs::msg::Image>(
      "/webcam/image_raw", 10,
      std::bind(&CartesianNode::img_cb, this, std::placeholders::_1)
    );
    sub_box_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/bottle_boxes", 10,
      std::bind(&CartesianNode::box_cb, this, std::placeholders::_1)
    );
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseArray>("bottle_poses",10);
  }

private:
  void img_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_img_ = msg;
  }

  void box_cb(const vision_msgs::msg::Detection2DArray::SharedPtr boxes)
  {
    if (!last_img_) return;
    auto frame = cv_bridge::toCvCopy(last_img_, "bgr8")->image;

    geometry_msgs::msg::PoseArray out;
    out.header = boxes->header;

    for (auto &det : boxes->detections) {
      if (det.results.empty() ||
          det.results[0].hypothesis.class_id != "bottle")  // only bottle
        continue;

      auto &bb = det.bbox;
      int x = int(bb.center.position.x - bb.size_x/2);
      int y = int(bb.center.position.y - bb.size_y/2);
      int w = int(bb.size_x), h = int(bb.size_y);

      cv::Mat roi = frame(cv::Rect(x,y,w,h));
      cv::Mat gray; cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

      std::vector<cv::Vec3f> circles;
      cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT,
                       1, gray.rows/4, 100, 20, 8, 20);
      if (circles.empty()) continue;

      double pix_d = circles[0][2]*2.0;
      double Z = (fx_ * (cap_mm_/1000.0)) / pix_d;

      double u = x + circles[0][0];
      double v = y + circles[0][1];
      double X = (u - cx_)*Z/fx_;
      double Y = (v - cy_)*Z/fy_;

      geometry_msgs::msg::Pose p;
      p.position.x = X; p.position.y = Y; p.position.z = Z;
      p.orientation.w = 1.0;
      out.poses.push_back(p);
    }

    pub_pose_->publish(out);
  }

  double fx_, fy_, cx_, cy_, cap_mm_;
  sensor_msgs::msg::Image::SharedPtr last_img_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_box_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianNode>());
  rclcpp::shutdown();
  return 0;
}
