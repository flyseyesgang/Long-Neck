// src/Cartesian.cpp
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class Cartesian : public rclcpp::Node {
public:
  Cartesian(): Node("cartesian") {
    // Declare parameters
    declare_parameter<double>("lid_diameter_mm", 28.0);
    declare_parameter<std::vector<double>>("camera_intrinsics",
      std::vector<double>{600.0, 0.0, 320.0,
                           0.0, 600.0, 240.0,
                           0.0,   0.0,   1.0});

    // Read parameters
    lid_diam_mm_ = get_parameter("lid_diameter_mm").as_double();
    auto cam = get_parameter("camera_intrinsics").as_double_array();
    K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(cam.data())).clone();

    // Subscribe to detection topic
    det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "yolo_detections", 10,
      std::bind(&Cartesian::detectionsCallback, this, _1)
    );
    // Publisher for 3D poses
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("lid_poses", 10);
    RCLCPP_INFO(get_logger(), "Cartesian node initialized");
  }

private:
  bool done_{false};

  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    // Only proceed once all 12 lids are detected
    if (done_) {
      return;
    }

    if (msg->detections.size() != 12) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Detections=%zu !=12, waiting...", msg->detections.size());
      return;
    }

    // Prepare PoseArray
    geometry_msgs::msg::PoseArray pa;
    pa.header = msg->header;

    struct Item { double u, v; geometry_msgs::msg::Pose pose; };
    std::vector<Item> items;
    items.reserve(msg->detections.size());

    // Compute 3D pose for each detection
    for (const auto &d : msg->detections) {
      double u = d.bbox.center.position.x;
      double v = d.bbox.center.position.y;
      double pix_diam = (d.bbox.size_x + d.bbox.size_y) * 0.5;
      double real_diam_m = lid_diam_mm_ * 1e-3;  // convert mm to m

      double fx = K_.at<double>(0, 0);
      double fy = K_.at<double>(1, 1);
      double cx = K_.at<double>(0, 2);
      double cy = K_.at<double>(1, 2);

      double Z = fx * real_diam_m / pix_diam;
      double X = (u - cx) * Z / fx;
      double Y = (v - cy) * Z / fy;

      Item it;
      it.u = u; it.v = v;
      it.pose.position.x = X;
      it.pose.position.y = Y;
      it.pose.position.z = Z;
      it.pose.orientation.w = 1.0;
      items.push_back(it);
    }

    // Sort detections: top-left to bottom-right
    std::sort(items.begin(), items.end(), [](const Item &a, const Item &b){
      if (a.v != b.v) return a.v < b.v;
      return a.u < b.u;
    });

    // Log and populate
    for (size_t i = 0; i < items.size(); ++i) {
      pa.poses.push_back(items[i].pose);
      RCLCPP_INFO(get_logger(),
        "bottle[%zu] = [%.3f, %.3f, %.3f]",
        i,
        items[i].pose.position.x,
        items[i].pose.position.y,
        items[i].pose.position.z
      );
    }

    // Publish once
    pose_pub_->publish(pa);
    done_ = true;
  }

  // Members
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  cv::Mat K_;
  double lid_diam_mm_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Cartesian>());
  rclcpp::shutdown();
  return 0;
}