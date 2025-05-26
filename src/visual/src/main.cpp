// src/main.cpp (fusion_node)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class FusionNode : public rclcpp::Node {
public:
  FusionNode(): Node("fusion_node") {
    esky_sub_ = create_subscription<std_msgs::msg::Bool>(
      "esky_ok", 10,
      std::bind(&FusionNode::onEsky, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "lid_poses", 10,
      std::bind(&FusionNode::onPoses, this, std::placeholders::_1));
    brand_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "brand_detections", 10,
      std::bind(&FusionNode::onBrands, this, std::placeholders::_1));

    pub_final_ = create_publisher<vision_msgs::msg::Detection2DArray>(
      "fusion/bottles", 10);
  }

private:
  void onEsky(const std_msgs::msg::Bool::SharedPtr msg) {
    esky_ok_ = msg->data;
  }
  void onPoses(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    poses_ = *msg;
  }
  void onBrands(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    brands_ = *msg;
    publishCombined();
  }
  void publishCombined() {
    if (!esky_ok_) return;
    brands_.header = poses_.header;
    pub_final_->publish(brands_);
  }

  bool esky_ok_{false};
  geometry_msgs::msg::PoseArray poses_;
  vision_msgs::msg::Detection2DArray brands_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           esky_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr brand_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_final_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionNode>());
  rclcpp::shutdown();
  return 0;
}
