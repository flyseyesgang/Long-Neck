// src/ur3_cartesian_motion_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class UR3CartesianMotionNode : public rclcpp::Node
{
public:
  UR3CartesianMotionNode()
  : Node("ur3_cartesian_motion_node")
  {
    // Delay MoveGroup creation until this node is owned by a shared_ptr
    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&UR3CartesianMotionNode::initialize_move_group, this));
  }

private:
  void initialize_move_group()
  {
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface...");

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "ur_manipulator");
    move_group_->setPoseReferenceFrame("base_link");

    cart_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "cartesian_goal", 10,
      std::bind(&UR3CartesianMotionNode::on_goal, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Ready to receive goals on /cartesian_goal");
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto &p = msg->pose.position;
    RCLCPP_INFO(get_logger(),
      "Received goal: x=%.3f, y=%.3f, z=%.3f", p.x, p.y, p.z);

    move_group_->setPoseTarget(msg->pose);
    bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      RCLCPP_INFO(get_logger(), "Motion succeeded.");
    } else {
      RCLCPP_ERROR(get_logger(), "Motion failed.");
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cart_sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UR3CartesianMotionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// class UR3CartesianMotionNode : public rclcpp::Node
// {
// public:
//   UR3CartesianMotionNode()
//   : Node("ur3_cartesian_motion_node")
//   {
//     // MoveIt initialization
//     move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//       shared_from_this(), "ur_manipulator");
//     move_group_->setPoseReferenceFrame("base_link");

//     // Subscribe to goals
//     cart_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//       "cartesian_goal", 10,
//       std::bind(&UR3CartesianMotionNode::on_goal, this, std::placeholders::_1));
//   }

// private:
//   void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//   {
//     auto target = msg->pose;
//     RCLCPP_INFO(get_logger(),
//       "Moving to: x=%.3f y=%.3f z=%.3f",
//       target.position.x,
//       target.position.y,
//       target.position.z);

//     move_group_->setPoseTarget(target);
//     bool ok = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
//     if (ok) {
//       RCLCPP_INFO(get_logger(), "Success.");
//     } else {
//       RCLCPP_ERROR(get_logger(), "Failure.");
//     }
//   }

//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cart_sub_;
//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<UR3CartesianMotionNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }