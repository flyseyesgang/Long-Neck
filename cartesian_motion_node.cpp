#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class UR3CartesianMotionNode : public rclcpp::Node
{
public:
  UR3CartesianMotionNode()
    : Node("ur3_cartesian_motion_node")
  {
    // Delay to allow MoveIt and controllers to fully initialize
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&UR3CartesianMotionNode::initialize, this));
  }

private:
  void initialize()
  {
    timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Initializing UR3 Cartesian Motion Node...");

    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "ur_manipulator");

    move_group->setPoseReferenceFrame("base_link");

    // Get the current pose as a base
    auto current_pose = move_group->getCurrentPose().pose;

    std::vector<geometry_msgs::msg::Pose> goals;

    // Goal 1
    geometry_msgs::msg::Pose pose1 = current_pose;
    pose1.position.x = 0.3;
    pose1.position.y = 0.2;
    pose1.position.z = 0.3;
    pose1.orientation = current_pose.orientation;  // keep orientation fixed
    goals.push_back(pose1);

    // Goal 2
    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.x = 0.4;
    goals.push_back(pose2);

    // Goal 3
    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.y = 0.0;
    goals.push_back(pose3);

    // Go through each pose and move there
    for (size_t i = 0; i < goals.size(); ++i)
    {
      const auto& target = goals[i];
      RCLCPP_INFO(this->get_logger(), "Moving to goal %zu: x=%.3f y=%.3f z=%.3f",
                  i + 1, target.position.x, target.position.y, target.position.z);

      move_group->setPoseTarget(target);
      bool success = (move_group->move() == moveit::core::MoveItErrorCode::SUCCESS);

      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Reached goal %zu successfully.", i + 1);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to reach goal %zu.", i + 1);
        break;
      }
    }

    RCLCPP_INFO(this->get_logger(), "All goals processed.");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UR3CartesianMotionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
