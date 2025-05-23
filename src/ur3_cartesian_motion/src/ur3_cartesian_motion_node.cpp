#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>


class UR3CartesianMotionNode : public rclcpp::Node
{
public:
  UR3CartesianMotionNode()
  : Node("ur3_cartesian_motion_node")
  {
        this->declare_parameter("velocity_scaling", 0.1);
        this->get_parameter("velocity_scaling", velocity_scaling_);

        status_pub_ = this->create_publisher<std_msgs::msg::Int32>("ur_status", 10);
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cartesian_goal", 10,
            std::bind(&UR3CartesianMotionNode::poseCallback, this, std::placeholders::_1)
        );
    }

    void initializeMoveGroup() {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::static_pointer_cast<rclcpp::Node>(shared_from_this()), "ur_manipulator"
        );
        move_group_interface_->setMaxVelocityScalingFactor(velocity_scaling_);
        startupInitPose();
    }

private:
    void startupInitPose() {
        std::vector<double> init_joint_positions = {2.61799, -0.7854, 1.0472, -1.5708, -1.5708, 0.0};
        move_group_interface_->setJointValueTarget(init_joint_positions);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface_->plan(plan));

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Executing startup pose.");
            move_group_interface_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan startup pose.");
        }
    }

    void scaleTrajectorySpeed(moveit_msgs::msg::RobotTrajectory &trajectory, double scale) {
        for (auto &point : trajectory.joint_trajectory.points) {
            for (auto &vel : point.velocities) {
                vel *= scale;
            }
            for (auto &acc : point.accelerations) {
                acc *= scale;
            }

            int64_t total_ns = static_cast<int64_t>(
                point.time_from_start.sec * 1e9 + point.time_from_start.nanosec
            );
            int64_t scaled_ns = static_cast<int64_t>(total_ns / scale);
            point.time_from_start = rclcpp::Duration(std::chrono::nanoseconds(scaled_ns));
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!move_group_interface_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received new target pose.");
        std_msgs::msg::Int32 status_msg;
        status_msg.data = 2;  // Currently executing
        status_pub_->publish(status_msg);

        move_group_interface_->setStartStateToCurrentState();

        std::vector<geometry_msgs::msg::Pose> waypoints = {msg->pose};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_interface_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        if (fraction < 0.9) {
            RCLCPP_ERROR(this->get_logger(), "Singularity detected or planning failed (%.2f%%)", fraction * 100.0);
            status_msg.data = 3;  // Singularity
        } else {
            scaleTrajectorySpeed(trajectory, velocity_scaling_);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            auto result = move_group_interface_->execute(plan);
            status_msg.data = (result == moveit::core::MoveItErrorCode::SUCCESS) ? 1 : 4;
        }

        status_pub_->publish(status_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    double velocity_scaling_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR3CartesianMotionNode>();
    node->initializeMoveGroup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


// kinda works 
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>

// using namespace std::chrono_literals;

// class UR3CartesianMotionNode : public rclcpp::Node
// {
// public:
//   UR3CartesianMotionNode()
//   : Node("ur3_cartesian_motion_node")
//   {
//     // Delay MoveGroup creation until this node is owned by a shared_ptr
//     timer_ = this->create_wall_timer(
//       500ms,
//       std::bind(&UR3CartesianMotionNode::initialize_move_group, this));
//   }

// private:
//   void initialize_move_group()
//   {
//     timer_->cancel();
//     RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface...");

//     move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//       shared_from_this(), "ur_manipulator");
//     move_group_->setPoseReferenceFrame("base_link");

//         // *** NEW: slow down for smooth Cartesian moves ***
//     move_group_->setMaxVelocityScalingFactor(0.1);
//     move_group_->setMaxAccelerationScalingFactor(0.1);


//     cart_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//       "cartesian_goal", 10,
//       std::bind(&UR3CartesianMotionNode::on_goal, this, std::placeholders::_1));

//     RCLCPP_INFO(get_logger(), "Ready to receive goals on /cartesian_goal");
//   }

//   void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//   {
//     auto &p = msg->pose.position;
//     RCLCPP_INFO(get_logger(),
//       "Received goal: x=%.3f, y=%.3f, z=%.3f", p.x, p.y, p.z);

//     move_group_->setPoseTarget(msg->pose);
//     bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       RCLCPP_INFO(get_logger(), "Motion succeeded.");
//     } else {
//       RCLCPP_ERROR(get_logger(), "Motion failed.");
//     }
//     RCLCPP_INFO(get_logger(),
//       "Cartesian path to: x=%.3f, y=%.3f, z=%.3f", p.x, p.y, p.z);

//     // build straight‐line waypoints: current → target
//     std::vector<geometry_msgs::msg::Pose> waypoints;
//     waypoints.push_back(move_group_->getCurrentPose().pose);
//     waypoints.push_back(msg->pose);

//     // compute the Cartesian path (eef_step=1cm, jump_thresh=0.0)
//     moveit_msgs::msg::RobotTrajectory trajectory;
//     double fraction = move_group_->computeCartesianPath(
//       waypoints, 0.01, 0.0, trajectory);

//     if (fraction < 0.99) {
//       RCLCPP_WARN(get_logger(),
//         "Only %.1f%% of path planned. Skipping this goal.",
//         fraction * 100.0);
//       return;
//     }

//     // wrap & execute
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     plan.trajectory_ = trajectory;
//     move_group_->execute(plan);
//     RCLCPP_INFO(get_logger(), "Executed Cartesian path successfully.");
//   }

//   rclcpp::TimerBase::SharedPtr timer_;
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

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_msgs/msg/robot_trajectory.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>

// using namespace std::chrono_literals;

// class UR3CartesianMotionNode : public rclcpp::Node
// {
// public:
//   UR3CartesianMotionNode()
//   : Node("ur3_cartesian_motion_node")
//   {
//     // delay init so shared_from_this() works
//     timer_ = this->create_wall_timer(
//       500ms,
//       std::bind(&UR3CartesianMotionNode::initialize_move_group, this));
//   }

// private:
//   void initialize_move_group()
//   {
//     timer_->cancel();
//     RCLCPP_INFO(get_logger(), "Initializing UR3 MoveGroupInterface...");

//     move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//       shared_from_this(), "ur_manipulator");
//     move_group_->setPoseReferenceFrame("base_link");

//     // smooth motion
//     move_group_->setMaxVelocityScalingFactor(0.1);
//     move_group_->setMaxAccelerationScalingFactor(0.1);

//     // subscribe to goals from drink_memory node
//     cart_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//       "cartesian_goal", 10,
//       std::bind(&UR3CartesianMotionNode::on_goal, this, std::placeholders::_1));

//     RCLCPP_INFO(get_logger(), "Ready to receive cartesian goals on /cartesian_goal");
//   }

//   void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//   {
//     // log target
//     auto &p = msg->pose.position;
//     RCLCPP_INFO(get_logger(),
//       "Received goal → x=%.3f y=%.3f z=%.3f", p.x, p.y, p.z);

//     // build waypoints: current → target
//     std::vector<geometry_msgs::msg::Pose> waypoints;
//     waypoints.push_back(move_group_->getCurrentPose().pose);
//     waypoints.push_back(msg->pose);

//     moveit_msgs::msg::RobotTrajectory trajectory;
//     double fraction = move_group_->computeCartesianPath(
//       waypoints,
//       /*eef_step*/     0.01,
//       /*jump_threshold*/0.0,
//       trajectory);

//     if (fraction < 0.99) {
//       RCLCPP_WARN(get_logger(),
//         "Only planned %.1f%% of path; aborting.", fraction * 100.0);
//       return;
//     }

//     // wrap and execute
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     plan.trajectory_ = trajectory;
//     bool ok = (move_group_->execute(plan)
//       == moveit::core::MoveItErrorCode::SUCCESS);

//     if (ok) {
//       RCLCPP_INFO(get_logger(), "Cartesian goal executed successfully.");
//     } else {
//       RCLCPP_ERROR(get_logger(), "Cartesian goal execution failed.");
//     }
//   }

//   rclcpp::TimerBase::SharedPtr timer_;
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



// src/ur3_cartesian_motion_node.cpp this works

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>

// using namespace std::chrono_literals;

// class UR3CartesianMotionNode : public rclcpp::Node
// {
// public:
//   UR3CartesianMotionNode()
//   : Node("ur3_cartesian_motion_node")
//   {
//     // Delay MoveGroup creation until this node is owned by a shared_ptr
//     timer_ = this->create_wall_timer(
//       500ms,
//       std::bind(&UR3CartesianMotionNode::initialize_move_group, this));
//   }

// private:
//   void initialize_move_group()
//   {
//     timer_->cancel();
//     RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface...");

//     move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//       shared_from_this(), "ur_manipulator");
//     move_group_->setPoseReferenceFrame("base_link");

//     cart_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//       "cartesian_goal", 10,
//       std::bind(&UR3CartesianMotionNode::on_goal, this, std::placeholders::_1));

//     RCLCPP_INFO(get_logger(), "Ready to receive goals on /cartesian_goal");
//   }

//   void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//   {
//     auto &p = msg->pose.position;
//     RCLCPP_INFO(get_logger(),
//       "Received goal: x=%.3f, y=%.3f, z=%.3f", p.x, p.y, p.z);

//     move_group_->setPoseTarget(msg->pose);
//     bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success) {
//       RCLCPP_INFO(get_logger(), "Motion succeeded.");
//     } else {
//       RCLCPP_ERROR(get_logger(), "Motion failed.");
//     }
//   }

//   rclcpp::TimerBase::SharedPtr timer_;
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