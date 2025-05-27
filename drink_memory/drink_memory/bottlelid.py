#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
import random
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from std_msgs.msg import Bool
 

class BottlePickerNode(Node):
    def __init__(self):
        super().__init__('bottle_picker_node')
        
        # Subscriber to the bottle coordinates matrix
        self.bottle_sub = self.create_subscription(
            PoseArray, '/bottle_lid_cartesian', self.bottle_callback, 10)

        # Publisher to send the selected bottle's coordinates to the robotic arm
        self.goal_pub = self.create_publisher(
            PoseStamped, '/cartesian_goal', 10)

        self.get_logger().info("Bottle Picker Node started, waiting for bottle data...")

    def bottle_callback(self, msg: PoseArray):
        if len(msg.poses) == 0:
            self.get_logger().warn("No bottles detected.")
            return

        # Randomly choose one of the detected bottles
        chosen_index = random.randint(0, len(msg.poses) - 1)
        chosen_pose = msg.poses[chosen_index]

        self.get_logger().info(f"Selected bottle ID: {chosen_index}, Coordinates: ({chosen_pose.position.x}, {chosen_pose.position.y}, {chosen_pose.position.z})")

        # Create and publish the PoseStamped message
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'base_link'  # Adjust to your robot's reference frame

        goal_msg.pose.position.x = chosen_pose.position.x
        goal_msg.pose.position.y = chosen_pose.position.y
        goal_msg.pose.position.z = chosen_pose.position.z

        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)
        self.get_logger().info("Published goal for robotic arm.")


def main(args=None):
    rclpy.init(args=args)
    node = BottlePickerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
