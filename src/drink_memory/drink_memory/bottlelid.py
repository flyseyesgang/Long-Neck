import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool, Int32MultiArray
import random
import numpy as np
from scipy.spatial.transform import Rotation as R
 

 
class BottlePickerNode(Node):
    def __init__(self):
        super().__init__('bottle_picker_node')
 
        # Camera offset in end effector frame (meters)
        self.cam_offset = np.array([0.055, 0.020, 0.010])  # 55mm, 20mm, 10mm
 
        # Latest end effector pose
        self.ee_pos = None
        self.ee_rot = None
 
        # Subscribers
        self.ee_pose_sub = self.create_subscription(
            PoseStamped, '/ee_pose', self.ee_pose_callback, 10)
        self.bottle_sub = self.create_subscription(
            Float64MultiArray, '/bottle_matrix', self.bottle_callback, 10)
 
        # Publisher
        self.goal_pub = self.create_publisher(PoseStamped, '/cartesian_goal', 10)
        self.get_logger().info("Bottle Picker Node started, waiting for bottle matrix and ee pose...")
 
    def ee_pose_callback(self, msg: PoseStamped):
        # Store latest end effector world position and rotation (quaternion)
        self.ee_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        # Quaternion to rotation matrix
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        self.ee_rot = R.from_quat(quat).as_matrix()
 
    def bottle_callback(self, msg: Float64MultiArray):
        if self.ee_pos is None or self.ee_rot is None:
            self.get_logger().warn("End effector pose not received yet.")
            return
 
        if len(msg.data) < 4:
            self.get_logger().warn("No bottles detected in matrix.")
            return
 
        num_bottles = len(msg.data) // 4
        chosen_index = random.randint(0, num_bottles - 1)
        start = chosen_index * 4
        bottle_id = int(msg.data[start + 0])
        x_cam = msg.data[start + 1]
        y_cam = msg.data[start + 2]
        z_cam = msg.data[start + 3]
        bottle_cam = np.array([x_cam, y_cam, z_cam])
 
        # Transform from camera to end effector
        bottle_ee = bottle_cam + self.cam_offset
 
        # Transform from end effector to world
        bottle_world = self.ee_rot.dot(bottle_ee) + self.ee_pos
 
        self.get_logger().info(
            f"Selected bottle ID: {bottle_id}, World Coords: ({bottle_world[0]:.3f}, {bottle_world[1]:.3f}, {bottle_world[2]:.3f})"
        )
 
        # Create and publish PoseStamped in world frame
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'world'  # or your actual world frame
 
        goal_msg.pose.position.x = float(bottle_world[0])
        goal_msg.pose.position.y = float(bottle_world[1])
        goal_msg.pose.position.z = float(bottle_world[2])
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
