import rclpy
import random
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Bool
from geometry_msgs.msg import PoseStamped, Quaternion, Point
# from threading import Timer
import numpy as np
from scipy.spatial.transform import Rotation as R

class MemoryDecisionNode(Node):
    def __init__(self):
        super().__init__('memory_decision_node')

        # --- Camera offset in end effector frame (meters) ---
        self.cam_offset = np.array([0.055, 0.020, 0.010])  # 55mm, 20mm, 10mm

        # --- Internal memory structures ---
        self.ee_pos = None
        self.ee_rot = None
        self.bottle_matrix = {}        # {id: (x_cam, y_cam, z_cam)}
        self.empty_ids = set()
        self.wrong_ids = set()
        self.id_to_drink = {}          # {id: drink}
        self.current_target = None     # requested drink string
        self._last_picked_id = None    # For associating scan results

        self._last_pick_pose = None   # Save world pose before scanning
        self._after_scan_pose = None  # Save where to return after scan
        self._awaiting_scan = False   # Flag: waiting for /brand_label

        self.esky_found = False
        self.esky_position = None   

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'base_link'         # Change as appropriate
        self.ee_frame = 'onrobot_base_link'   # Change as appropriate


        # --- Subscribers ---
        # self.ee_pose_sub = self.create_subscription(
        #     PoseStamped, '/ee_pose', self.ee_pose_callback, 10)
        self.bottle_matrix_sub = self.create_subscription(
            Float64MultiArray, '/bottle_matrix', self.bottle_matrix_callback, 10)
        self.requested_drink_sub = self.create_subscription(
            String, 'requested_drink', self.request_cb, 10)
        # self.scanned_drink_sub = self.create_subscription(
        #     String, 'scanned_drink', self.scan_cb, 10)
        self.brand_label_sub = self.create_subscription(
            String, '/brand_label', self.scan_cb, 10)

        self.esky_found_sub = self.create_subscription(
            Bool, '/esky/found', self.esky_found_cb, 10
        )
        self.esky_position_sub = self.create_subscription(
            Point, '/esky/position', self.esky_position_cb, 10
        )
        # --- Publisher ---
        self.goal_pub = self.create_publisher(PoseStamped, '/cartesian_goal', 10)

        # self.gripper_pub = self.create_publisher(Float64, '/gripper_width', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/finger_width_controller/commands', 10)

        self.move_to_wait_pose()
        self.waiting_for_esky = True


        self.get_logger().info("MemoryDecisionNode started. Waiting for drink requests and bottle data...")

    def move_to_wait_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = 0.0
        pose.pose.position.y = -0.3
        pose.pose.position.z = 0.2
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        self.goal_pub.publish(pose)
        self.get_logger().info("Moved to initial waiting pose (0, -0.3, 0.2)")


    def get_ee_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())
            pos = np.array([t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z])
            quat = [t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w]
            rot = R.from_quat(quat).as_matrix()
            return pos, rot
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("TF: End effector pose not available yet.")
            return None, None

    def esky_found_cb(self, msg: Bool):
        if msg.data and self.waiting_for_esky:
            self.esky_found = True
            self.get_logger().info("Esky detected by scanner.")
            self.waiting_for_esky = False
            # You may want to do something here, like log or move to next state

    def esky_position_cb(self, msg: Point):
        self.esky_position = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f"Esky position updated: {self.esky_position}")

    # def ee_pose_callback(self, msg: PoseStamped):
    #     # Store latest end effector world position and rotation (quaternion)
    #     self.ee_pos = np.array([
    #         msg.pose.position.x,
    #         msg.pose.position.y,
    #         msg.pose.position.z
    #     ])
    #     quat = [
    #         msg.pose.orientation.x,
    #         msg.pose.orientation.y,
    #         msg.pose.orientation.z,
    #         msg.pose.orientation.w
    #     ]
    #     self.ee_rot = R.from_quat(quat).as_matrix()
        

    def bottle_matrix_callback(self, msg: Float64MultiArray):
        # Update the internal matrix: {id: (x_cam, y_cam, z_cam)}
        self.bottle_matrix = {}
        for i in range(len(msg.data)//4):
            id = int(msg.data[i*4])
            x, y, z = msg.data[i*4+1:i*4+4]
            self.bottle_matrix[id] = np.array([x, y, z])
        # Re-attempt bottle selection if we're waiting on a request
        if self.current_target:
            self.decide_next_bottle()

    # def request_cb(self, msg: String):
    #     # New drink requested
    #     self.current_target = msg.data.strip().lower()
    #     self.get_logger().info(f"Requested drink: {self.current_target}")
    #     self.decide_next_bottle()
    def request_cb(self, msg: String):
        if self.waiting_for_esky:
            self.get_logger().info("Waiting for esky to be found before accepting drink requests.")
            return
        self.current_target = msg.data.strip().lower()
        self.get_logger().info(f"Requested drink: {self.current_target}")
        self.decide_next_bottle()


    # def scan_cb(self, msg: String):
    #     if self._awaiting_scan:
    #         self._awaiting_scan = False
    #         # Return to the after-dropoff pose
    #         if self._after_scan_pose:
    #             self.goal_pub.publish(self._after_scan_pose)
    #             self.get_logger().info("Returning to post-drop-off pose after scan.")
    #         # A scan result comes back after a pick; update memory accordingly
    #         result = msg.data.strip().lower()
    #         if self._last_picked_id is None:
    #             self.get_logger().warn('Scan result received without context of a last picked ID.')
    #             return
    #         checked_id = self._last_picked_id
    #         if result == 'unknown':
    #             self.empty_ids.add(checked_id)
    #             self.id_to_drink.pop(checked_id, None)
    #             self.get_logger().info(f"Bottle ID {checked_id} marked empty.")
    #         elif result != self.current_target:
    #             self.wrong_ids.add(checked_id)
    #             self.id_to_drink[checked_id] = result
    #             self.get_logger().info(f"Bottle ID {checked_id} holds '{result}', not '{self.current_target}'. Marked as wrong.")
    #         else:
    #             # Correct drink found; mark ID as empty and clear memory
    #             self.empty_ids.add(checked_id)
    #             self.id_to_drink.pop(checked_id, None)
    #             self.get_logger().info(f"Found and removed '{self.current_target}' (ID {checked_id}) from esky.")
    #         # After handling, try the next bottle if a request is active
    #         if self.current_target:
    #             self.decide_next_bottle()
    def scan_cb(self, msg: String):
        if self._awaiting_scan:
            self._awaiting_scan = False
            if self._after_scan_pose:
                self.goal_pub.publish(self._after_scan_pose)
                self.get_logger().info("Returning to post-drop-off pose after scan.")
            result = msg.data.strip().lower()
            if self._last_picked_id is None:
                self.get_logger().warn('Scan result received without context of a last picked ID.')
                return
            checked_id = self._last_picked_id
            if result == 'unknown':
                self.empty_ids.add(checked_id)
                self.id_to_drink.pop(checked_id, None)
                self.get_logger().info(f"Bottle ID {checked_id} marked empty.")
            elif result != self.current_target:
                self.wrong_ids.add(checked_id)
                self.id_to_drink[checked_id] = result
                self.get_logger().info(f"Bottle ID {checked_id} holds '{result}', not '{self.current_target}'. Marked as wrong.")
            else:
                # Correct drink found; mark as empty but KEEP in id_to_drink for memory testing
                self.empty_ids.add(checked_id)
                # self.id_to_drink.pop(checked_id, None)  # <--- comment this to test memory
                self.get_logger().info(f"Found and removed '{self.current_target}' (ID {checked_id}) from esky.")
            self.get_logger().info(f"id_to_drink (memory): {self.id_to_drink}")
            if self.current_target:
                self.decide_next_bottle()


    def decide_next_bottle(self):
        ee_pos, ee_rot = self.get_ee_pose()
        if ee_pos is None or ee_rot is None:
            self.get_logger().warn("End effector pose not received yet, cannot select bottle.")
            return
        if not self.bottle_matrix:
            self.get_logger().warn("No bottle data available.")
            return

        # 1. PRIORITY: Pick known-matching bottle if in memory and not empty
        for id, drink in self.id_to_drink.items():
            if (
                drink == self.current_target and
                id in self.bottle_matrix and
                id not in self.empty_ids
            ):
                self.get_logger().info(f"Prioritising bottle ID {id} already known to have '{self.current_target}'.")
                self.publish_goal(id, self.bottle_matrix[id], ee_pos, ee_rot)
                return

        # 2. Log ignored bottles
        ignored_empty = [id for id in self.bottle_matrix if id in self.empty_ids]
        if ignored_empty:
            self.get_logger().info(f"Ignoring bottle IDs marked as empty: {ignored_empty}")

        ignored_wrong = [
            id for id in self.bottle_matrix
            if id in self.wrong_ids and self.id_to_drink.get(id) != self.current_target
        ]
        if ignored_wrong:
            self.get_logger().info(f"Ignoring bottle IDs marked as wrong drink: {ignored_wrong}")

        # 3. Build fallback candidate list
        candidates = []
        for id, pos in self.bottle_matrix.items():
            if id in self.empty_ids:
                continue
            if (id in self.wrong_ids) and (self.id_to_drink.get(id) != self.current_target):
                continue
            candidates.append((id, pos))

        candidate_ids = [id for id, _ in candidates]
        self.get_logger().info(f"Candidate bottle IDs after filtering: {candidate_ids}")

        # 4. Fallback: random pick
        if candidates:
            selected_id, selected_pos = random.choice(candidates)
            self.get_logger().info(f"Randomly selected bottle ID {selected_id} for drink '{self.current_target}'.")
            self.publish_goal(selected_id, selected_pos, ee_pos, ee_rot)
        else:
            self.get_logger().warn("No available bottles match the criteria for this drink.")
            self._last_picked_id = None

    # def decide_next_bottle(self):
    #     ee_pos, ee_rot = self.get_ee_pose()
    #     if ee_pos is None or ee_rot is None:
    #         self.get_logger().warn("End effector pose not received yet, cannot select bottle.")
    #         return
    #     # if self.ee_pos is None or self.ee_rot is None:
    #     #     self.get_logger().warn("End effector pose not received yet, cannot select bottle.")
    #     #     return
    #     if not self.bottle_matrix:
    #         self.get_logger().warn("No bottle data available.")
    #         return
        
    #     ignored_empty = [id for id in self.bottle_matrix if id in self.empty_ids]
    #     if ignored_empty:
    #         self.get_logger().info(f"Ignoring bottle IDs marked as empty: {ignored_empty}")

    #      # Show which IDs are being ignored as wrong
    #     ignored_wrong = [
    #         id for id in self.bottle_matrix
    #         if id in self.wrong_ids and self.id_to_drink.get(id) != self.current_target
    #     ]
    #     # ignored_wrong = [id for id in self.bottle_matrix if id in self.wrong_ids]
    #     if ignored_wrong:
    #         self.get_logger().info(f"Ignoring bottle IDs marked as wrong drink: {ignored_wrong}")

    #     # # Step 1: Build filtered candidate list (exclude empties/wrongs)
    #     # candidates = [
    #     #     (id, pos) for id, pos in self.bottle_matrix.items()
    #     #     if id not in self.empty_ids and id not in self.wrong_ids
    #     # ]
    #     # candidate_ids = [id for id, _ in candidates]
    #     # self.get_logger().info(f"Candidate bottle IDs after filtering: {candidate_ids}")
    #     # Step 1: Build filtered candidate list
    #     candidates = []
    #     for id, pos in self.bottle_matrix.items():
    #         # Allow IDs that are not empty and either:
    #         #  - not in wrong_ids, OR
    #         #  - in wrong_ids but now match current_target in id_to_drink
    #         if id in self.empty_ids:
    #             continue
    #         if (id in self.wrong_ids) and (self.id_to_drink.get(id) != self.current_target):
    #             continue
    #         candidates.append((id, pos))

    #     # # Step 2: Prioritise known location if in memory
    #     # for id, drink in self.id_to_drink.items():
    #     #     if drink == self.current_target and id in self.bottle_matrix and id not in self.empty_ids:
    #     #         self.get_logger().info(f"Prioritising bottle ID {id} already known to have '{self.current_target}'.")
    #     #         # self.publish_goal(id, self.bottle_matrix[id])
    #     #         self.publish_goal(id, self.bottle_matrix[id], ee_pos, ee_rot)
    #     #         return
    #     prioritised_candidates = [
    #         (id, pos) for id, pos in candidates if self.id_to_drink.get(id) == self.current_target
    #     ]
    #     if prioritised_candidates:
    #         # If there are multiple, you could random.choice, but usually just one.
    #         id, pos = prioritised_candidates[0]
    #         self.get_logger().info(f"Prioritising bottle ID {id} already known to have '{self.current_target}'.")
    #         self.publish_goal(id, pos, ee_pos, ee_rot)
    #         return

    #     # Step 3: Fallback—pick any candidate (here: first, but could randomise/optimise)
    #     # if candidates:
    #     #     # You could also use: selected_id, selected_pos = random.choice(candidates)
    #     #     selected_id, selected_pos = candidates[0]
    #     #     self.publish_goal(selected_id, selected_pos)
    #     # else:
    #     #     self.get_logger().warn("No available bottles match the criteria for this drink.")
    #     #     self._last_picked_id = None  # Don't leave dangling state
    #     if candidates:
    #         selected_id, selected_pos = random.choice(candidates)
    #         self.get_logger().info(f"Randomly selected bottle ID {selected_id} for drink '{self.current_target}'.")
    #         # self.publish_goal(selected_id, selected_pos)
    #         self.publish_goal(selected_id, selected_pos, ee_pos, ee_rot)
    #     else:
    #         self.get_logger().warn("No available bottles match the criteria for this drink.")
    #         self._last_picked_id = None 

    # def publish_goal(self, id, bottle_cam):
    #     # Transform bottle_cam to end effector frame
    #     bottle_ee = bottle_cam + self.cam_offset
    #     # Transform to world frame
    #     bottle_world = self.ee_rot.dot(bottle_ee) + self.ee_pos
    def publish_goal(self, id, bottle_cam, ee_pos, ee_rot):
        bottle_ee = bottle_cam + self.cam_offset
        bottle_world = ee_rot.dot(bottle_ee) + ee_pos
        # Build PoseStamped
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        # goal_msg.header.frame_id = 'world'
        goal_msg.header.frame_id = self.base_frame
        goal_msg.pose.position.x = float(bottle_world[0])
        goal_msg.pose.position.y = float(bottle_world[1])
        goal_msg.pose.position.z = float(bottle_world[2])
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)
        self._last_picked_id = id
        self.get_logger().info(
            f"Published goal for bottle ID {id}, drink='{self.current_target}', coords (world): {tuple(bottle_world)}"
        )
        self._last_pick_pose = goal_msg  
        self.do_pick_and_place_sequence(bottle_world) #### not sure, might need to change
        
    def do_pick_and_place_sequence(self, bottle_world):
        # Step 1: Close gripper
        close_msg = Float64MultiArray()
        close_msg.data = [0.032]
        self.gripper_pub.publish(close_msg)
        self.get_logger().info("Step 1: Gripper closing to 0.032.")

        # Step 2: Lift bottle
        lift_pose = PoseStamped()
        lift_pose.header.stamp = self.get_clock().now().to_msg()
        lift_pose.header.frame_id = 'world'
        lift_pose.pose.position.x = bottle_world[0]
        lift_pose.pose.position.y = bottle_world[1]
        lift_pose.pose.position.z = bottle_world[2] + 0.15
        lift_pose.pose.orientation.x = 0.0
        lift_pose.pose.orientation.y = 0.0
        lift_pose.pose.orientation.z = 0.0
        lift_pose.pose.orientation.w = 1.0
        self.goal_pub.publish(lift_pose)
        self.get_logger().info("Step 2: Lifted bottle by 0.15m.")

        # Step 3: Move to drop-off
        drop_pose = PoseStamped()
        drop_pose.header.stamp = self.get_clock().now().to_msg()
        drop_pose.header.frame_id = 'world'
        drop_pose.pose.position.x = -0.2
        drop_pose.pose.position.y = -0.2
        drop_pose.pose.position.z = 0.133
        drop_pose.pose.orientation.x = 0.0
        drop_pose.pose.orientation.y = 0.0
        drop_pose.pose.orientation.z = 0.0
        drop_pose.pose.orientation.w = 1.0
        self.goal_pub.publish(drop_pose)
        self.get_logger().info("Step 3: Moved to drop-off location.")

        # Step 4: Open gripper
        open_msg = Float64MultiArray()
        open_msg.data = [0.09]
        self.gripper_pub.publish(open_msg)
        self.get_logger().info("Step 4: Gripper opened to 0.09.")

        # Step 5: Move to scanning pose
        scan_pose = PoseStamped()
        scan_pose.header.stamp = self.get_clock().now().to_msg()
        scan_pose.header.frame_id = 'world'
        scan_pose.pose.position.x = 0.0
        scan_pose.pose.position.y = -0.2
        scan_pose.pose.position.z = 0.1
        scan_pose.pose.orientation.x = 0.0
        scan_pose.pose.orientation.y = 1.0
        scan_pose.pose.orientation.z = 0.0
        scan_pose.pose.orientation.w = 0.0
        self.goal_pub.publish(scan_pose)
        self.get_logger().info("Step 5: Moved to scanning pose (sideways). Waiting for /brand_label...")

        self._awaiting_scan = True  # Set flag to wait for scan

    # def do_pick_and_place_sequence(self, bottle_world):
    #     # close_msg = Float64()
    #     # close_msg.data = 0.032
    #     # self.gripper_pub.publish(close_msg)
    #     close_msg = Float64MultiArray()
    #     close_msg.data = [0.032]  # For closing
    #     self.gripper_pub.publish(close_msg)
        
    #     self.get_logger().info("Step 1: Gripper closing to 0.032.")

    #     def lift_bottle(_):
    #         lift_pose = PoseStamped()
    #         lift_pose.header.stamp = self.get_clock().now().to_msg()
    #         lift_pose.header.frame_id = 'world'
    #         lift_pose.pose.position.x = bottle_world[0]
    #         lift_pose.pose.position.y = bottle_world[1]
    #         lift_pose.pose.position.z = bottle_world[2] + 0.15
    #         lift_pose.pose.orientation.x = 0.0
    #         lift_pose.pose.orientation.y = 0.0
    #         lift_pose.pose.orientation.z = 0.0
    #         lift_pose.pose.orientation.w = 1.0
    #         self.goal_pub.publish(lift_pose)
    #         self.get_logger().info("Step 2: Lifted bottle by 0.15m.")
            
    #         def move_to_dropoff(_):
    #             drop_pose = PoseStamped()
    #             drop_pose.header.stamp = self.get_clock().now().to_msg()
    #             drop_pose.header.frame_id = 'world'
    #             drop_pose.pose.position.x = -0.2
    #             drop_pose.pose.position.y = -0.2
    #             drop_pose.pose.position.z = 0.133
    #             drop_pose.pose.orientation.x = 0.0
    #             drop_pose.pose.orientation.y = 0.0
    #             drop_pose.pose.orientation.z = 0.0
    #             drop_pose.pose.orientation.w = 1.0
    #             self.goal_pub.publish(drop_pose)
    #             self.get_logger().info("Step 3: Moved to drop-off location.")

    #             def open_and_scan(_):
    #                 # open_msg = Float64()
    #                 # open_msg.data = 0.09
    #                 # self.gripper_pub.publish(open_msg)
    #                 open_msg = Float64MultiArray()
    #                 open_msg.data = [0.09]  # For opening
    #                 self.gripper_pub.publish(open_msg)

    #                 self.get_logger().info("Step 4: Gripper opened to 0.09.")

    #                 # Save the drop-off pose to return to after scan
    #                 self._after_scan_pose = PoseStamped()
    #                 self._after_scan_pose.header.stamp = self.get_clock().now().to_msg()
    #                 self._after_scan_pose.header.frame_id = 'world'
    #                 self._after_scan_pose.pose.position.x = -0.2
    #                 self._after_scan_pose.pose.position.y = -0.2
    #                 self._after_scan_pose.pose.position.z = 0.133
    #                 self._after_scan_pose.pose.orientation.x = 0.0
    #                 self._after_scan_pose.pose.orientation.y = 0.0
    #                 self._after_scan_pose.pose.orientation.z = 0.0
    #                 self._after_scan_pose.pose.orientation.w = 1.0

    #                 # Move to scanning pose (0, -0.2, 0.1), with orientation facing -x
    #                 scan_pose = PoseStamped()
    #                 scan_pose.header.stamp = self.get_clock().now().to_msg()
    #                 scan_pose.header.frame_id = 'world'
    #                 scan_pose.pose.position.x = 0.0
    #                 scan_pose.pose.position.y = -0.2
    #                 scan_pose.pose.position.z = 0.1
    #                 # Orientation: -x (sideways), for UR: x=0, y=1, z=0, w=0 gives 180 deg about Y (flip x)
    #                 scan_pose.pose.orientation.x = 0.0
    #                 scan_pose.pose.orientation.y = 1.0
    #                 scan_pose.pose.orientation.z = 0.0
    #                 scan_pose.pose.orientation.w = 0.0
    #                 self.goal_pub.publish(scan_pose)
    #                 self.get_logger().info("Step 5: Moved to scanning pose (sideways). Waiting for /brand_label...")

    #                 self._awaiting_scan = True  # Set flag to wait for scan

    #     #         Timer(1.5, open_and_scan, args=[None]).start()
    #     #     Timer(1.5, move_to_dropoff, args=[None]).start()
    #     # Timer(1.0, lift_bottle, args=[None]).start()


def main(args=None):
    rclpy.init(args=args)
    node = MemoryDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




# # This should hopefully be able to be integrated with Rhys's Pieces
# # !/usr/bin/env python3
# """
# ROS2 node for drink memory and search strategy.
# Supports test steps 1-5:
#  1: Avoid empty slots.
#  2: Avoid wrong slots.
#  3: Seek previously found locations (also includes 1 & 2).
#  4: Efficiently update moved drinks (not implemented here yet).
#  5: Detect empty esky and request restock.
# """
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Int32
# from geometry_msgs.msg import PoseStamped
# from std_srvs.srv import Empty
# from std_msgs.msg import Bool
 

# class MemoryDecisionNode(Node):
#     def __init__(self):
#         super().__init__('drink_memory')
#         # Decision logic parameters
#         self.declare_parameter('test_step', 1)
#         self.test_step = self.get_parameter('test_step').value

#         self.esky_found = False
#         self.create_subscription(
#             Bool, 'esky_found', self.esky_found_cb, 10)


#         # Dynamic visual data (initially None or empty)
#         self.total_slots = None            # will be set by esky_slot_count subscriber
#         self.slot_coords = {}              # slot_id -> (x,y,z)
#         self.dropoff_coord = None          # (x,y,z)

#         # Internal memory structures
#         self.empty_slots = set()
#         self.checked_wrong = set()
#         self.drink_locations = {}
#         self.current_target = None
#         self.checked_this_request = set()

#         # Publishers
#         self.next_slot_pub = self.create_publisher(Int32, 'next_slot', 10)
#         self.restock_pub   = self.create_publisher(String, 'restock_request', 10)
#         # self.cart_goal_pub = self.create_publisher(PoseStamped, 'cartesian_goal', 10)
#         self.cart_goal_pub = self.create_publisher(PoseStamped, '/cartesian_goal', 10)

#         # Subscribers: decision inputs
#         self.create_subscription(String, 'requested_drink', self.request_cb, 10)
#         self.create_subscription(Int32,  'scanned_slot',     self.slot_cb,   10)
#         self.create_subscription(String, 'scanned_drink',    self.scan_cb,   10)

#         # Subscribers: visual data
#         self.create_subscription(Int32,     'esky_slot_count', self.slot_count_cb, 10)
#         self.create_subscription(PoseStamped, 'esky_slot_pose',  self.slot_pose_cb,  10)
#         self.create_subscription(PoseStamped, 'dropoff_location', self.dropoff_cb,   10)

#         ## New subscriber for overall esky origin pose
#         self.create_subscription(PoseStamped, 'esky_pose', self.esky_pose_cb, 10)

#         # Service client to trigger the esky scan motion
#         self.scan_client = self.create_client(Empty, 'scan_for_esky')

#         self.get_logger().info(
#             f'DrinkMemoryNode started (test_step={self.test_step}), waiting for visual data...'
#         )
#         # Kick off the esky search at startup
#         self.on_startup()
    
#     def on_startup(self):
#         """Call the UR3 node to sweep the camera and publish esky poses."""
#         if self.esky_found:
#             return
#         if not self.scan_client.wait_for_service(timeout_sec=2.0):
#             self.get_logger().error('scan_for_esky service not available!')
#             return
#         req = Empty.Request()
#         self.scan_client.call_async(Empty.Request())
#         self.get_logger().info('Requested esky scan.')
#         # self.scan_client.call_async(req)
#         # self.get_logger().info('Requested esky scan.')

#     def esky_found_cb(self, msg: Bool):
#         self.esky_found = msg.data
#         if self.esky_found:
#             self.get_logger().info("Esky located—unlocking search logic.")

#     def slot_count_cb(self, msg: Int32):
#         self.total_slots = msg.data
#         # self.esky_found = True
#         self.get_logger().info(f"Total slots updated: {self.total_slots}")

#     def dropoff_cb(self, msg: PoseStamped):
#         self.dropoff_coord = (
#             msg.pose.position.x,
#             msg.pose.position.y,
#             msg.pose.position.z
#         )
#         self.get_logger().info(f"Drop-off coord set to {self.dropoff_coord}")

#     def slot_pose_cb(self, msg: PoseStamped):
#         # if the header or a field encodes which slot:
#         slot_id = int(msg.header.frame_id)  # or use msg.slot_index if custom
#         self.slot_coords[slot_id] = (
#             msg.pose.position.x,
#             msg.pose.position.y,
#             msg.pose.position.z
#         )
#         self.get_logger().info(f"Slot {slot_id} pose updated: {self.slot_coords[slot_id]}")

#     def esky_pose_cb(self, msg: PoseStamped):
#         self.esky_origin = (
#             msg.pose.position.x,
#             msg.pose.position.y,
#             msg.pose.position.z
#         )
#         self.get_logger().info(f"Esky origin set to: {self.esky_origin}")

#     def move_to(self, coord):########
#         """
#         Pseudo-interface for UR3 movement to a Cartesian goal.
#         In real integration, replace this with action client calls.
#         """
#         """Publish a PoseStamped that offsets by esky origin."""
#         if self.esky_origin:
#             x = self.esky_origin[0] + coord[0]
#             y = self.esky_origin[1] + coord[1]
#             z = self.esky_origin[2] + coord[2]
#         else:
#             x, y, z = coord
#         # self.get_logger().info(f'[UR3] Pseudo-moving to {coord}')
#         msg = PoseStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'base_link'    # match your MoveIt frame
#         msg.pose.position.x = x
#         msg.pose.position.y = y
#         msg.pose.position.z = z
#         msg.pose.orientation.x = 0.0
#         msg.pose.orientation.y = 0.0
#         msg.pose.orientation.z = 0.0
#         msg.pose.orientation.w = 1.0

#         self.cart_goal_pub.publish(msg)
#         self.get_logger().info(f'Published cartesian_goal: ({x:.3f}, {y:.3f}, {z:.3f})')
#         # msg.pose.position.x = coord[0]
#         # msg.pose.position.y = coord[1]
#         # msg.pose.position.z = coord[2]
#         # # keep orientation fixed (you could parameterise this)
#         # msg.pose.orientation.x = 0.0
#         # msg.pose.orientation.y = 0.0
#         # msg.pose.orientation.z = 0.0
#         # msg.pose.orientation.w = 1.0

#         # self.cart_goal_pub.publish(msg)
#         # self.get_logger().info(f'Published cartesian_goal: {coord}')

#     def request_cb(self, msg: String):
#         """
#         Handle new drink request: reset per-request memory and start search.
#         """
#         self.current_target = msg.data.strip().lower()
#         self.checked_this_request.clear()
#         self.get_logger().info(f'Received request for "{self.current_target}"')
#         self.decide_next_slot()

#     def slot_cb(self, msg: Int32):
#         """
#         Record the slot number being scanned for context.
#         """
#         self._last_scanned_slot = msg.data

#     def scan_cb(self, msg: String):
#         """
#         Process scan result: update memory, move UR3, and conclude.
#         """
#         if not hasattr(self, '_last_scanned_slot'):
#             self.get_logger().warn('Scan result without slot context')
#             return

#         slot = self._last_scanned_slot
#         result = msg.data.strip().lower()

#         if result == 'unknown':
#             # Mark slot empty and clear any stale mapping
#             self.empty_slots.add(slot)
#             for drink, loc in list(self.drink_locations.items()):
#                 if loc == slot:
#                     del self.drink_locations[drink]
#             self.get_logger().info(f'Slot {slot} marked empty')
#             self.decide_next_slot()

#         elif result != self.current_target:
#             # Wrong drink: record and store its location
#             self.checked_wrong.add(slot)
#             self.checked_this_request.add(slot)
#             self.drink_locations[result] = slot
#             self.get_logger().info(f'Slot {slot} contains "{result}", not "{self.current_target}"')
#             self.decide_next_slot()

#         else:
#             # Correct drink found: move to slot and drop-off
#             coord = self.slot_coords.get(slot, (0,0,0))
#             self.move_to(coord)              # move to pickup
#             self.move_to(self.dropoff_coord) # move to drop-off
#             self.empty_slots.add(slot)       # now empty
#             self.get_logger().info(f'Found and removed "{self.current_target}" from slot {slot}')
#             # Wait for next request (no further search)

#     def decide_next_slot(self):
#         """
#         Determine next slot to check based on test_step logic. Publish and move.
#         """
#         if not self.esky_found:
#             self.get_logger().warn("Waiting for esky detection—no slot checks yet.")
#             return
#         # Test 3+: prioritise known location if still valid
#         if self.test_step >= 3 and self.current_target in self.drink_locations:
#             known = self.drink_locations[self.current_target]
#             if known not in self.empty_slots:
#                 self.get_logger().info(f'[*] Prioritising known slot {known} for "{self.current_target}"')
#                 self.publish_slot(known)
#                 return
#             else:
#                 del self.drink_locations[self.current_target]

#         # Build candidate list
#         candidates = list(range(1, self.total_slots + 1))

#         # Test 1+: avoid emptied slots
#         if self.test_step >= 1:
#             empties = [s for s in candidates if s in self.empty_slots]
#             if empties:
#                 self.get_logger().info(f'Ignoring emptied slots: {empties}')
#             candidates = [s for s in candidates if s not in self.empty_slots]

#         # Test 2+: avoid known wrong slots
#         if self.test_step >= 2:
#             wrongs = [s for s in candidates if s in self.checked_wrong]
#             if wrongs:
#                 self.get_logger().info(f'Ignoring known wrong slots: {wrongs}')
#             candidates = [s for s in candidates if s not in self.checked_wrong]

#         # Handle no candidates
#         if not candidates:
#             if self.test_step >= 5:
#                 req = String()
#                 req.data = f'Restock needed for "{self.current_target}"'
#                 self.restock_pub.publish(req)
#                 self.get_logger().warn('No slots left -> requesting restock')
#             else:
#                 self.get_logger().warn('No remaining candidates; halting search')
#             return

#         # Pick first candidate
#         self.publish_slot(candidates[0])

#     def publish_slot(self, slot: int):
#         """
#         Publish next slot to check, and pseudo-move UR3 there.
#         """
#         coord = self.slot_coords.get(slot, (0,0,0))
#         self.get_logger().info(f'-> Next slot to check: {slot} (coord={coord})')
#         msg = Int32(data=slot)
#         self.next_slot_pub.publish(msg)
#         self.move_to(coord)

# def main(args=None):
#     rclpy.init(args=args)
#     node = MemoryDecisionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

##########################################################################################################################################
# This needs to be tested on real robot
#!/usr/bin/env python3
# """
# ROS2 node for drink memory and search strategy.
# Supports test steps 1-5:
#  1: Avoid empty slots.
#  2: Avoid wrong slots.
#  3: Seek previously found locations (also includes 1 & 2).
#  4: Efficiently update moved drinks (not implemented here yet).
#  5: Detect empty esky and request restock.
# """
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Int32
# from geometry_msgs.msg import PoseStamped
 

# class MemoryDecisionNode(Node):
#     def __init__(self):
#         super().__init__('drink_memory')
#         # Declare ROS parameters
#         self.declare_parameter('total_slots', 9)  # default 3x3 grid
#         self.declare_parameter('test_step', 1)    # test step 1..5
#         self.total_slots = self.get_parameter('total_slots').value
#         self.test_step = self.get_parameter('test_step').value

#         # Placeholder coordinates for each slot (3x3 grid)
#         # Format: slot_number: (x, y, z) in robot frame
#         self.slot_coords = {
#             1: (0.1, 0.1, 0.0), 2: (0.2, 0.1, 0.0), 3: (0.3, 0.1, 0.0),
#             4: (0.1, 0.2, 0.0), 5: (0.2, 0.2, 0.0), 6: (0.3, 0.2, 0.0),
#             7: (0.1, 0.3, 0.0), 8: (0.2, 0.3, 0.0), 9: (0.3, 0.3, 0.0),
   
#         }
#         # Placeholder drop-off coordinate
#         self.dropoff_coord = (0.5, 0.0, 0.2)

#         # Internal memory structures
#         self.empty_slots = set()      # slots known empty or retrieved
#         self.checked_wrong = set()    # slots checked but contained wrong drinks
#         self.drink_locations = {}     # mapping drink_name -> slot
#         self.current_target = None    # drink currently being searched
#         self.checked_this_request = set()  # slots checked for the current request

#         # Publishers for slot decisions and restock requests
#         self.next_slot_pub = self.create_publisher(Int32, 'next_slot', 10)
#         self.restock_pub   = self.create_publisher(String, 'restock_request', 10)
#         self.cart_goal_pub = self.create_publisher(PoseStamped, '/cartesian_goal', 10)

#         # Subscribers for requests and scan results
#         self.create_subscription(String, 'requested_drink', self.request_cb, 10)
#         self.create_subscription(Int32,  'scanned_slot',     self.slot_cb,   10)
#         self.create_subscription(String, 'scanned_drink',    self.scan_cb,   10)

#         self.get_logger().info(f'MemoryDecisionNode started (step={self.test_step}, slots={self.total_slots})')

#     def move_to(self, coord):########
#         """
#         Pseudo-interface for UR3 movement to a Cartesian goal.
#         In real integration, replace this with action client calls.
#         """
#         # self.get_logger().info(f'[UR3] Pseudo-moving to {coord}')
#         msg = PoseStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'base_link'    # match your MoveIt frame
#         msg.pose.position.x = coord[0]
#         msg.pose.position.y = coord[1]
#         msg.pose.position.z = coord[2]
#         # keep orientation fixed (you could parameterise this)
#         msg.pose.orientation.x = 0.0
#         msg.pose.orientation.y = 0.0
#         msg.pose.orientation.z = 0.0
#         msg.pose.orientation.w = 1.0

#         self.cart_goal_pub.publish(msg)
#         self.get_logger().info(f'Published cartesian_goal: {coord}')

#     def request_cb(self, msg: String):
#         """
#         Handle new drink request: reset per-request memory and start search.
#         """
#         self.current_target = msg.data.strip().lower()
#         self.checked_this_request.clear()
#         self.get_logger().info(f'Received request for "{self.current_target}"')
#         self.decide_next_slot()

#     def slot_cb(self, msg: Int32):
#         """
#         Record the slot number being scanned for context.
#         """
#         self._last_scanned_slot = msg.data

#     def scan_cb(self, msg: String):
#         """
#         Process scan result: update memory, move UR3, and conclude.
#         """
#         if not hasattr(self, '_last_scanned_slot'):
#             self.get_logger().warn('Scan result without slot context')
#             return

#         slot = self._last_scanned_slot
#         result = msg.data.strip().lower()

#         if result == 'unknown':
#             # Mark slot empty and clear any stale mapping
#             self.empty_slots.add(slot)
#             for drink, loc in list(self.drink_locations.items()):
#                 if loc == slot:
#                     del self.drink_locations[drink]
#             self.get_logger().info(f'Slot {slot} marked empty')
#             self.decide_next_slot()

#         elif result != self.current_target:
#             # Wrong drink: record and store its location
#             self.checked_wrong.add(slot)
#             self.checked_this_request.add(slot)
#             self.drink_locations[result] = slot
#             self.get_logger().info(f'Slot {slot} contains "{result}", not "{self.current_target}"')
#             self.decide_next_slot()

#         else:
#             # Correct drink found: move to slot and drop-off
#             coord = self.slot_coords.get(slot, (0,0,0))
#             self.move_to(coord)              # move to pickup
#             self.move_to(self.dropoff_coord) # move to drop-off
#             self.empty_slots.add(slot)       # now empty
#             self.get_logger().info(f'Found and removed "{self.current_target}" from slot {slot}')
#             # Wait for next request (no further search)

#     def decide_next_slot(self):
#         """
#         Determine next slot to check based on test_step logic. Publish and move.
#         """
#         # Test 3+: prioritise known location if still valid
#         if self.test_step >= 3 and self.current_target in self.drink_locations:
#             known = self.drink_locations[self.current_target]
#             if known not in self.empty_slots:
#                 self.get_logger().info(f'[*] Prioritising known slot {known} for "{self.current_target}"')
#                 self.publish_slot(known)
#                 return
#             else:
#                 del self.drink_locations[self.current_target]

#         # Build candidate list
#         candidates = list(range(1, self.total_slots + 1))

#         # Test 1+: avoid emptied slots
#         if self.test_step >= 1:
#             empties = [s for s in candidates if s in self.empty_slots]
#             if empties:
#                 self.get_logger().info(f'Ignoring emptied slots: {empties}')
#             candidates = [s for s in candidates if s not in self.empty_slots]

#         # Test 2+: avoid known wrong slots
#         if self.test_step >= 2:
#             wrongs = [s for s in candidates if s in self.checked_wrong]
#             if wrongs:
#                 self.get_logger().info(f'Ignoring known wrong slots: {wrongs}')
#             candidates = [s for s in candidates if s not in self.checked_wrong]

#         # Handle no candidates
#         if not candidates:
#             if self.test_step >= 5:
#                 req = String()
#                 req.data = f'Restock needed for "{self.current_target}"'
#                 self.restock_pub.publish(req)
#                 self.get_logger().warn('No slots left -> requesting restock')
#             else:
#                 self.get_logger().warn('No remaining candidates; halting search')
#             return

#         # Pick first candidate
#         self.publish_slot(candidates[0])

#     def publish_slot(self, slot: int):
#         """
#         Publish next slot to check, and pseudo-move UR3 there.
#         """
#         coord = self.slot_coords.get(slot, (0,0,0))
#         self.get_logger().info(f'-> Next slot to check: {slot} (coord={coord})')
#         msg = Int32(data=slot)
#         self.next_slot_pub.publish(msg)
#         self.move_to(coord)

# def main(args=None):
#     rclpy.init(args=args)
#     node = MemoryDecisionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



##################################################################################################################################

#########fundamental logic works
# class MemoryDecisionNode(Node):
#     def __init__(self):
#         super().__init__('drink_memory')
#         # Declare ROS parameters
#         # Default to a 3x3 grid (9 slots) and starting at test_step 1
#         self.declare_parameter('total_slots', 9)
#         self.declare_parameter('test_step', 1)  # 1..5
#         self.total_slots = self.get_parameter('total_slots').value
#         self.test_step = self.get_parameter('test_step').value

#         # Internal memory structures
#         self.empty_slots = set()      # slots known empty or retrieved
#         self.checked_wrong = set()    # slots checked but contained wrong drink
#         self.drink_locations = {}     # mapping drink_name -> slot number
#         self.current_target = None    # drink currently being searched
#         self.checked_this_request = set()  # slots checked for the current request

#         # publishers 
#         self.next_slot_pub = self.create_publisher(Int32, 'next_slot', 10)
#         self.restock_pub   = self.create_publisher(String, 'restock_request', 10)

#         # Subscribers  
#         self.create_subscription(String, 'requested_drink', self.request_cb, 10)
#         self.create_subscription(String, 'scanned_drink', self.scan_cb, 10)
#         self.create_subscription(Int32,  'scanned_slot',  self.slot_cb, 10)

#         # self.get_logger().info(f'MemoryDecision ready (test_step={self.test_step})')
#         # self.get_logger().info('MemoryDecisionNode started')
#         # self.get_logger().info(f'MemoryDecisionNode started (step={self.test_step})')
#         self.get_logger().info(f'MemoryDecisionNode started (step={self.test_step}, slots={self.total_slots})')

#     def request_cb(self, msg: String):
#         """
#         Callback for a new drink request.
#         Resets per-request memory and initiates slot decision.
#         """
#         # self.current_target = msg.data.strip()
#         self.current_target = msg.data.strip().lower()
#         self.checked_this_request.clear()
#         self.get_logger().info(f'Received request for "{self.current_target}"')
#         self.decide_next_slot()

#     def slot_cb(self, msg: Int32):
#         """
#         Callback to record the slot number being scanned.
#         Sets context for the next scan result.
#         """
#         self._last_scanned_slot = msg.data

#     def scan_cb(self, msg: String):
#         """
#         Callback for scan result of the previously published slot.
#         Updates internal memory based on scan and continues search.
#         """
#         if not hasattr(self, '_last_scanned_slot'):
#             # self.get_logger().warn('Got scan result but no slot info yet.')
#             self.get_logger().warn('Scan result received without slot context')
#             return

#         slot = self._last_scanned_slot
#         result = msg.data.strip().lower()

#         # Test 1: mark empty
#         # if result in ('', 'unknown'):
#         if result == 'unknown':
#             # mark slot as empty/unavailable
#             self.empty_slots.add(slot)
#             # self.get_logger().info(f'Slot {slot} marked empty')
#             self.get_logger().info(f'Slot {slot} marked empty')
#         # Test 2: wrong drink?
#         # elif result != self.current_target:
#         # elif result != self.current_target.lower():
#         #     # Slot had a different drink
#         #     self.checked_wrong.add(slot)
#         #     self.checked_this_request.add(slot)
#         #     self.get_logger().info(f'Slot {slot} has "{result}", not "{self.current_target}"')
#         elif result != self.current_target:
#             # Record wrong drink location
#             self.checked_wrong.add(slot)
#             self.checked_this_request.add(slot)
#             self.drink_locations[result] = slot
#             self.get_logger().info(f'Slot {slot} contains "{result}", not "{self.current_target}"')
#         # Found it!
#         else:
#             # self.drink_locations[self.current_target] = slot
#             # self.empty_slots.add(slot) # now empty after retrieval
#             # self.get_logger().info(f'Found "{self.current_target}" in slot {slot}; marking empty after retrieval')
#             # # self.get_logger().info(f'Found "{self.current_target}" in slot {slot}')
#             # Correct drink found -> deliver and mark empty
#             self.drink_locations[self.current_target] = slot
#             self.empty_slots.add(slot)
#             self.get_logger().info(f'Found "{self.current_target}" in slot {slot}; removing it')

#         # Decide next slot based on updated memory
#         self.decide_next_slot()

#     def decide_next_slot(self):
#         """
#         Core logic to choose the next slot according to test_step.
#         """
#         # Test 3+: prioritise known location of this drink
#         # if we already know location and test>=3, prioritize it
#         # if self.test_step >= 3 and self.current_target in self.drink_locations:
#         #     best = self.drink_locations[self.current_target]
#         #     self.get_logger().info(f'[*] Prioritising known slot {best}')
#         #     self.publish_slot(best)
#         #     return
#         # if self.test_step >= 3 and self.current_target in self.drink_locations:
#         #     known_slot = self.drink_locations[self.current_target]
#         #     self.get_logger().info(f'[*] Prioritising known slot {known_slot} for "{self.current_target}"')
#         #     self.publish_slot(known_slot)
#         #     return
#         if self.test_step >= 3 and self.current_target in self.drink_locations:
#             known_slot = self.drink_locations[self.current_target]
#             if known_slot not in self.empty_slots:
#                 self.get_logger().info(f'[*] Prioritising known slot {known_slot} for "{self.current_target}"')
#                 self.publish_slot(known_slot)
#                 return
#             else:
#                 # Remove mapping if slot is now empty
#                 del self.drink_locations[self.current_target]

#         # build candidate list
#         # Build list of all slot numbers
#         candidates = list(range(1, self.total_slots+1))

#         # Test 1+: avoid empty
#         if self.test_step >= 1:
#             # candidates = [s for s in candidates if s not in self.empty_slots]
#             # avoided = [s for s in candidates if s in self.empty_slots]
#             # avoided_empty = [s for s in candidates if s in self.empty_slots]
#             # if avoided_empty:
#             #     self.get_logger().info(f'Ignoring emptied slots: {avoided_empty}')
#             # candidates = [s for s in candidates if s not in self.empty_slots]
#             empties = [s for s in candidates if s in self.empty_slots]
#             if empties:
#                 self.get_logger().info(f'Ignoring emptied slots: {empties}')
#             candidates = [s for s in candidates if s not in self.empty_slots]
#             # if avoided:
#             #     self.get_logger().info(f'Ignoring emptied slots: {avoided}')
#             # candidates = [s for s in candidates if s not in self.empty_slots]

#         # Test 2: avoid previously wrong for this target
#         # if self.test_step >= 2:
#         #     # candidates = [s for s in candidates if s not in self.checked_wrong]
#         #     avoided = [s for s in candidates if s in self.checked_wrong]
#         #     if avoided:
#         #         self.get_logger().info(f'Ignoring known wrong slots: {avoided}')
#         #     candidates = [s for s in candidates if s not in self.checked_wrong]
#         # Test 2+: avoid slots known wrong for any previous drink
#         # if self.test_step >= 2:
#         #     avoided_wrong = [s for s in candidates if s in self.checked_wrong]
#         #     if avoided_wrong:
#         #         self.get_logger().info(f'Ignoring known wrong slots: {avoided_wrong}')
#         #     candidates = [s for s in candidates if s not in self.checked_wrong]
#         if self.test_step >= 2:
#             wrongs = [s for s in candidates if s in self.checked_wrong]
#             if wrongs:
#                 self.get_logger().info(f'Ignoring known wrong slots: {wrongs}')
#             candidates = [s for s in candidates if s not in self.checked_wrong]

#         # If no candidates remain, handle accordingly
#         # if not candidates:
#         #     # Test 5: empty esky -> restock
#         #     if self.test_step >= 5:#5->9
#         #         self.get_logger().warn('No slots left to check -> requesting restock')
#         #         req = String()
#         #         req.data = f'Restock needed for "{self.current_target}"'
#         #         self.restock_pub.publish(req)
#         #         # return
#         #     else:
#         #         # self.get_logger().warn('No candidates but test_step<5; halting.')
#         #         self.get_logger().warn('No remaining candidates; halting search')
#         #     return
#         if not candidates:
#             if self.test_step >= 5:
#                 self.get_logger().warn('No slots left -> requesting restock')
#                 req = String()
#                 req.data = f'Restock needed for "{self.current_target}"'
#                 self.restock_pub.publish(req)
#             else:
#                 self.get_logger().warn('No remaining candidates; halting search')
#             return

        
#         # Default: pick the first candidate
#         # choose first candidate (could be randomized or optimised)
#         next_slot = candidates[0]
#         self.publish_slot(next_slot)

#     def publish_slot(self, slot: int):
#         """
#         Publish the next slot to check and simulate UR3 movement.
#         """
#         self.get_logger().info(f'-> Next slot to check: {slot}')
#         # msg = Int32()
#         msg = Int32(data=slot)
#         self.next_slot_pub.publish(msg)
#         self.get_logger().info(f'[UR3] Pseudo-moving to slot {slot}...')
#         # msg.data = slot
#         # self.next_slot_pub.publish(msg)
#         # # pseudo-move:
#         # self.get_logger().info(f'[UR3] Pseudo-moving to slot {slot}...')

# def main(args=None):
#     rclpy.init(args=args)
#     node = MemoryDecisionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# UR3 implement attempt
# #!/usr/bin/env python3
# """
# Decoupled ROS 2 nodes for drink-memory and UR3 motion control.

# Design rationale:
# 1. **Separation of concerns**: MemoryDecisionNode focuses on search logic and goal publication, while UR3TrajectoryPublisher handles IK and trajectory execution.
# 2. **Loose coupling**: MemoryDecisionNode publishes simple Cartesian goals (`geometry_msgs/Point`) to `/ur3_goal`, without embedding IK or trajectory details.
# 3. **Modularity**: We now have two clear entry points (`main_memory` and `main_ur3`) to launch each node independently via console scripts.
# 4. **Testability & Scalability**: Each node can be tested or replaced separately, and new motion nodes can subscribe to `/ur3_goal` without changing search logic.

# Entry points (configured in `setup.py`):
# - `memory_decision = drink_memory_node:main_memory`
# - `ur3_trajectory  = drink_memory_node:main_ur3`
# """
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Int32
# from geometry_msgs.msg import Point
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from roboticstoolbox import models, jtraj
# from spatialmath import SE3

# # Node 1: Decision logic, publishes Cartesian goals only
# class MemoryDecisionNode(Node):
#     def __init__(self):
#         super().__init__('memory_decision')
#         # Configuration parameters
#         self.declare_parameter('total_slots', 9)
#         self.declare_parameter('test_step', 1)
#         self.total_slots = self.get_parameter('total_slots').value
#         self.test_step   = self.get_parameter('test_step').value

#         # Coordinates for slots (3×3 grid) and drop-off
#         self.slot_coords = {
#             1:(0.1,0.1,0.0), 2:(0.2,0.1,0.0), 3:(0.3,0.1,0.0),
#             4:(0.1,0.2,0.0), 5:(0.2,0.2,0.0), 6:(0.3,0.2,0.0),
#             7:(0.1,0.3,0.0), 8:(0.2,0.3,0.0), 9:(0.3,0.3,0.0),
#         }
#         self.dropoff_coord = (0.5,0.0,0.2)

#         # Internal search state
#         self.empty_slots     = set()
#         self.checked_wrong   = set()
#         self.drink_locations = {}
#         self.current_target  = None

#         # Publishers for decoupled communication
#         self.coord_pub    = self.create_publisher(Point,   '/ur3_goal', 10)
#         self.decision_pub = self.create_publisher(Int32,   'next_slot', 10)
#         self.restock_pub  = self.create_publisher(String,  'restock_request', 10)

#         # Subscribers for requests and perception
#         self.create_subscription(String, 'requested_drink', self.request_cb, 10)
#         self.create_subscription(Int32,  'scanned_slot',     self.slot_cb,   10)
#         self.create_subscription(String, 'scanned_drink',    self.scan_cb,   10)

#         self.get_logger().info(f'[memory_decision] Started (test_step={self.test_step})')

#     def request_cb(self, msg: String):
#         """
#         New drink requested: start search.
#         """
#         self.current_target = msg.data.strip().lower()
#         self.get_logger().info(f'Requested: "{self.current_target}"')
#         self.decide_next_slot()

#     def slot_cb(self, msg: Int32):
#         """
#         Record last scanned slot index.
#         """
#         self._last_slot = msg.data

#     def scan_cb(self, msg: String):
#         """
#         Process scan result: update memory and publish goals.
#         """
#         slot = getattr(self, '_last_slot', None)
#         if slot is None:
#             self.get_logger().warn('Scan result without slot index')
#             return
#         result = msg.data.strip().lower()

#         if result == 'unknown':  # Test 1
#             self.empty_slots.add(slot)
#             self.get_logger().info(f'Slot {slot} → empty')
#         elif result != self.current_target:  # Test 2
#             self.checked_wrong.add(slot)
#             self.drink_locations[result] = slot
#             self.get_logger().info(f'Slot {slot} → "{result}"')
#         else:
#             # Found correct drink: publish pickup & dropoff
#             self.drink_locations[self.current_target] = slot
#             self.empty_slots.add(slot)
#             self._publish_goal(slot)
#             self._publish_dropoff()
#             self.get_logger().info(f'Delivered "{self.current_target}" from slot {slot}')
#             return

#         # Continue search
#         self.decide_next_slot()

#     def decide_next_slot(self):
#         """
#         Apply Test 1–3 filters and publish next goal.
#         """
#         # Test 3: prioritise known location if still valid
#         if self.test_step >= 3 and self.current_target in self.drink_locations:
#             loc = self.drink_locations[self.current_target]
#             if loc not in self.empty_slots:
#                 self._publish_goal(loc)
#                 return
#             else:
#                 del self.drink_locations[self.current_target]

#         # Filter slots
#         cands = list(range(1, self.total_slots + 1))
#         if self.test_step >= 1:
#             cands = [s for s in cands if s not in self.empty_slots]
#         if self.test_step >= 2:
#             cands = [s for s in cands if s not in self.checked_wrong]

#         # No candidates → restock
#         if not cands:
#             self.restock_pub.publish(String(data=f'Restock "{self.current_target}"'))
#             self.get_logger().warn('Esky empty; requesting restock')
#             return

#         # Publish first candidate
#         self._publish_goal(cands[0])

#     def _publish_goal(self, slot: int):
#         """
#         Publish slot index and Cartesian goal for UR3 to pick up.
#         """
#         coord = self.slot_coords.get(slot, self.dropoff_coord)
#         self.decision_pub.publish(Int32(data=slot))
#         pt = Point(x=coord[0], y=coord[1], z=coord[2])
#         self.coord_pub.publish(pt)
#         self.get_logger().info(f'Goal: slot={slot}, coord={coord}')

#     def _publish_dropoff(self):
#         """
#         After pickup, publish drop-off coordinate.
#         """
#         x,y,z = self.dropoff_coord
#         pt = Point(x=x, y=y, z=z)
#         self.coord_pub.publish(pt)
#         self.get_logger().info(f'Drop-off coord: {self.dropoff_coord}')

# # Node 2: Motion control, subscribes to `/ur3_goal`
# class UR3TrajectoryPublisher(Node):
#     def __init__(self):
#         super().__init__('ur3_trajectory')
#         self.traj_pub = self.create_publisher(JointTrajectory,
#             '/scaled_joint_trajectory_controller/joint_trajectory', 10)
#         self.create_subscription(Point, '/ur3_goal', self.goal_cb, 10)
#         # UR3 model for IK
#         self.robot = models.UR3()
#         self.robot.q = self.robot.qz
#         self.get_logger().info('[ur3_trajectory] Started')

#     def goal_cb(self, msg: Point):
#         """
#         Plan and send trajectory to reach received Cartesian goal.
#         """
#         coord = (msg.x, msg.y, msg.z)
#         T = SE3(*coord)
#         sol = self.robot.ikine_LM(T)
#         if not sol.success:
#             self.get_logger().error(f'IK failed for {coord}')
#             return
#         q_start, q_goal = self.robot.q, sol.q
#         traj = jtraj(q_start, q_goal, 50)

#         jtraj_msg = JointTrajectory()
#         jtraj_msg.joint_names = [
#             'shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
#             'wrist_1_joint','wrist_2_joint','wrist_3_joint'
#         ]
#         for i, q in enumerate(traj.q):
#             pt = JointTrajectoryPoint()
#             pt.positions = q.tolist()
#             pt.time_from_start = Duration(sec=0, nanosec=int(i*2e7))
#             jtraj_msg.points.append(pt)

#         self.traj_pub.publish(jtraj_msg)
#         self.robot.q = q_goal
#         self.get_logger().info(f'Trajectory published to {coord}')

# # Entry points called by console_scripts in setup.py

# def main_memory(args=None):
#     rclpy.init(args=args)
#     node = MemoryDecisionNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# def main_ur3(args=None):
#     rclpy.init(args=args)
#     node = UR3TrajectoryPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

