# This should hopefully be able to be integrated with Rhys's Pieces
# !/usr/bin/env python3
"""
ROS2 node for drink memory and search strategy.
Supports test steps 1-5:
 1: Avoid empty slots.
 2: Avoid wrong slots.
 3: Seek previously found locations (also includes 1 & 2).
 4: Efficiently update moved drinks (not implemented here yet).
 5: Detect empty esky and request restock.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
 

class MemoryDecisionNode(Node):
    def __init__(self):
        super().__init__('drink_memory')
        # Decision logic parameters
        self.declare_parameter('test_step', 1)
        self.test_step = self.get_parameter('test_step').value

        # Dynamic visual data (initially None or empty)
        self.total_slots = None            # will be set by esky_slot_count subscriber
        self.slot_coords = {}              # slot_id -> (x,y,z)
        self.dropoff_coord = None          # (x,y,z)

        # Internal memory structures
        self.empty_slots = set()
        self.checked_wrong = set()
        self.drink_locations = {}
        self.current_target = None
        self.checked_this_request = set()

        # Publishers
        self.next_slot_pub = self.create_publisher(Int32, 'next_slot', 10)
        self.restock_pub   = self.create_publisher(String, 'restock_request', 10)
        self.cart_goal_pub = self.create_publisher(PoseStamped, 'cartesian_goal', 10)

        # Subscribers: decision inputs
        self.create_subscription(String, 'requested_drink', self.request_cb, 10)
        self.create_subscription(Int32,  'scanned_slot',     self.slot_cb,   10)
        self.create_subscription(String, 'scanned_drink',    self.scan_cb,   10)

        # Subscribers: visual data
        self.create_subscription(Int32,     'esky_slot_count', self.slot_count_cb, 10)
        self.create_subscription(PoseStamped, 'esky_slot_pose',  self.slot_pose_cb,  10)
        self.create_subscription(PoseStamped, 'dropoff_location', self.dropoff_cb,   10)

        self.get_logger().info(
            f'DrinkMemoryNode started (test_step={self.test_step}), waiting for visual data...'
        )

    def slot_count_cb(self, msg: Int32):
        self.total_slots = msg.data
        self.get_logger().info(f"Total slots updated: {self.total_slots}")

    def dropoff_cb(self, msg: PoseStamped):
        self.dropoff_coord = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        self.get_logger().info(f"Drop-off coord set to {self.dropoff_coord}")

    def slot_pose_cb(self, msg: PoseStamped):
        # if the header or a field encodes which slot:
        slot_id = int(msg.header.frame_id)  # or use msg.slot_index if custom
        self.slot_coords[slot_id] = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        self.get_logger().info(f"Slot {slot_id} pose updated: {self.slot_coords[slot_id]}")


    def move_to(self, coord):########
        """
        Pseudo-interface for UR3 movement to a Cartesian goal.
        In real integration, replace this with action client calls.
        """
        # self.get_logger().info(f'[UR3] Pseudo-moving to {coord}')
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'    # match your MoveIt frame
        msg.pose.position.x = coord[0]
        msg.pose.position.y = coord[1]
        msg.pose.position.z = coord[2]
        # keep orientation fixed (you could parameterise this)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.cart_goal_pub.publish(msg)
        self.get_logger().info(f'Published cartesian_goal: {coord}')

    def request_cb(self, msg: String):
        """
        Handle new drink request: reset per-request memory and start search.
        """
        self.current_target = msg.data.strip().lower()
        self.checked_this_request.clear()
        self.get_logger().info(f'Received request for "{self.current_target}"')
        self.decide_next_slot()

    def slot_cb(self, msg: Int32):
        """
        Record the slot number being scanned for context.
        """
        self._last_scanned_slot = msg.data

    def scan_cb(self, msg: String):
        """
        Process scan result: update memory, move UR3, and conclude.
        """
        if not hasattr(self, '_last_scanned_slot'):
            self.get_logger().warn('Scan result without slot context')
            return

        slot = self._last_scanned_slot
        result = msg.data.strip().lower()

        if result == 'unknown':
            # Mark slot empty and clear any stale mapping
            self.empty_slots.add(slot)
            for drink, loc in list(self.drink_locations.items()):
                if loc == slot:
                    del self.drink_locations[drink]
            self.get_logger().info(f'Slot {slot} marked empty')
            self.decide_next_slot()

        elif result != self.current_target:
            # Wrong drink: record and store its location
            self.checked_wrong.add(slot)
            self.checked_this_request.add(slot)
            self.drink_locations[result] = slot
            self.get_logger().info(f'Slot {slot} contains "{result}", not "{self.current_target}"')
            self.decide_next_slot()

        else:
            # Correct drink found: move to slot and drop-off
            coord = self.slot_coords.get(slot, (0,0,0))
            self.move_to(coord)              # move to pickup
            self.move_to(self.dropoff_coord) # move to drop-off
            self.empty_slots.add(slot)       # now empty
            self.get_logger().info(f'Found and removed "{self.current_target}" from slot {slot}')
            # Wait for next request (no further search)

    def decide_next_slot(self):
        """
        Determine next slot to check based on test_step logic. Publish and move.
        """
        # Test 3+: prioritise known location if still valid
        if self.test_step >= 3 and self.current_target in self.drink_locations:
            known = self.drink_locations[self.current_target]
            if known not in self.empty_slots:
                self.get_logger().info(f'[*] Prioritising known slot {known} for "{self.current_target}"')
                self.publish_slot(known)
                return
            else:
                del self.drink_locations[self.current_target]

        # Build candidate list
        candidates = list(range(1, self.total_slots + 1))

        # Test 1+: avoid emptied slots
        if self.test_step >= 1:
            empties = [s for s in candidates if s in self.empty_slots]
            if empties:
                self.get_logger().info(f'Ignoring emptied slots: {empties}')
            candidates = [s for s in candidates if s not in self.empty_slots]

        # Test 2+: avoid known wrong slots
        if self.test_step >= 2:
            wrongs = [s for s in candidates if s in self.checked_wrong]
            if wrongs:
                self.get_logger().info(f'Ignoring known wrong slots: {wrongs}')
            candidates = [s for s in candidates if s not in self.checked_wrong]

        # Handle no candidates
        if not candidates:
            if self.test_step >= 5:
                req = String()
                req.data = f'Restock needed for "{self.current_target}"'
                self.restock_pub.publish(req)
                self.get_logger().warn('No slots left -> requesting restock')
            else:
                self.get_logger().warn('No remaining candidates; halting search')
            return

        # Pick first candidate
        self.publish_slot(candidates[0])

    def publish_slot(self, slot: int):
        """
        Publish next slot to check, and pseudo-move UR3 there.
        """
        coord = self.slot_coords.get(slot, (0,0,0))
        self.get_logger().info(f'-> Next slot to check: {slot} (coord={coord})')
        msg = Int32(data=slot)
        self.next_slot_pub.publish(msg)
        self.move_to(coord)

def main(args=None):
    rclpy.init(args=args)
    node = MemoryDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

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

