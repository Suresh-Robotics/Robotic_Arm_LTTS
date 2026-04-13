#!/usr/bin/env python3
"""
Pick and place node combining Cartesian and joint-space moves with smooth joint transitions.
Scans and stores all box positions before starting motion sequence.

ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B

"""

from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import ltts

import math
import time


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("target_color", "R")
        self.declare_parameter("scan_duration", 3.0)  # How long to scan for boxes
        self.target_color = self.get_parameter("target_color").value.upper()
        self.scan_duration = self.get_parameter("scan_duration").value

        # Flags and storage
        self.scanning_complete = False
        self.detected_boxes = {}  # Dictionary to store all detected boxes: {color: [x, y, z]}
        self.target_coords = None  # Final locked coordinates for target color

        self.callback_group = ReentrantCallbackGroup()

        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ltts.joint_names(),
            base_link_name=ltts.base_link_name(),
            end_effector_name=ltts.end_effector_name(),
            group_name=ltts.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Set lower velocity & acceleration for smoother motion
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1

        # Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=ltts.gripper_joint_names(),
            open_gripper_joint_positions=ltts.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=ltts.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=ltts.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Subscriber
        self.sub = self.create_subscription(
            String, "/color_coordinates", self.coords_callback, 10
        )

        # Predefined joint positions (in radians)
        self.start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-125.0)]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]
        # self.drop_joints  = [math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
        #                      math.radians(-124.0), math.radians(44.0), math.radians(163.0), math.radians(7.0)]
        self.red_drop_joints  = [math.radians(0.0), math.radians(-90.0), math.radians(-90.0),
                              math.radians(-4.0), math.radians(-131.0), math.radians(92.0), math.radians(56.0)]
        self.green_drop_joints  = [math.radians(0.0), math.radians(-90.0), math.radians(-90.0),
                              math.radians(-35.0), math.radians(-90.0), math.radians(84.0), math.radians(48.0)]
        self.blue_drop_joints  = [math.radians(-45.0), math.radians(-81.0), math.radians(-90.0),
                              math.radians(-35.0), math.radians(-90.0), math.radians(84.0), math.radians(0.0)]

        # Move to start joint configuration (scanning position)
        self.get_logger().info("Moving to scanning position...")
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()

        # Start scanning phase
        self.start_scanning()

    def start_scanning(self):
        """Scan for boxes for a set duration before starting motion."""
        self.get_logger().info(f"Scanning workspace for {self.scan_duration} seconds...")
        self.get_logger().info(f"Looking for target color: {self.target_color}")
        
        # Create a timer to end scanning after duration
        self.scan_timer = self.create_timer(self.scan_duration, self.finish_scanning)

    def finish_scanning(self):
        """End scanning phase and start pick-and-place if target found."""
        self.scan_timer.cancel()
        self.scanning_complete = True
        
        self.get_logger().info(f"Scanning complete. Detected boxes: {list(self.detected_boxes.keys())}")
        
        # Check if target color was found
        if self.target_color in self.detected_boxes:
            self.target_coords = self.detected_boxes[self.target_color]
            self.get_logger().info(
                f"Target {self.target_color} locked at: "
                f"[{self.target_coords[0]:.3f}, {self.target_coords[1]:.3f}, {self.target_coords[2]:.3f}]"
            )
            # Start pick-and-place sequence
            self.execute_pick_and_place()
        else:
            self.get_logger().warn(f"Target color {self.target_color} not found during scan!")
            self.get_logger().info("Returning to start position...")
            self.moveit2.move_to_configuration(self.start_joints)
            self.moveit2.wait_until_executed()
            rclpy.shutdown()

    def coords_callback(self, msg):
        # Store detected box coordinates from vision node
        if self.scanning_complete:
            return

        try:
            color_id, x, y, z = msg.data.split(",")
            color_id = color_id.strip().upper()
            coords = [float(x), float(y), float(z)]

            # Store or update the box position
            if color_id not in self.detected_boxes:
                self.get_logger().info(f"Detected {color_id} box at [{coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f}]")
            
            # Update detected box coordinates
            self.detected_boxes[color_id] = coords

        except Exception as e:
            self.get_logger().error(f"Error parsing /color_coordinates: {e}")

    def execute_pick_and_place(self):
        """Execute the pick-and-place sequence using stored coordinates."""
        pick_position = [self.target_coords[0], self.target_coords[1], self.target_coords[2] - 0.60]
        self.get_logger().info(f"Suresh Testing {pick_position}...")
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]

        # --- Pick-and-place sequence ---

        # 1. Move to home joint configuration
        self.get_logger().info("Moving to home position...")
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 2. Move above target (Cartesian)
        self.get_logger().info("Moving above target...")
        self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 3. Open gripper
        self.get_logger().info("Opening gripper...")
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 4. Move down to approach object
        self.get_logger().info("Approaching object...")
        approach_position = [pick_position[0], pick_position[1], pick_position[2] - 0.31]
        self.moveit2.move_to_pose(position=approach_position, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 5. Close gripper
        self.get_logger().info("Grasping object...")
        self.gripper.close()
        self.gripper.wait_until_executed()

        # 6. Move to home joint configuration
        self.get_logger().info("Lifting object...")
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        if self.target_color == "R":
            # 7. Move to drop joint configuration (intermediate waypoint)
            self.get_logger().info("Going to drop RED object")
            self.moveit2.move_to_configuration(self.red_drop_joints)
            self.moveit2.wait_until_executed()
        elif self.target_color == "G":
            # 7. Move to drop joint configuration (intermediate waypoint)
            self.get_logger().info("Going to drop GREEN object")
            self.moveit2.move_to_configuration(self.green_drop_joints)
            self.moveit2.wait_until_executed()
        else:
            # 7. Move to drop joint configuration (intermediate waypoint)
            self.get_logger().info("Going to drop BLUE object")
            self.moveit2.move_to_configuration(self.blue_drop_joints)
            self.moveit2.wait_until_executed()

        # 8. Open gripper to release on table
        self.get_logger().info("Releasing object onto CoffeeTable_5...")
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 9. Close gripper
        self.gripper.close()
        self.gripper.wait_until_executed()

        # 10. Return to start joint configuration
        self.get_logger().info("Returning to start position...")
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        self.get_logger().info("Pick-and-place sequence complete.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
