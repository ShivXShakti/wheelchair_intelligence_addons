#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import json
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose


class Nav2GoalFromTopic(Node):

    def __init__(self):
        super().__init__('nav2_goal_from_topic')

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.create_subscription(String, '/llm_command', self.llama_cb, 10)

        self.current_goal_handle = None
        self.is_navigating = False
        self.destinations = self.load_locations()

        self.active_goal_pose = None
        self.paused_goal_pose = None

        self.get_logger().info("Waiting for Nav2 action server...")
        self.action_client.wait_for_server()
        self.get_logger().info("Nav2 server ready!")

    

    def load_locations(self):
        pkg_path = get_package_share_directory("wheelchair_intelligence_addons")
        file_path = os.path.join(pkg_path, "config", "rrc_map_semantics.yaml")

        try:
            with open(file_path, 'r') as file:
                data = yaml.safe_load(file) or {}

            self.get_logger().info("Locations loaded successfully")
            return data

        except Exception as e:
            self.get_logger().error(f"Failed to load locations: {e}")
            return {}
    
    def llama_cb(self, msg):
        try:
            data = json.loads(msg.data)
            action = data.get("action", "").lower()
            destination = data.get("destination", "").lower()

            self.get_logger().info(f"Action: {action}, Destination: {destination}")

            # ==============================
            # STOP COMMAND
            # ==============================
            if action == "stop":
                self.stop_navigation()
                return

            # ==============================
            # PAUSE COMMAND
            # ==============================
            if action == "wait":
                self.pause_navigation()
                return

            # ==============================
            # RESUME COMMAND
            # ==============================
            if action == "resume":
                self.resume_navigation()
                return

            # ==============================
            # NAVIGATION COMMAND
            # ==============================
            if action != "navigate":
                return

            if destination not in self.destinations:
                self.get_logger().warn("Unknown destination!")
                return

            goal_data = self.destinations[destination]

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = goal_data["x"]
            pose.pose.position.y = goal_data["y"]
            pose.pose.position.z = 0.0

            yaw = goal_data["yaw"]
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            self.active_goal_pose = pose
            self.paused_goal_pose = None
            self.send_nav2_goal(pose)

        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON: {msg.data}")

    def stop_navigation(self):

        if self.current_goal_handle is None:
            self.get_logger().info("No active goal to stop.")
            return

        self.get_logger().info("Stopping navigation completely...")

        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._stop_done)

        # Clear everything
        self.active_goal_pose = None
        self.paused_goal_pose = None


    def _stop_done(self, future):
        self.get_logger().info("Navigation stopped.")
        self.current_goal_handle = None
        self.is_navigating = False
        
    def send_nav2_goal(self, pose):

        # If already navigating → cancel old goal first
        if self.is_navigating and self.current_goal_handle is not None:
            self.get_logger().info("Cancelling previous goal...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda future: self._send_goal_after_cancel(pose)
            )
            return

        self._send_goal_after_cancel(pose)

    def pause_navigation(self):

        if self.current_goal_handle is None:
            self.get_logger().info("Nothing to pause.")
            return

        self.get_logger().info("Pausing navigation...")

        # Save current active goal for resume
        self.paused_goal_pose = self.active_goal_pose

        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._pause_done)


    def _pause_done(self, future):
        self.get_logger().info("Navigation paused.")
        self.current_goal_handle = None
        self.is_navigating = False

    def resume_navigation(self):

        if self.paused_goal_pose is not None:
            self.get_logger().info("Resuming navigation...")
            self.send_nav2_goal(self.paused_goal_pose)
            self.paused_goal_pose = None
        else:
            self.get_logger().info("No paused goal to resume.")

    def _send_goal_after_cancel(self, pose):

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.is_navigating = True

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)
    

    # ================================
    # Goal response
    # ================================
    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted!')
        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    # ================================
    # Result callback
    # ================================
    def get_result_callback(self, future):

        status = future.result().status
        self.get_logger().info(f'Navigation finished with status: {status}')

        self.is_navigating = False
        self.current_goal_handle = None

    # ================================
    # Feedback
    # ================================
    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Distance remaining: {feedback.distance_remaining:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalFromTopic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
