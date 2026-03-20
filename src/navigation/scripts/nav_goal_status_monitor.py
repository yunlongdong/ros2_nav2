#!/usr/bin/env python3

import argparse
import math
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw: float):
    """Convert yaw angle (rad) to quaternion (x, y, z, w)."""
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class NavGoalStatusMonitor(Node):
    STATUS_TEXT = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }

    def __init__(self, x: float, y: float, yaw: float, frame_id: str, wait_server_sec: float):
        super().__init__("nav_goal_status_monitor")

        self.target_x = x
        self.target_y = y
        self.target_yaw = yaw
        self.frame_id = frame_id
        self.wait_server_sec = wait_server_sec

        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._status_sub = self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._status_callback,
            10,
        )

        self._goal_handle = None
        self._goal_uuid_str: Optional[str] = None
        self._last_status_code: Optional[int] = None
        self.done = False

    @staticmethod
    def _uuid_to_str(uuid_msg) -> str:
        return "".join(f"{b:02x}" for b in uuid_msg.uuid)

    @classmethod
    def _status_to_text(cls, code: int) -> str:
        return cls.STATUS_TEXT.get(code, f"UNKNOWN_CODE_{code}")

    def send_goal(self):
        self.get_logger().info("Waiting for /navigate_to_pose action server...")
        if not self._action_client.wait_for_server(timeout_sec=self.wait_server_sec):
            self.get_logger().error(
                f"Action server /navigate_to_pose not available after {self.wait_server_sec:.1f}s"
            )
            self.done = True
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.target_x
        goal.pose.pose.position.y = self.target_y
        goal.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self.target_yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Sending goal_pose: x={self.target_x:.3f}, y={self.target_y:.3f}, "
            f"yaw={self.target_yaw:.3f} rad, frame_id={self.frame_id}"
        )

        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2")
            self.done = True
            return

        self._goal_handle = goal_handle
        self._goal_uuid_str = self._uuid_to_str(goal_handle.goal_id)
        self.get_logger().info(f"Goal accepted, goal_id={self._goal_uuid_str}")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        # Different Nav2 versions may expose slightly different feedback fields.
        if hasattr(fb, "distance_remaining"):
            self.get_logger().info(
                f"Feedback: distance_remaining={fb.distance_remaining:.3f} m, "
                f"recoveries={getattr(fb, 'number_of_recoveries', 0)}"
            )

    def _status_callback(self, msg: GoalStatusArray):
        if not self._goal_uuid_str:
            return

        matched_status = None
        for status in msg.status_list:
            if self._uuid_to_str(status.goal_info.goal_id) == self._goal_uuid_str:
                matched_status = status.status
                break

        if matched_status is None:
            return

        if matched_status != self._last_status_code:
            self._last_status_code = matched_status
            self.get_logger().info(
                f"Status update: {self._status_to_text(matched_status)} ({matched_status})"
            )

    def _result_callback(self, future):
        result = future.result()
        final_status = result.status
        self.get_logger().info(
            f"Navigation finished with status: "
            f"{self._status_to_text(final_status)} ({final_status})"
        )
        self.done = True

    def cancel_goal(self):
        if self._goal_handle is None:
            return
        self.get_logger().info("Canceling goal...")
        self._goal_handle.cancel_goal_async()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send a NavigateToPose goal and loop-monitor /navigate_to_pose/_action/status"
    )
    parser.add_argument("--x", type=float, required=True, help="Target x in map frame")
    parser.add_argument("--y", type=float, required=True, help="Target y in map frame")
    parser.add_argument("--yaw", type=float, default=0.0, help="Target yaw (rad)")
    parser.add_argument("--frame-id", type=str, default="map", help="Target frame id")
    parser.add_argument(
        "--wait-server-sec",
        type=float,
        default=10.0,
        help="How long to wait for action server",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()

    node = NavGoalStatusMonitor(
        x=args.x,
        y=args.y,
        yaw=args.yaw,
        frame_id=args.frame_id,
        wait_server_sec=args.wait_server_sec,
    )

    node.send_goal()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received")
        node.cancel_goal()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
