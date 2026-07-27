"""Shared support for one-shot ROS command publishers."""

from __future__ import annotations

import time

import rclpy


def publish_once(node, publisher, message, topic: str, timeout: float) -> bool:
    """Wait for the driver subscription, publish once, and flush discovery."""

    deadline = time.monotonic() + timeout
    while (
        rclpy.ok()
        and publisher.get_subscription_count() == 0
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.05)

    if publisher.get_subscription_count() == 0:
        node.get_logger().error(
            f"No subscriber found on {topic}; is the Mirobot driver running?"
        )
        return False

    publisher.publish(message)
    deadline = time.monotonic() + 0.25
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    return True
