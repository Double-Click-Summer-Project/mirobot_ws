"""Publish one validated six-joint target to the Mirobot driver."""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

from .cli_common import publish_once


JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send one six-axis Mirobot joint target.",
    )
    parser.add_argument(
        "positions",
        type=float,
        nargs=6,
        metavar="JOINT",
        help="six joint values in J1 through J6 order",
    )
    parser.add_argument(
        "--degrees",
        action="store_true",
        help="interpret the six values as degrees (default: radians)",
    )
    parser.add_argument(
        "--topic",
        default="/mirobot/joint_command",
        help="target JointState topic",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="seconds to wait for the driver subscriber",
    )
    return parser


def main(args=None) -> int:
    full_args = sys.argv if args is None else ["joint_command", *args]
    parsed = _parser().parse_args(remove_ros_args(args=full_args)[1:])
    positions = list(parsed.positions)
    if parsed.degrees:
        positions = [math.radians(value) for value in positions]

    rclpy.init(args=full_args)
    node = Node("mirobot_joint_command")
    try:
        publisher = node.create_publisher(
            JointState,
            parsed.topic,
            10,
        )
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = positions
        if not publish_once(
            node,
            publisher,
            message,
            parsed.topic,
            parsed.timeout,
        ):
            return 1
        node.get_logger().info(
            f"Published joint target in rad: {positions}"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
