"""Publish one validated Cartesian XYZ target to the Mirobot driver."""

from __future__ import annotations

import argparse
import sys

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from .cli_common import publish_once


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send one Mirobot Cartesian XYZ target.",
    )
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument(
        "--millimeters",
        action="store_true",
        help="interpret XYZ as millimeters (default: meters)",
    )
    parser.add_argument(
        "--topic",
        default="/mirobot/xyz_command",
        help="target Point topic",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="seconds to wait for the driver subscriber",
    )
    return parser


def main(args=None) -> int:
    full_args = sys.argv if args is None else ["xyz_command", *args]
    parsed = _parser().parse_args(remove_ros_args(args=full_args)[1:])
    scale = 0.001 if parsed.millimeters else 1.0

    rclpy.init(args=full_args)
    node = Node("mirobot_xyz_command")
    try:
        publisher = node.create_publisher(Point, parsed.topic, 10)
        message = Point()
        message.x = parsed.x * scale
        message.y = parsed.y * scale
        message.z = parsed.z * scale
        if not publish_once(
            node,
            publisher,
            message,
            parsed.topic,
            parsed.timeout,
        ):
            return 1
        node.get_logger().info(
            "Published XYZ target in meters: "
            f"({message.x}, {message.y}, {message.z})"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

