"""Offline test stub: accepts the real node's parameters and does nothing (kinsim_ros.py is the interface)."""
import rclpy
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node('flight_interface', allow_undeclared_parameters=True,
                automatically_declare_parameters_from_overrides=True)
    node.get_logger().info('STUB flight interface (offline test only): params ' +
                           ', '.join(f'{k}={p.value}' for k, p in sorted(node._parameters.items())))
    try:
        rclpy.spin(node)
    except Exception:
        pass
