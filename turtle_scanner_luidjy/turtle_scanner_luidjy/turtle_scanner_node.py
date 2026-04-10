#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Message Pose de turtlesim
from turtlesim.msg import Pose


class TurtleScannerNode(Node):
    def __init__(self):
        # Nom du noeud
        super().__init__("turtle_scanner_node")

        # Variable qui va stocker la position de turtle1
        self.pose_scanner = None

        # Variable qui va stocker la position de turtle_target
        self.pose_target = None

        # Subscriber sur /turtle1/pose
        self.subscriber_scanner = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.scanner_pose_callback,
            10
        )

        # Subscriber sur /turtle_target/pose
        self.subscriber_target = self.create_subscription(
            Pose,
            "/turtle_target/pose",
            self.target_pose_callback,
            10
        )

        self.get_logger().info("Turtle scanner node started")

    def scanner_pose_callback(self, msg):
        # A chaque message recu, on met a jour la pose du scanner
        self.pose_scanner = msg

        # Affichage simple pour verifier
        self.get_logger().info(
            f"Scanner pose -> x: {msg.x:.2f}, y: {msg.y:.2f}, theta: {msg.theta:.2f}"
        )

    def target_pose_callback(self, msg):
        # A chaque message recu, on met a jour la pose de la cible
        self.pose_target = msg

        # Affichage simple pour verifier
        self.get_logger().info(
            f"Target pose -> x: {msg.x:.2f}, y: {msg.y:.2f}, theta: {msg.theta:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = TurtleScannerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()