#!/usr/bin/env python3

import random

# Librairies ROS2
import rclpy
from rclpy.node import Node

# Service /spawn de turtlesim
from turtlesim.srv import Spawn


class SpawnTargetNode(Node):
    def __init__(self):
        # Nom du noeud
        super().__init__("spawn_target_node")

        # Client du service /spawn
        self.client = self.create_client(Spawn, "/spawn")

        # On attend que le service soit disponible
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn service...")

        # Creation de la requete
        self.request = Spawn.Request()

        # Coordonnees aleatoires entre 1 et 10
        self.target_x = random.uniform(1.0, 10.0)
        self.target_y = random.uniform(1.0, 10.0)

        # Remplissage de la requete
        self.request.x = self.target_x
        self.request.y = self.target_y
        self.request.theta = 0.0
        self.request.name = "turtle_target"

        # Envoi de la requete en asynchrone
        self.future = self.client.call_async(self.request)

        # Petit message de confirmation
        self.get_logger().info("Spawn request sent...")


def main(args=None):
    # Initialisation ROS2
    rclpy.init(args=args)

    # Creation du noeud
    node = SpawnTargetNode()

    # On attend la reponse du service
    while rclpy.ok():
        rclpy.spin_once(node)

        if node.future.done():
            try:
                response = node.future.result()

                # Affichage demande dans la question
                node.get_logger().info(
                    f"Target spawned: {response.name} at ({node.target_x:.2f}, {node.target_y:.2f})"
                )
            except Exception as e:
                node.get_logger().error(f"Service call failed: {e}")
            break

    # Fermeture propre
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()