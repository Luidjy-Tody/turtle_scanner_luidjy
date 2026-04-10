#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

# Service personnalise
from turtle_interfaces.srv import ResetMission


class MissionClient(Node):
    def __init__(self):
        # Nom du noeud
        super().__init__("mission_client_node")

        # Client du service /reset_mission
        self.client = self.create_client(ResetMission, "/reset_mission")

        # Subscriber sur /target_detected
        self.subscriber = self.create_subscription(
            Bool,
            "/target_detected",
            self.detected_callback,
            10
        )

        # Etat de detection
        self.target_detected = False
        self.previous_detected = False

        # Pour eviter plusieurs appels en meme temps
        self.request_in_progress = False

        # Nombre de missions deja lancees
        self.mission_count = 0
        self.max_missions = 3

        # Pour lancer la premiere mission automatiquement
        self.first_request_sent = False

        # Timer pour surveiller et decider quand envoyer la requete
        self.timer = self.create_timer(0.2, self.main_loop)

        self.get_logger().info("Mission client node started")

    def detected_callback(self, msg):
        # Mise a jour de la valeur de detection
        self.target_detected = msg.data

    def main_loop(self):
        # Si on a deja termine les 3 missions, on ne fait plus rien
        if self.mission_count >= self.max_missions:
            return

        # On attend que le service soit disponible
        if not self.client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info("Waiting for /reset_mission service...")
            return

        # Si une requete est deja en cours, on attend
        if self.request_in_progress:
            return

        # Premiere mission : on envoie une requete une seule fois
        if not self.first_request_sent:
            self.send_request()
            self.first_request_sent = True
            return

        # Pour les missions suivantes :
        # on envoie une nouvelle requete seulement quand on detecte
        # le passage de False vers True sur /target_detected
        if self.target_detected and not self.previous_detected:
            self.send_request()

        # On memorise l'etat precedent
        self.previous_detected = self.target_detected

    def send_request(self):
        # Si on a deja atteint 3 missions, on ne fait plus rien
        if self.mission_count >= self.max_missions:
            return

        request = ResetMission.Request()
        request.target_x = 0.0
        request.target_y = 0.0
        request.random_target = True

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

        self.request_in_progress = True
        self.mission_count += 1

        self.get_logger().info(
            f"Sending reset request number {self.mission_count}"
        )

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Response: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        # On libere l'etat pour permettre une future requete
        self.request_in_progress = False

        # On remet la detection a False pour attendre la prochaine vraie detection
        self.target_detected = False
        self.previous_detected = False


def main(args=None):
    rclpy.init(args=args)

    node = MissionClient()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()