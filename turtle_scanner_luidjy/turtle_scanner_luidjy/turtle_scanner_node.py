#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleScannerNode(Node):
    def __init__(self):
        # Nom du noeud
        super().__init__("turtle_scanner_node")

        # Variables pour stocker les poses
        self.pose_scanner = None
        self.pose_target = None

        # Subscribers sur les poses des deux tortues
        self.subscriber_scanner = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.scanner_pose_callback,
            10
        )

        self.subscriber_target = self.create_subscription(
            Pose,
            "/turtle_target/pose",
            self.target_pose_callback,
            10
        )

        # Publisher pour commander turtle1
        self.cmd_publisher = self.create_publisher(
            Twist,
            "/turtle1/cmd_vel",
            10
        )

        # Parametres du serpentin donnes dans l'enonce
        self.nb_lignes = 5
        self.y_start = 1.0
        self.y_step = 2.0
        self.x_min = 1.0
        self.x_max = 10.0

        # Parametres de vitesse du tableau
        self.linear_speed = 2.0
        self.angular_speed = 1.5

        # Parametres de regulation
        self.waypoint_tolerance = 0.3
        self.Kp_ang = 5.0
        self.Kp_lin = 1.0

        # Waypoints du trajet
        self.waypoints = []
        self.current_waypoint_index = 0
        self.scan_finished = False
        self.finish_message_sent = False

        # Generation des waypoints
        self.generate_serpentine_waypoints()

        # Timer ROS2 a 20 Hz
        self.timer = self.create_timer(0.05, self.scan_step)

        self.get_logger().info("Turtle scanner node started")
        self.get_logger().info(f"Waypoints generated: {self.waypoints}")

    def scanner_pose_callback(self, msg):
        # Mise a jour de la pose du scanner
        self.pose_scanner = msg

    def target_pose_callback(self, msg):
        # Mise a jour de la pose de la cible
        self.pose_target = msg

    def generate_serpentine_waypoints(self):
        # Generation programmatique des waypoints du serpentin
        self.waypoints = []

        for i in range(self.nb_lignes):
            y = self.y_start + i * self.y_step

            # Ligne paire : gauche vers droite
            if i % 2 == 0:
                self.waypoints.append([self.x_min, y])
                self.waypoints.append([self.x_max, y])

            # Ligne impaire : droite vers gauche
            else:
                self.waypoints.append([self.x_max, y])
                self.waypoints.append([self.x_min, y])

    def compute_angle(self, point_a, point_b):
        # Calcul de l'angle entre A et B
        return math.atan2(point_b[1] - point_a[1], point_b[0] - point_a[0])

    def compute_distance(self, point_a, point_b):
        # Calcul de la distance euclidienne entre A et B
        return math.sqrt(
            (point_b[0] - point_a[0]) ** 2 +
            (point_b[1] - point_a[1]) ** 2
        )

    def stop_turtle(self):
        # Arret de la tortue
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_publisher.publish(msg)

    def scan_step(self):
        # Cette methode est appelee periodiquement par le timer

        # Si on ne connait pas encore la position de turtle1
        if self.pose_scanner is None:
            return

        # Si tous les waypoints sont finis
        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_turtle()
            self.scan_finished = True

            if not self.finish_message_sent:
                self.get_logger().info("Balayage termine")
                self.finish_message_sent = True

            return

        # Position actuelle de turtle1
        current_point = [self.pose_scanner.x, self.pose_scanner.y]

        # Waypoint actuel
        target_point = self.waypoints[self.current_waypoint_index]

        # Distance au waypoint
        distance_error = self.compute_distance(current_point, target_point)

        # Si on est proche du waypoint, on passe au suivant
        if distance_error < self.waypoint_tolerance:
            self.get_logger().info(f"Waypoint atteint : {target_point}")
            self.current_waypoint_index += 1
            return

        # Angle desire vers le waypoint
        desired_theta = self.compute_angle(current_point, target_point)

        # Erreur angulaire demandee dans l'enonce
        heading_error = math.atan(
            math.tan((desired_theta - self.pose_scanner.theta) / 2.0)
        )

        # Commandes proportionnelles
        angular_command = self.Kp_ang * heading_error
        linear_command = self.Kp_lin * distance_error

        # Limitation de la vitesse lineaire
        if linear_command > self.linear_speed:
            linear_command = self.linear_speed

        # Limitation de la vitesse angulaire
        if angular_command > self.angular_speed:
            angular_command = self.angular_speed
        elif angular_command < -self.angular_speed:
            angular_command = -self.angular_speed

        # Publication de la commande
        msg = Twist()
        msg.linear.x = linear_command
        msg.angular.z = angular_command
        self.cmd_publisher.publish(msg)


def main(args=None):
    # Initialisation ROS2
    rclpy.init(args=args)

    # Creation du noeud
    node = TurtleScannerNode()

    # Boucle ROS2
    rclpy.spin(node)

    # Fermeture propre
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()