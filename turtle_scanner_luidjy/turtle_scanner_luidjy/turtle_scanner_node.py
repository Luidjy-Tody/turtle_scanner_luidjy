#!/usr/bin/env python3

# Calculs mathematiques
import math
import random
import time

# Librairies ROS2
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# Messages utilises
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn

# Service personnalise
from turtle_interfaces.srv import ResetMission


class TurtleScannerNode(Node):
    def __init__(self):
        # Nom du noeud
        super().__init__("turtle_scanner_node")

        # Variables pour stocker les poses
        self.pose_scanner = None
        self.pose_target = None

        # Subscribers sur les poses
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

        # Publisher pour la detection
        self.detected_publisher = self.create_publisher(
            Bool,
            "/target_detected",
            10
        )

        # Clients de services turtlesim
        self.spawn_client = self.create_client(Spawn, "/spawn")
        self.kill_client = self.create_client(Kill, "/kill")

        # Service reset mission
        self.reset_service = self.create_service(
            ResetMission,
            "/reset_mission",
            self.reset_mission_callback
        )

        # Parametres du serpentin
        self.nb_lignes = 5
        self.y_start = 1.0
        self.y_step = 2.0
        self.x_min = 1.0
        self.x_max = 10.0

        # Parametres de vitesse
        self.linear_speed = 2.0
        self.angular_speed = 1.5

        # Parametres de regulation
        self.waypoint_tolerance = 0.3
        self.Kp_ang = 5.0
        self.Kp_lin = 1.0

        # Rayon de detection
        self.detection_radius = 1.5

        # Etat de detection
        self.target_detected = False
        self.detected_message_sent = False

        # Nom de la tortue cible
        self.target_name = "turtle_target"

        # Waypoints du trajet
        self.waypoints = []
        self.current_waypoint_index = 0
        self.scan_finished = False
        self.finish_message_sent = False

        # Generation des waypoints
        self.generate_serpentine_waypoints()

        # Timer ROS2
        self.timer = self.create_timer(0.05, self.scan_step)

        # Au debut on publie False
        self.publish_detected_state(False)

        self.get_logger().info("Turtle scanner node started")
        self.get_logger().info(f"Waypoints generated: {self.waypoints}")

    def scanner_pose_callback(self, msg):
        # Mise a jour de la pose du scanner
        self.pose_scanner = msg

    def target_pose_callback(self, msg):
        # Mise a jour de la pose de la cible
        self.pose_target = msg

    def generate_serpentine_waypoints(self):
        # Generation des waypoints du serpentin
        self.waypoints = []

        for i in range(self.nb_lignes):
            y = self.y_start + i * self.y_step

            if i % 2 == 0:
                self.waypoints.append([self.x_min, y])
                self.waypoints.append([self.x_max, y])
            else:
                self.waypoints.append([self.x_max, y])
                self.waypoints.append([self.x_min, y])

        self.current_waypoint_index = 0
        self.scan_finished = False
        self.finish_message_sent = False

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

    def publish_detected_state(self, state):
        # Publication sur /target_detected
        msg = Bool()
        msg.data = state
        self.detected_publisher.publish(msg)

    def check_target_detection(self):
        # Si les poses ne sont pas encore disponibles
        if self.pose_scanner is None or self.pose_target is None:
            return False

        scanner_point = [self.pose_scanner.x, self.pose_scanner.y]
        target_point = [self.pose_target.x, self.pose_target.y]

        distance_target = self.compute_distance(scanner_point, target_point)

        if distance_target < self.detection_radius:
            self.target_detected = True
            self.stop_turtle()
            self.publish_detected_state(True)

            if not self.detected_message_sent:
                self.get_logger().info(
                    f"Cible detectee a ({self.pose_target.x:.2f}, {self.pose_target.y:.2f}) !"
                )
                self.detected_message_sent = True

            return True

        self.publish_detected_state(False)
        return False

    def wait_future(self, future, timeout_sec=2.0):
        # Petite attente pour laisser le service repondre
        end_time = time.time() + timeout_sec

        while rclpy.ok() and not future.done() and time.time() < end_time:
            time.sleep(0.05)

        return future.done()

    def kill_target(self):
        # Suppression de l'ancienne cible
        if not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /kill service...")
            return False

        request = Kill.Request()
        request.name = self.target_name

        future = self.kill_client.call_async(request)
        self.wait_future(future, 1.0)

        # Meme si la tortue n'existe pas encore, on ne bloque pas la suite
        return True

    def spawn_target(self, x, y):
        # Spawn d'une nouvelle cible
        if not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn service...")
            return False

        request = Spawn.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = 0.0
        request.name = self.target_name

        future = self.spawn_client.call_async(request)

        if not self.wait_future(future, 2.0):
            return False

        try:
            future.result()
            return True
        except Exception as e:
            self.get_logger().error(f"Spawn failed: {e}")
            return False

    def reset_internal_state(self):
        # Remise a zero pour recommencer une mission
        self.pose_target = None
        self.target_detected = False
        self.detected_message_sent = False
        self.generate_serpentine_waypoints()
        self.publish_detected_state(False)

    def reset_mission_callback(self, request, response):
        # Service /reset_mission
        # Si random_target = True, on choisit une position aleatoire
        target_x = request.target_x
        target_y = request.target_y

        if request.random_target:
            target_x = random.uniform(1.0, 10.0)
            target_y = random.uniform(1.0, 10.0)

        # On arrete turtle1
        self.stop_turtle()

        # On supprime l'ancienne cible
        self.kill_target()

        # On spawne la nouvelle cible
        spawn_success = self.spawn_target(target_x, target_y)

        if not spawn_success:
            response.success = False
            response.message = "Mission reset failed: impossible to spawn target"
            return response

        # On reinitialise le serpentin et la detection
        self.reset_internal_state()

        response.success = True
        response.message = f"Mission reset with target at ({target_x:.2f}, {target_y:.2f})"
        self.get_logger().info(response.message)

        return response

    def scan_step(self):
        # Methode appelee par le timer

        if self.pose_scanner is None:
            return

        if self.target_detected:
            self.stop_turtle()
            self.publish_detected_state(True)
            return

        if self.check_target_detection():
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_turtle()
            self.scan_finished = True

            if not self.finish_message_sent:
                self.get_logger().info("Balayage termine")
                self.finish_message_sent = True

            return

        current_point = [self.pose_scanner.x, self.pose_scanner.y]
        target_point = self.waypoints[self.current_waypoint_index]

        distance_error = self.compute_distance(current_point, target_point)

        if distance_error < self.waypoint_tolerance:
            self.get_logger().info(f"Waypoint atteint : {target_point}")
            self.current_waypoint_index += 1
            return

        desired_theta = self.compute_angle(current_point, target_point)

        heading_error = math.atan(
            math.tan((desired_theta - self.pose_scanner.theta) / 2.0)
        )

        angular_command = self.Kp_ang * heading_error
        linear_command = self.Kp_lin * distance_error

        if linear_command > self.linear_speed:
            linear_command = self.linear_speed

        if angular_command > self.angular_speed:
            angular_command = self.angular_speed
        elif angular_command < -self.angular_speed:
            angular_command = -self.angular_speed

        msg = Twist()
        msg.linear.x = linear_command
        msg.angular.z = angular_command
        self.cmd_publisher.publish(msg)


def main(args=None):
    # Initialisation ROS2
    rclpy.init(args=args)

    # Creation du noeud
    node = TurtleScannerNode()

    # Executor multi-thread pour laisser tourner timer + service
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()