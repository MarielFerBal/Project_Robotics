#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class SimpleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('simple_avoidance_node')

        # Publicador de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscriptor al LiDAR o sensor
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Timer para mover el robot
        self.timer = self.create_timer(0.1, self.move_robot)

        # Variables de estado
        self.obstacle_ahead = False
        self.turning = False
        self.safe_distance = 0.5  # metros

        self.get_logger().info('Nodo de evasión simple iniciado...')

    def scan_callback(self, msg: LaserScan):
        """Analiza el rango frontal del LiDAR"""
        # Tomamos una sección frontal del escaneo (ej. ±15°)
        ranges = msg.ranges
        total = len(ranges)
        front_angles = ranges[total//2 - 15 : total//2 + 15]

        # Filtra valores válidos
        valid = [r for r in front_angles if not math.isinf(r)]

        if valid:
            min_distance = min(valid)
            self.obstacle_ahead = min_distance < self.safe_distance
            if self.obstacle_ahead:
                self.get_logger().info(f'⚠️ Obstáculo detectado a {min_distance:.2f} m')
        else:
            self.obstacle_ahead = False

    def move_robot(self):
        msg = Twist()

        if self.obstacle_ahead:
            # Si hay obstáculo, girar en el sitio
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.turning = True
        else:
            # Si el camino está libre, avanzar
            msg.linear.x = 0.2
            msg.angular.z = 0.0
            self.turning = False

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()