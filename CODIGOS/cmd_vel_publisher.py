#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class CmdVelFromSim(Node):
    def __init__(self):
        super().__init__('cmd_vel_from_sim')
        
        # Publicador cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscriptor a la odometría
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Velocidad obtenida de la simulación
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # Timer para publicar cmd_vel
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)
        self.get_logger().info('Nodo cmd_vel_from_sim iniciado...')

    def odom_callback(self, msg: Odometry):
        # Obtenemos velocidades lineal y angular directamente de Odometry
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

    def publish_cmd_vel(self):
        msg = Twist()
        # Usamos las velocidades obtenidas del simulador
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Publicando cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelFromSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
