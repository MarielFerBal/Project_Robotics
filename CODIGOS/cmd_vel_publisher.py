#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # segundos
        self.timer = self.create_timer(timer_period, self.publish_cmd_vel)
        self.get_logger().info('Nodo de cmd_vel iniciado...')

    def publish_cmd_vel(self):
        msg = Twist()
        # Velocidad lineal (x = adelante)
        msg.linear.x = 0.2
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # Velocidad angular (z = giro)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.3

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando: lineal={msg.linear.x}, angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido manualmente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()