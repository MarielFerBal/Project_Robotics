#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class Smoth(Node):
    def __init__(self):
        super().__init__('smoth')

        # Suscripción al Velocity Policy
        self.subscription = self.create_subscription(
            Twist,
            '/velocity_cmd',       # Nodo Velocity Policy
            self.velocity_callback,
            10
        )

        # Suscripción al Safety (solo flag de emergencia)
        self.safety_sub = self.create_subscription(
            Bool,
            '/safety_stop',        # Safety envía True si hay emergencia
            self.safety_callback,
            10
        )

        # Publicador final al motor
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables para suavizar
        self.current_velocity = Twist()
        self.target_velocity = Twist()
        self.smoothing_factor = 0.1  # 0.1 = suave, 1 = instantáneo

        # Flag de emergencia
        self.emergency_stop = False

    def velocity_callback(self, msg):
        """Recibe velocidad objetivo de Velocity Policy"""
        self.target_velocity = msg
        self.smooth_and_publish()

    def safety_callback(self, msg):
        """Recibe flag de emergencia de Safety"""
        self.emergency_stop = msg.data
        self.smooth_and_publish()

    def smooth_and_publish(self):
        """Aplica suavizado y publica velocidad final"""
        final_velocity = Twist()

        if self.emergency_stop:
            # Prioridad: parada de emergencia
            final_velocity.linear.x = 0.0
            final_velocity.angular.z = 0.0
            self.current_velocity = final_velocity  # reset para suavizado
        else:
            # Suavizado lineal
            self.current_velocity.linear.x += self.smoothing_factor * (self.target_velocity.linear.x - self.current_velocity.linear.x)
            self.current_velocity.angular.z += self.smoothing_factor * (self.target_velocity.angular.z - self.current_velocity.angular.z)
            final_velocity = self.current_velocity

        # Publicar velocidad final al motor
        self.publisher_.publish(final_velocity)

def main(args=None):
    rclpy.init(args=args)
    node = Smoth()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
